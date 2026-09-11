import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from dart_interfaces.msg import (
    VisibilityEvidence,
    ObservationWindow,
    BaseReference,
    LidarObservation,
)
from .core import (
    Accumulator,
    voxel,
    crop,
    read_pcd,
    read_stl,
    fit_base,
    fit_module,
    visibility,
)


def sec(t):
    return t.sec + t.nanosec * 1e-9


def transform(t):
    out = np.eye(4)
    q = t.rotation
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return out


def apply(t, p):
    return np.asarray(p) @ t[:3, :3].T + t[:3, 3]


class Lidar(Node):
    def __init__(self):
        super().__init__("lidar")
        defaults = dict(
            fixed_frame="dart_base_link",
            base_frame="base_link",
            rail_frame="rail_origin_link",
            lidar_frame="livox_frame",
            cloud_topic="livox/lidar",
            timestamp_source="receive",
            sensor_timeout=0.3,
            base_model="",
            module_model="",
            base_window=0.8,
            module_window=0.12,
            initialization_timeout=3.0,
            initialization_min_span=0.5,
            base_max_points=15000,
            module_max_points=3000,
            base_max_shift=0.3,
            base_threshold=0.08,
            base_min_support=0.65,
            base_min_points=80,
            base_confirmations=2,
            base_consistency=0.03,
            module_threshold=0.035,
            module_min_points=12,
            module_min_support=0.55,
            module_step=0.01,
            module_ambiguity=0.002,
            background_distance=0.025,
            near_lower=[0.3, -0.6, -0.6],
            near_upper=[2.0, 0.6, 0.6],
            min_near_points=15,
            min_far_points=10,
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value
        for name in (
            "sensor_timeout",
            "base_window",
            "module_window",
            "initialization_timeout",
            "initialization_min_span",
            "base_max_points",
            "module_max_points",
            "base_max_shift",
            "base_threshold",
            "base_min_points",
            "base_confirmations",
            "base_consistency",
            "module_threshold",
            "module_min_points",
            "module_step",
        ):
            if not np.isfinite(self.p(name)) or self.p(name) <= 0:
                raise ValueError(name + " must be positive")
        for name in ("base_min_support", "module_min_support"):
            if not 0 < self.p(name) <= 1:
                raise ValueError(name + " must be in (0,1]")
        low, high = np.asarray(self.p("near_lower")), np.asarray(self.p("near_upper"))
        if (
            low.shape != (3,)
            or high.shape != (3,)
            or not np.all(np.isfinite([low, high]))
            or not np.all(low < high)
        ):
            raise ValueError("Invalid near-field box")
        if self.p("timestamp_source") not in ("receive", "header"):
            raise ValueError("Invalid timestamp_source")
        self.base_model = voxel(read_pcd(self.p("base_model")))
        self.base_tree = cKDTree(self.base_model)
        self.module_model = voxel(read_stl(self.p("module_model")), size=0.008)
        self.base_acc = Accumulator(self.p("base_window"), self.p("base_max_points"))
        self.module_acc = Accumulator(
            self.p("module_window"), self.p("module_max_points")
        )
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.window = None
        self.epoch = 0
        self.revision = 0
        self.source = 0
        self.finalized = False
        self.base = None
        self.prior = None
        self.rail_local = None
        self.last_cloud = None
        self.candidate = None
        self.confirmations = 0
        self.rmse = 0.0
        self.support = 0.0
        self.last_fit = 0.0
        self.last_tick = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub = self.create_publisher(
            BaseReference, "lidar/base_reference", latched
        )
        self.evidence_pub = self.create_publisher(
            VisibilityEvidence, "lidar/visibility", qos_profile_sensor_data
        )
        self.module_pub = self.create_publisher(
            LidarObservation, "lidar/observation", qos_profile_sensor_data
        )
        self.create_subscription(
            ObservationWindow, "observation_window", self.on_window, latched
        )
        self.create_subscription(
            PointCloud2, self.p("cloud_topic"), self.cloud, qos_profile_sensor_data
        )
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            "Lidar uses %s timestamps; calibrate door ROI before field use."
            % self.p("timestamp_source")
        )

    def lookup(self, parent, child):
        return transform(self.tf.lookup_transform(parent, child, Time()).transform)

    def on_window(self, m):
        if m.epoch != self.epoch:
            self.get_logger().info(
                f"Starting observation epoch {m.epoch}; resetting to startup prior"
            )
            self.epoch = m.epoch
            self.revision = 0
            self.source = 0
            self.finalized = False
            self.base = None if self.prior is None else self.prior.copy()
            self.base_acc.clear()
            self.module_acc.clear()
            self.candidate = None
            self.confirmations = 0
            self.last_fit = 0.0
            self.rmse = 0.0
            self.support = 0.0
        if m.state != ObservationWindow.CLEAR:
            self.base_acc.clear()
            self.module_acc.clear()
        self.window = m

    def reference(self):
        if self.base is None:
            return
        m = BaseReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.p("fixed_frame")
        m.epoch = self.epoch
        m.revision = self.revision
        m.source = self.source
        m.finalized = self.finalized
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(
            float, self.base[:3, 3]
        )
        q = Rotation.from_matrix(self.base[:3, :3]).as_quat()
        (
            m.pose.orientation.x,
            m.pose.orientation.y,
            m.pose.orientation.z,
            m.pose.orientation.w,
        ) = map(float, q)
        m.rmse_m = self.rmse
        m.support = self.support
        self.ref_pub.publish(m)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_tick is not None and now < self.last_tick:
            self.base_acc.clear()
            self.module_acc.clear()
            self.last_cloud = None
            self.window = None
        self.last_tick = now
        try:
            if self.prior is None:
                self.prior = self.lookup(self.p("fixed_frame"), self.p("base_frame"))
                self.rail_local = self.lookup(
                    self.p("base_frame"), self.p("rail_frame")
                )
                self.base = self.prior.copy()
        except TransformException:
            return
        w = self.window
        if (
            w is not None
            and w.state == 2
            and not self.finalized
            and now - sec(w.opened_at) > self.p("initialization_timeout")
        ):
            self.finalized = True
            self.get_logger().info(
                f"Epoch {self.epoch}: initialization timed out; using startup prior"
            )
        self.reference()
        if self.last_cloud is None or not 0 <= now - self.last_cloud <= self.p(
            "sensor_timeout"
        ):
            m = VisibilityEvidence()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.p("lidar_frame")
            self.evidence_pub.publish(m)

    def cloud(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = now if self.p("timestamp_source") == "receive" else sec(m.header.stamp)
        if not 0 <= now - stamp <= self.p("sensor_timeout"):
            return
        if self.last_cloud is not None and stamp <= self.last_cloud:
            return
        self.last_cloud = stamp
        try:
            if self.prior is None:
                return
            data = point_cloud2.read_points(
                m, field_names=("x", "y", "z"), skip_nans=True
            )
            if isinstance(data, np.ndarray) and data.dtype.names:
                points = np.column_stack([data[x].reshape(-1) for x in ("x", "y", "z")])
            else:
                points = np.asarray(list(data), dtype=float).reshape(-1, 3)
            points = voxel(points, size=0.015, limit=25000)
            # Fixed sensors only. Reject a mismatched cloud frame instead of rotating with camera yaw.
            if m.header.frame_id != self.p("lidar_frame"):
                return
            world = apply(self.lookup(self.p("fixed_frame"), m.header.frame_id), points)
            local = apply(np.linalg.inv(self.prior), world)
            far = crop(
                local,
                self.base_model.min(axis=0) - 0.4,
                self.base_model.max(axis=0) + 0.4,
            )
            state, near_count, far_count = visibility(
                points,
                np.array(self.p("near_lower")),
                np.array(self.p("near_upper")),
                len(far),
                self.p("min_near_points"),
                self.p("min_far_points"),
            )
            evidence = VisibilityEvidence()
            evidence.header.stamp = Time(seconds=stamp).to_msg()
            evidence.header.frame_id = m.header.frame_id
            evidence.state = state
            evidence.near_points = near_count
            evidence.far_points = far_count
            evidence.quality = 1.0 if state else 0.0
            self.evidence_pub.publish(evidence)
            w = self.window
            if (
                w is None
                or w.state != 2
                or not 0 <= now - sec(w.header.stamp) <= 0.3
                or stamp < sec(w.opened_at)
            ):
                return
            if not self.finalized:
                self.base_acc.add(stamp, far)
                accumulated, start = self.base_acc.get()
                if (
                    stamp - start >= self.p("initialization_min_span")
                    and stamp - self.last_fit >= 0.25
                ):
                    self.last_fit = stamp
                    fit = fit_base(
                        accumulated,
                        self.base_model,
                        self.p("base_max_shift"),
                        self.p("base_threshold"),
                        self.p("base_min_support"),
                        self.p("base_min_points"),
                    )
                    if fit is not None:
                        shift, error, support = fit
                        self.confirmations = (
                            self.confirmations + 1
                            if self.candidate is not None
                            and np.linalg.norm(shift - self.candidate)
                            < self.p("base_consistency")
                            else 1
                        )
                        self.candidate = shift
                        # Independent accumulation windows for confirmation.
                        self.base_acc.clear()
                        if self.confirmations >= self.p("base_confirmations"):
                            self.base = self.prior.copy()
                            self.base[:3, 3] += self.prior[:3, :3] @ shift
                            self.revision += 1
                            self.source = 1
                            self.finalized = True
                            self.rmse = error
                            self.support = support
                            self.module_acc.clear()
                            self.get_logger().info(
                                f"Epoch {self.epoch}: accepted base reference, RMSE={error:.4f} m, support={support:.3f}"
                            )
                            self.reference()
                    else:
                        self.confirmations = 0
                        self.candidate = None
            rail = self.base @ self.rail_local
            base_points = apply(np.linalg.inv(self.base), world)
            background_distance, _ = self.base_tree.query(base_points)
            foreground = world[background_distance > self.p("background_distance")]
            rail_points = apply(np.linalg.inv(rail), foreground)
            low = self.module_model.min(axis=0) - 0.05
            high = self.module_model.max(axis=0) + np.array([0.61, 0.05, 0.05])
            self.module_acc.add(stamp, crop(rail_points, low, high))
            accumulated, start = self.module_acc.get()
            fit = fit_module(
                accumulated,
                self.module_model,
                self.p("module_step"),
                self.p("module_threshold"),
                self.p("module_min_points"),
                self.p("module_min_support"),
                self.p("module_ambiguity"),
            )
            out = LidarObservation()
            out.header.stamp = Time(seconds=stamp).to_msg()
            out.header.frame_id = self.p("fixed_frame")
            out.window_start = Time(seconds=start).to_msg()
            out.epoch = self.epoch
            out.reference_revision = self.revision
            if fit is not None:
                out.valid = True
                out.rail_position_m, out.rmse_m, out.support = fit
            self.module_pub.publish(out)
        except (TransformException, ValueError, TypeError) as error:
            self.get_logger().warning(
                "Cloud rejected: " + str(error), throttle_duration_sec=2.0
            )


def main():
    rclpy.init()
    node = Lidar()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
