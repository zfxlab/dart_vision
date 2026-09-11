import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from dart_interfaces.msg import (
    VisibilityEvidence,
    ObservationWindow,
    BaseReference,
    LidarObservation,
    CameraObservation,
    TargetEstimation,
)
from .core import VisibilityWindow, fresh, point, ray_rail
from .ros_geometry import seconds, xyz, transform, pose_matrix, fill_point


class Estimator(Node):
    def __init__(self):
        super().__init__("target_estimation")
        defaults = dict(
            fixed_frame="dart_base_link",
            base_frame="base_link",
            rail_frame="rail_origin_link",
            module_frame="dart_detection_module_link",
            light_frame="green_light_link",
            base_aim_point=[0.0, 0.0, 0.91],
            center_q=0.28,
            observation_timeout=0.25,
            reference_timeout=1.0,
            open_confirm=0.25,
            close_confirm=0.2,
            visibility_timeout=0.3,
            ray_residual=0.06,
            sensor_disagreement=0.10,
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.window = VisibilityWindow(
            self.p("open_confirm"),
            self.p("close_confirm"),
            self.p("visibility_timeout"),
        )
        self.reference = self.lidar = self.camera = None
        self.last_now = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.window_pub = self.create_publisher(
            ObservationWindow, "observation_window", latched
        )
        self.pub = self.create_publisher(TargetEstimation, "target_estimation", 10)
        self.create_subscription(
            VisibilityEvidence,
            "lidar/visibility",
            self.evidence,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BaseReference, "lidar/base_reference", self.on_reference, latched
        )
        self.create_subscription(
            LidarObservation,
            "lidar/observation",
            self.on_lidar,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraObservation,
            "camera/observation",
            self.on_camera,
            qos_profile_sensor_data,
        )
        self.create_timer(0.02, self.tick)

    def evidence(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        if fresh(seconds(m.header.stamp), now, self.p("visibility_timeout")):
            self.window.observe(m.state, seconds(m.header.stamp))

    def on_reference(self, m):
        if m.header.frame_id == self.p("fixed_frame"):
            self.reference = m

    def on_lidar(self, m):
        self.lidar = m

    def on_camera(self, m):
        self.camera = m

    def lookup(self, parent, child, stamp=None):
        return transform(
            self.tf.lookup_transform(
                parent, child, Time() if stamp is None else Time.from_msg(stamp)
            ).transform
        )

    def tick(self):
        clock = self.get_clock().now()
        now = clock.nanoseconds * 1e-9
        if self.last_now is not None and now < self.last_now:
            # Preserve monotonic epoch numbers across bag / simulation clock resets.
            epoch = self.window.epoch
            self.window = VisibilityWindow(
                self.p("open_confirm"),
                self.p("close_confirm"),
                self.p("visibility_timeout"),
            )
            self.window.epoch = epoch
            self.reference = self.lidar = self.camera = None
        self.last_now = now
        w = ObservationWindow()
        w.header.stamp = clock.to_msg()
        w.state = self.window.tick(now)
        w.epoch = self.window.epoch
        w.opened_at = Time(seconds=self.window.opened_at).to_msg()
        self.window_pub.publish(w)
        out = TargetEstimation()
        out.header.stamp = clock.to_msg()
        out.header.frame_id = self.p("fixed_frame")
        out.epoch = w.epoch
        out.visibility = w.state
        ref = self.reference
        try:
            if (
                ref is None
                or ref.epoch != w.epoch
                or not fresh(
                    seconds(ref.header.stamp), now, self.p("reference_timeout")
                )
            ):
                self.pub.publish(out)
                return
            base = pose_matrix(ref.pose)
            rail = base @ self.lookup(self.p("base_frame"), self.p("rail_frame"))
            light_offset = self.lookup(self.p("module_frame"), self.p("light_frame"))[
                :3, 3
            ]
            zero = point(rail, light_offset)
            axis = rail[:3, 0]
            fill_point(out.base_center, point(base, self.p("base_aim_point")))
            fill_point(out.center_light, zero + self.p("center_q") * axis)
            out.base_valid = True
            out.base_source = ref.source
            out.reference_revision = ref.revision
            lidar_position = camera_position = None
            lm, cm = self.lidar, self.camera
            if (
                lm is not None
                and lm.valid
                and lm.epoch == w.epoch
                and lm.reference_revision == ref.revision
                and lm.header.frame_id == self.p("fixed_frame")
                and fresh(seconds(lm.header.stamp), now, self.p("observation_timeout"))
                and seconds(lm.window_start) >= self.window.opened_at
                and 0 <= lm.rail_position_m <= 0.56
            ):
                lidar_position = zero + lm.rail_position_m * axis
            if (
                cm is not None
                and cm.status_code == CameraObservation.STATUS_OK
                and fresh(seconds(cm.header.stamp), now, self.p("observation_timeout"))
                and seconds(cm.header.stamp) >= self.window.opened_at
            ):
                try:
                    camera_tf = self.lookup(
                        self.p("fixed_frame"), cm.header.frame_id, cm.header.stamp
                    )
                    camera_position = ray_rail(
                        camera_tf[:3, 3],
                        camera_tf[:3, :3] @ xyz(cm.bearing),
                        zero,
                        axis,
                        max_residual=self.p("ray_residual"),
                    )
                except TransformException:
                    pass
            out.sensor_conflict = bool(
                lidar_position is not None
                and camera_position is not None
                and np.linalg.norm(lidar_position - camera_position)
                > self.p("sensor_disagreement")
            )
            if w.state == ObservationWindow.CLEAR and not out.sensor_conflict:
                if lidar_position is not None:
                    out.light_valid = True
                    out.light_source = out.LIDAR
                    out.light_stamp = lm.header.stamp
                    fill_point(out.light_center, lidar_position)
                elif camera_position is not None:
                    out.light_valid = True
                    out.light_source = out.CAMERA_GEOMETRY
                    out.light_stamp = cm.header.stamp
                    fill_point(out.light_center, camera_position)
        except (TransformException, ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().debug(str(error))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Estimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
