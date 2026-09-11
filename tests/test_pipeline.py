"""Run real node callbacks with an in-process ROS/TF test harness.
This exercises lifecycle/message flow, not DDS or ROS binary compatibility.
"""

import importlib
import sys
import types
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
for package in ("dart_lidar", "dart_target_estimation", "dart_aiming"):
    sys.path.insert(0, str(ROOT / "src" / package))
from dart_lidar.core import voxel, read_pcd, read_stl


class Harness:
    def __init__(self):
        self.now = 100.0
        self.callbacks = {}
        self.outputs = {}
        self.saved = {}
        self.overrides = {
            "base_model": str(ROOT / "src/dart_bringup/models/pcd/base_pnx_5mm.pcd"),
            "module_model": str(
                ROOT
                / "third_party/dart_description/meshes/pnx_dart_detection_module_link.stl"
            ),
        }
        self.frames = {"dart_base_link": np.eye(4), "launcher_frame": np.eye(4)}
        base = np.eye(4)
        base[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
        base[:3, 3] = [25, 0, 0]
        self.frames["base_link"] = base
        rail = np.eye(4)
        rail[:3, 3] = [-0.28, -0.03193, 0.91]
        self.frames["rail_origin_link"] = base @ rail
        self.frames["dart_detection_module_link"] = base @ rail
        light = np.eye(4)
        light[:3, 3] = [0, 0.1222, 0.02363]
        self.frames["green_light_link"] = base @ rail @ light
        self.frames["livox_frame"] = np.eye(4)
        camera = np.eye(4)
        camera[:3, 3] = [0.22, 0, 0.62]
        camera[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        self.frames["camera_optical_frame"] = camera
        self.install()

    def install_module(self, name, **attrs):
        self.saved[name] = sys.modules.get(name)
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    def install(self):
        h = self

        def vec():
            return types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

        def stamp():
            return types.SimpleNamespace(sec=0, nanosec=0)

        def header():
            return types.SimpleNamespace(stamp=stamp(), frame_id="")

        primitives = {
            "std_msgs/Header": header,
            "builtin_interfaces/Time": stamp,
            "geometry_msgs/Point": vec,
            "geometry_msgs/Vector3": vec,
            "geometry_msgs/Pose": lambda: types.SimpleNamespace(
                position=vec(),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            ),
        }
        classes = {}
        for path in (ROOT / "src/dart_interfaces/msg").glob("*.msg"):
            fields = []
            const = {}
            for raw in path.read_text(encoding="utf-8").splitlines():
                raw = raw.split("#")[0].strip()
                if not raw:
                    continue
                kind, name = raw.split()
                if "=" in name:
                    key, value = name.split("=")
                    const[key] = int(value)
                    continue
                ctor = primitives.get(
                    kind,
                    lambda k=kind: (
                        False if k == "bool" else (0.0 if k.startswith("float") else 0)
                    ),
                )
                fields.append((name, ctor))

            def init(obj, fields=fields):
                for name, ctor in fields:
                    setattr(obj, name, ctor())

            classes[path.stem] = type(path.stem, (), dict(const, __init__=init))
        self.msg = types.SimpleNamespace(**classes)
        self.install_module("dart_interfaces")
        self.install_module("dart_interfaces.msg", **classes)

        class Time:
            def __init__(self, seconds=0, **kw):
                self.nanoseconds = int(seconds * 1e9)

            @classmethod
            def from_msg(cls, m):
                return cls(m.sec + m.nanosec * 1e-9)

            def to_msg(self):
                return types.SimpleNamespace(
                    sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9
                )

        self.Time = Time

        class Node:
            def __init__(self, name):
                self.params = {}
                self.name = name

            def declare_parameter(self, k, v):
                self.params[k] = h.overrides.get(k, v)

            def get_parameter(self, k):
                return types.SimpleNamespace(value=self.params[k])

            def get_clock(self):
                return types.SimpleNamespace(now=lambda: Time(h.now))

            def get_logger(self):
                return types.SimpleNamespace(
                    info=lambda *a, **k: None,
                    warning=lambda *a, **k: None,
                    debug=lambda *a, **k: None,
                )

            def create_timer(self, *a):
                pass

            def create_subscription(self, cls, topic, cb, qos):
                h.callbacks.setdefault(topic, []).append(cb)

            def create_publisher(self, cls, topic, qos):
                def publish(m):
                    h.outputs[topic] = m
                    for cb in h.callbacks.get(topic, []):
                        cb(m)

                return types.SimpleNamespace(publish=publish)

        class Buffer:
            def lookup_transform(self, parent, child, time):
                t = np.linalg.inv(h.frames[parent]) @ h.frames[child]
                q = Rotation.from_matrix(t[:3, :3]).as_quat()
                return types.SimpleNamespace(
                    header=types.SimpleNamespace(stamp=Time(h.now).to_msg()),
                    transform=types.SimpleNamespace(
                        translation=types.SimpleNamespace(**dict(zip("xyz", t[:3, 3]))),
                        rotation=types.SimpleNamespace(**dict(zip("xyzw", q))),
                    ),
                )

        self.install_module("rclpy")
        self.install_module("rclpy.node", Node=Node)
        self.install_module("rclpy.time", Time=Time)
        self.install_module(
            "rclpy.qos",
            qos_profile_sensor_data=None,
            QoSProfile=lambda **k: None,
            DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
        )
        self.install_module(
            "tf2_ros",
            Buffer=Buffer,
            TransformListener=lambda *a: None,
            TransformException=KeyError,
        )
        self.install_module("sensor_msgs")
        self.install_module("sensor_msgs.msg", PointCloud2=object)
        pc = types.SimpleNamespace(read_points=lambda m, **k: m.points)
        self.install_module("sensor_msgs_py", point_cloud2=pc)
        for name in (
            "dart_lidar.node",
            "dart_target_estimation.node",
            "dart_aiming.node",
        ):
            self.saved[name] = sys.modules.pop(name, None)
        self.lidar = importlib.import_module("dart_lidar.node").Lidar()
        self.estimator = importlib.import_module(
            "dart_target_estimation.node"
        ).Estimator()
        self.aiming = importlib.import_module("dart_aiming.node").Aiming()

    def close(self):
        for name, old in self.saved.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old

    def control(self, mode=3):
        m = self.msg.ControllerState()
        m.header.stamp = self.Time(self.now).to_msg()
        m.target_id = mode
        m.offset_rad = 0.015
        self.aiming.on_controller(m)

    def step(self, points, mode=3):
        self.now += 0.05
        self.control(mode)
        self.lidar.tick()
        cloud = types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=self.Time(self.now).to_msg(), frame_id="livox_frame"
            ),
            points=[tuple(p) for p in points],
        )
        self.lidar.cloud(cloud)
        self.estimator.tick()
        self.aiming.tick()
        return self.outputs["aim_command"]


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.h = Harness()
        h = self.h
        model = voxel(read_pcd(h.overrides["base_model"]))
        base = h.frames["base_link"]
        self.far = model @ base[:3, :3].T + base[:3, 3]
        rng = np.random.default_rng(1)
        self.door = rng.uniform([0.5, -0.3, -0.3], [0.6, 0.3, 0.3], (100, 3))

    def tearDown(self):
        self.h.close()

    def test_open_initialize_close_reopen_and_stale_command(self):
        h = self.h
        for _ in range(50):
            out = h.step(self.far)
        self.assertEqual(h.lidar.epoch, 1)
        self.assertEqual(h.lidar.source, 1)
        self.assertEqual(out.state, out.TRACKING)
        self.assertAlmostEqual(out.yaw_rad, 0.015, places=6)
        for _ in range(10):
            out = h.step(self.door)
        self.assertEqual(out.state, out.INVALID)
        for _ in range(9):
            out = h.step(self.far)
        self.assertEqual(h.lidar.epoch, 2)
        self.assertEqual(h.lidar.source, 0)
        self.assertEqual(out.state, out.DEGRADED)
        self.assertEqual(h.lidar.revision, 0)
        h.now += 1
        h.aiming.tick()
        self.assertEqual(h.outputs["aim_command"].state, out.INVALID)

    def test_timeout_prior_and_acquisition(self):
        h = self.h
        # Valid far returns but fewer than required base-fitting support points.
        sparse = self.far[::100]
        for _ in range(80):
            out = h.step(sparse, mode=1)
        self.assertTrue(h.lidar.finalized)
        self.assertEqual(h.lidar.source, 0)
        self.assertEqual(out.state, out.ACQUIRE)

    def test_camera_geometry_fallback_and_old_epoch_rejection(self):
        h = self.h
        for _ in range(35):
            h.step(self.far, mode=1)
        rail = h.frames["rail_origin_link"]
        local = np.array([0.41, 0.1222, 0.02363])
        target = rail[:3, :3] @ local + rail[:3, 3]
        camera = h.frames["camera_optical_frame"]
        bearing = camera[:3, :3].T @ (target - camera[:3, 3])
        bearing /= np.linalg.norm(bearing)
        cm = h.msg.CameraObservation()
        cm.header.stamp = h.Time(h.now).to_msg()
        cm.header.frame_id = "camera_optical_frame"
        cm.status_code = cm.STATUS_OK
        cm.bearing.x, cm.bearing.y, cm.bearing.z = map(float, bearing)
        h.estimator.on_camera(cm)
        h.estimator.tick()
        h.aiming.tick()
        estimate = h.outputs["target_estimation"]
        self.assertTrue(estimate.light_valid)
        self.assertEqual(estimate.light_source, estimate.CAMERA_GEOMETRY)
        self.assertEqual(h.outputs["aim_command"].state, h.msg.AimCommand.TRACKING)
        old = h.msg.LidarObservation()
        old.valid = True
        old.header.stamp = h.Time(h.now).to_msg()
        old.header.frame_id = "dart_base_link"
        old.epoch = 0
        old.reference_revision = h.lidar.revision
        old.rail_position_m = 0.1
        old.window_start = h.Time(h.now).to_msg()
        h.estimator.on_lidar(old)
        h.estimator.tick()
        self.assertEqual(
            h.outputs["target_estimation"].light_source, estimate.CAMERA_GEOMETRY
        )
        h.now += 0.3
        h.estimator.tick()
        h.control(1)
        h.aiming.tick()
        self.assertFalse(h.outputs["target_estimation"].light_valid)

    def test_lidar_module_becomes_actual_light_and_changes_distance(self):
        h = self.h
        module = read_stl(h.overrides["module_model"], count=1000) + [0.45, 0, 0]
        rail = h.frames["rail_origin_link"]
        world = module @ rail[:3, :3].T + rail[:3, 3]
        cloud = np.concatenate([self.far, world])
        for _ in range(45):
            out = h.step(cloud, mode=1)
        estimate = h.outputs["target_estimation"]
        self.assertTrue(estimate.light_valid)
        self.assertEqual(estimate.light_source, estimate.LIDAR)
        self.assertIn(out.state, (out.TRACKING, out.DEGRADED))
        self.assertGreater(abs(out.yaw_rad - 0.015), 0.004)


if __name__ == "__main__":
    unittest.main()
