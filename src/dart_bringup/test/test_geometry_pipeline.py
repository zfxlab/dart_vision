"""Synthetic stereo -> aiming -> PTY serial regression; no physical devices.

Run after building and sourcing the workspace:
python3 -m unittest discover -s src/dart_bringup/test -p test_geometry_pipeline.py -v
"""

import math
import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
import tty
import unittest
from pathlib import Path

import numpy as np
import rclpy
import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from target_geometry import Geometry
from tf2_ros import StaticTransformBroadcaster

from dart_interfaces.msg import AimCommand, GreenLightDetection, StereoTarget


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


class GeometryPipelineTest(unittest.TestCase):
    def test_signed_geometry_closed_and_timeout_reach_wire(self):
        self.check_pipeline("launcher_frame", (0.0, 0.0, 0.0))

    def test_ballistic_reference_transform_reaches_wire(self):
        self.check_pipeline("ballistic_reference_frame", (0.2, 0.0, 0.6))

    def test_installed_description_geometry_reaches_wire(self):
        self.check_pipeline("launcher_frame", real_description=True)

    def test_installed_description_ballistic_transform_reaches_wire(self):
        self.check_pipeline("ballistic_reference_frame", real_description=True)

    def check_pipeline(self, reference_frame, reference_origin=None, *, real_description=False):
        domain = 100 + os.getpid() % 100
        context = Context()
        rclpy.init(context=context, domain_id=domain)
        self.addCleanup(context.shutdown)
        node = rclpy.create_node("geometry_pipeline_test", context=context)
        self.addCleanup(node.destroy_node)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        self.addCleanup(executor.shutdown)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        master, slave = pty.openpty()
        tty.setraw(slave)
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        os.set_blocking(master, False)
        environment = {**os.environ, "ROS_DOMAIN_ID": str(domain), "ROS_LOG_DIR": temporary.name}
        configs = Path(__file__).resolve().parents[1] / "config"
        processes = []
        sides = ("top", "bottom")
        camera_poses = {}
        if real_description:
            share = Path(get_package_share_directory("dart_description"))
            geometry = Geometry(path=share / "urdf/dart_system.urdf.xacro")
            camera_poses = {
                side: geometry.pose(f"{side}_camera_optical_frame", reference="launcher_frame")
                for side in sides
            }
            reference_origin = geometry.pose(reference_frame, reference="launcher_frame")[:3, 3]
            robot_description = xacro.process_file(str(share / "urdf/dart_robot.urdf.xacro"))
            description_config = Path(temporary.name) / "description.yaml"
            description_config.write_text(
                yaml.safe_dump(
                    {"/**": {"ros__parameters": {"robot_description": robot_description.toxml()}}}
                )
            )
        else:
            for side, z in zip(sides, (0.1, -0.1), strict=True):
                pose = np.eye(4)
                pose[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
                pose[2, 3] = z
                camera_poses[side] = pose

        def stop(process):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)

        node_specs = [
            ("dart_stereo", "stereo_node", "stereo_triangulator.yaml", []),
            (
                "dart_aiming",
                "aiming_node",
                "aiming.yaml",
                ["-p", f"reference_frame:={reference_frame}"]
                if reference_frame != "launcher_frame"
                else [],
            ),
            ("dart_serial", "serial_node", "serial.yaml", ["-p", f"device:={os.ttyname(slave)}"]),
        ]
        if real_description:
            node_specs.append(
                ("robot_state_publisher", "robot_state_publisher", description_config, [])
            )
        for package, executable, config, overrides in node_specs:
            binary = Path(get_package_prefix(package)) / "lib" / package / executable
            log = open(Path(temporary.name) / f"{package}.log", "w+")
            self.addCleanup(log.close)
            process = subprocess.Popen(
                [str(binary), "--ros-args", "--params-file", str(configs / config), *overrides],
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            self.addCleanup(stop, process)
            processes.append((process, log))

        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        # Optical axes -> reference axes: optical z forward, x right, y down.
        for child, xyz, rotation in [
            ("top_camera_optical_frame", (0.0, 0.0, 0.1), (-0.5, 0.5, -0.5, 0.5)),
            ("bottom_camera_optical_frame", (0.0, 0.0, -0.1), (-0.5, 0.5, -0.5, 0.5)),
            ("ballistic_reference_frame", (0.2, 0.0, 0.6), (0.0, 0.0, 0.0, 1.0)),
        ]:
            transform = TransformStamped()
            transform.header.frame_id = "launcher_frame"
            transform.child_frame_id = child
            transform.header.stamp = node.get_clock().now().to_msg()
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ) = xyz
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ) = rotation
            transforms.append(transform)
        # The real-layout cases must obtain TF from robot_state_publisher, not a test broadcaster.
        if not real_description:
            broadcaster.sendTransform(transforms)
        publishers = [
            node.create_publisher(
                GreenLightDetection, f"/{side}_camera/detection", qos_profile_sensor_data
            )
            for side in sides
        ]
        commands, targets = [], []
        node.create_subscription(AimCommand, "/aim_command", commands.append, 10)
        node.create_subscription(
            StereoTarget, "/stereo_target", targets.append, qos_profile_sensor_data
        )
        incoming = bytearray()

        def run_until(predicate, publish, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                for process, log in processes:
                    if process.poll() is not None:
                        log.seek(0)
                        self.fail(log.read())
                publish()
                executor.spin_once(timeout_sec=0.01)
                try:
                    incoming.extend(os.read(master, 65536))
                except BlockingIOError:
                    pass
                frames = []
                while len(incoming) >= 16:
                    self.assertEqual(incoming[0], 0xA5)
                    frame = bytes(incoming[:16])
                    del incoming[:16]
                    self.assertEqual(struct.unpack_from("<H", frame, 14)[0], crc16(frame[:14]))
                    frames.append(struct.unpack("<BBfffH", frame))
                if predicate(frames):
                    return
            self.fail("Timed out waiting for stereo/aiming/wire result")

        def publish(z, status=GreenLightDetection.DETECTED, mode=0, y=0.0):
            # Exercise the inbound controller protocol too.
            controller = struct.pack("<BBBff", 0x5A, mode, 0, 0.0, 0.0)
            os.write(master, controller + struct.pack("<H", crc16(controller)))
            stamp = node.get_clock().now().to_msg()
            for publisher, side in zip(publishers, sides, strict=True):
                detection = GreenLightDetection()
                detection.header.stamp = stamp
                detection.header.frame_id = f"{side}_camera_optical_frame"
                detection.status = status
                pose = camera_poses[side]
                optical = pose[:3, :3].T @ (np.array([10.0, y, z]) - pose[:3, 3])
                norm = math.sqrt(sum(value * value for value in optical))
                detection.unit_ray.x, detection.unit_ray.y, detection.unit_ray.z = [
                    v / norm for v in optical
                ]
                publisher.publish(detection)

        for mode, height, y in [(0, -1.0, -0.3), (1, 0.0, 0.0), (1, 2.0, 0.4)]:
            expected_distance = math.hypot(10.0 - reference_origin[0], y - reference_origin[1])
            expected_yaw = math.atan2(y - reference_origin[1], 10.0 - reference_origin[0])
            with self.subTest(mode=mode, height=height, y=y):
                commands.clear()
                targets.clear()
                run_until(
                    lambda frames, height=height, expected_yaw=expected_yaw, expected_distance=expected_distance: (
                        any(
                            f[1] == 1
                            and abs(f[2] - expected_yaw) < 1e-5
                            and abs(f[3] - expected_distance) < 1e-4
                            and abs(f[4] - height) < 1e-4
                            for f in frames
                        )
                        and any(
                            c.state == 1 and abs(c.vertical_difference_m - height) < 1e-4
                            for c in commands
                        )
                        and any(
                            t.status == 1
                            and abs(t.position.z - (height + reference_origin[2])) < 1e-4
                            for t in targets
                        )
                    ),
                    lambda height=height, mode=mode, y=y: publish(
                        height + reference_origin[2], mode=mode, y=y
                    ),
                )
                valid = [t for t in targets if t.status == 1]
                self.assertTrue(valid)
                self.assertAlmostEqual(valid[-1].position.x, 10.0, places=5)
                self.assertAlmostEqual(valid[-1].position.y, y, places=5)
                self.assertTrue(all(c.header.frame_id == reference_frame for c in commands))
        run_until(
            lambda frames: any(f[1] == 0 and f[2:5] == (0.0, 0.0, 0.0) for f in frames),
            lambda: publish(0.0, GreenLightDetection.CLOSED),
        )
        run_until(
            lambda frames: any(f[1] == 2 and f[2:5] == (0.0, 0.0, 0.0) for f in frames),
            lambda: None,
        )


if __name__ == "__main__":
    unittest.main()
