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

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
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

        def stop(process):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)

        for package, executable, config, overrides in [
            ("dart_stereo", "stereo_node", "stereo_triangulator.yaml", []),
            ("dart_aiming", "aiming_node", "aiming.yaml", []),
            ("dart_serial", "serial_node", "serial.yaml", ["-p", f"device:={os.ttyname(slave)}"]),
        ]:
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
            ("left_camera_optical_frame", (0.0, 0.1, 0.0), (-0.5, 0.5, -0.5, 0.5)),
            ("right_camera_optical_frame", (0.0, -0.1, 0.0), (-0.5, 0.5, -0.5, 0.5)),
            ("ballistic_reference_frame", (0.2, 0.0, 0.6), (0.0, 0.0, 0.0, 1.0)),
        ]:
            transform = TransformStamped()
            transform.header.frame_id = "stereo_camera_center_link"
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
        broadcaster.sendTransform(transforms)
        publishers = [
            node.create_publisher(
                GreenLightDetection, f"/{side}_camera/detection", qos_profile_sensor_data
            )
            for side in ("left", "right")
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

        def publish(z, status=GreenLightDetection.DETECTED, mode=0):
            # Exercise the inbound controller protocol too.
            controller = struct.pack("<BBBff", 0x5A, mode, 0, 0.0, 0.0)
            os.write(master, controller + struct.pack("<H", crc16(controller)))
            stamp = node.get_clock().now().to_msg()
            for publisher, side, origin_y in zip(
                publishers, ("left", "right"), (0.1, -0.1), strict=True
            ):
                detection = GreenLightDetection()
                detection.header.stamp = stamp
                detection.header.frame_id = f"{side}_camera_optical_frame"
                detection.status = status
                optical = (origin_y, -z, 10.0)
                norm = math.sqrt(sum(value * value for value in optical))
                detection.unit_ray.x, detection.unit_ray.y, detection.unit_ray.z = [
                    v / norm for v in optical
                ]
                publisher.publish(detection)

        for mode, height in [(0, -1.0), (1, 0.0), (1, 2.0)]:
            with self.subTest(mode=mode, height=height):
                commands.clear()
                targets.clear()
                run_until(
                    lambda frames, height=height: (
                        any(
                            f[1] == 1 and abs(f[3] - 9.8) < 1e-4 and abs(f[4] - height) < 1e-4
                            for f in frames
                        )
                        and any(
                            c.state == 1 and abs(c.vertical_difference_m - height) < 1e-4
                            for c in commands
                        )
                        and any(
                            t.status == 1 and abs(t.position.z - (height + 0.6)) < 1e-4
                            for t in targets
                        )
                    ),
                    lambda height=height, mode=mode: publish(height + 0.6, mode=mode),
                )
                valid = [t for t in targets if t.status == 1]
                self.assertTrue(valid)
                self.assertAlmostEqual(valid[-1].position.x, 10.0, places=5)
                self.assertTrue(
                    all(c.header.frame_id == "ballistic_reference_frame" for c in commands)
                )
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
