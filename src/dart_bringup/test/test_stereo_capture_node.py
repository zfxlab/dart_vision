"""Headless ROS capture regression with synthetic image publishers."""

import csv
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from cv_bridge import CvBridge
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger


class CaptureNodeTest(unittest.TestCase):
    def test_preview_keys_render_without_modifying_original_pixels(self):
        directory = Path(get_package_prefix("dart_bringup")) / "lib/dart_bringup"
        sys.path.insert(0, str(directory))
        from capture_stereo_pairs import StereoCapture
        from stereo_capture_core import Frame, PairBuffer

        original = np.full((24, 32, 3), 80, np.uint8)
        now = time.monotonic_ns()
        frames = {
            camera: Frame(original, 100, now, 100, 1, camera, "bgr8", "bgr8")
            for camera in ("camera_1", "camera_2")
        }
        fake = SimpleNamespace(
            preview_enabled=True,
            preview_scale=0.5,
            next_preview_ns=0,
            latest=frames,
            buffer=PairBuffer(10, 5, 10**9),
            saved=2,
            get_parameter=lambda _: SimpleNamespace(value="train"),
            status="Ready",
            window_name="test",
            begin_capture=Mock(return_value=(None, "")),
            set_parameters=Mock(),
        )
        with (
            patch("capture_stereo_pairs.cv2.imshow") as show,
            patch("capture_stereo_pairs.cv2.getWindowProperty", return_value=1),
            patch(
                "capture_stereo_pairs.cv2.waitKey",
                side_effect=[ord("s"), ord("v"), ord("t"), ord("q")],
            ),
        ):
            for _ in range(3):
                fake.next_preview_ns = 0
                self.assertTrue(StereoCapture.show_preview(fake))
            fake.next_preview_ns = 0
            self.assertFalse(StereoCapture.show_preview(fake))
        fake.begin_capture.assert_called_once()
        self.assertEqual(
            [call.args[0][0].value for call in fake.set_parameters.call_args_list],
            ["validation", "train"],
        )
        self.assertEqual(show.call_args.args[1].shape, (112, 32, 3))
        np.testing.assert_array_equal(original, np.full((24, 32, 3), 80, np.uint8))

    def test_fresh_pairs_busy_request_validation_split_and_timeout(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        domain = 100 + os.getpid() % 100
        context = Context()
        rclpy.init(context=context, domain_id=domain)
        self.addCleanup(context.shutdown)
        node = rclpy.create_node("capture_test", context=context)
        self.addCleanup(node.destroy_node)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        self.addCleanup(executor.shutdown)
        parameters = {
            "preview_enabled": False,
            "output_root": str(root / "captures"),
            "capture_timeout_s": 1.5,
            "max_pair_delta_s": 0.005,
            "require_driver_parameters": True,
        }
        for camera in ("camera_1", "camera_2"):
            driver = rclpy.create_node("driver", namespace=f"/{camera}", context=context)
            self.addCleanup(driver.destroy_node)
            executor.add_node(driver)
            driver.declare_parameters(
                "",
                [
                    ("serial_number", f"test_{camera}"),
                    ("transport", "usb"),
                    ("exposure_time", 3000.0),
                    ("auto_exposure", False),
                    ("gain", 0.0),
                    ("auto_gain", False),
                    ("frame_rate", 50.0),
                    ("trigger_mode", "off"),
                    ("output_encoding", "bgr8"),
                    ("sdk_buffer_count", 4),
                    ("grab_timeout_ms", 1000),
                ],
            )
            parameters.update(
                {
                    f"{camera}.image_topic": f"/{camera}/image_raw",
                    f"{camera}.camera_info_topic": f"/{camera}/camera_info",
                    f"{camera}.frame_id": f"{camera}_optical",
                    f"{camera}.driver_node": f"/{camera}/driver",
                    f"{camera}.serial_number": f"test_{camera}",
                }
            )
        config = root / "capture.yaml"
        config.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
        script = (
            Path(get_package_prefix("dart_bringup")) / "lib/dart_bringup/capture_stereo_pairs.py"
        )
        log = (root / "node.log").open("w+")
        self.addCleanup(log.close)
        process = subprocess.Popen(
            [sys.executable, str(script), "--ros-args", "--params-file", str(config)],
            env={**os.environ, "ROS_DOMAIN_ID": str(domain), "ROS_LOG_DIR": str(root / "log")},
            stdout=log,
            stderr=subprocess.STDOUT,
        )

        def stop():
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)

        self.addCleanup(stop)
        bridge = CvBridge()
        images = {
            camera: np.full((24, 32, 3), [10 + index, 30, 80], dtype=np.uint8)
            for index, camera in enumerate(("camera_1", "camera_2"))
        }
        publishers = {
            camera: node.create_publisher(Image, f"/{camera}/image_raw", qos_profile_sensor_data)
            for camera in images
        }
        info_publishers = {
            camera: node.create_publisher(
                CameraInfo, f"/{camera}/camera_info", qos_profile_sensor_data
            )
            for camera in images
        }

        def publish(cameras=("camera_1", "camera_2")):
            stamp = node.get_clock().now().to_msg()
            for camera in cameras:
                encoding = "rgb8" if camera == "camera_1" else "bgr8"
                image = bridge.cv2_to_imgmsg(images[camera], encoding=encoding)
                image.header.stamp = stamp
                image.header.frame_id = f"{camera}_optical"
                info = CameraInfo(width=32, height=24)
                info.header = image.header
                info_publishers[camera].publish(info)
                publishers[camera].publish(image)

        def until(predicate, send=None, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    log.seek(0)
                    self.fail(log.read())
                if send:
                    send()
                executor.spin_once(timeout_sec=0.01)
                if predicate():
                    return
            log.seek(0)
            self.fail("Capture test timed out\n" + log.read())

        client = node.create_client(Trigger, "/stereo_capture/capture")
        until(
            lambda: (
                client.service_is_ready()
                and all(publisher.get_subscription_count() for publisher in publishers.values())
            )
        )
        for _ in range(5):
            publish()
            executor.spin_once(timeout_sec=0.01)
        future = client.call_async(Trigger.Request())
        # Buffered images must not satisfy a new save request.
        for _ in range(8):
            executor.spin_once(timeout_sec=0.01)
        self.assertFalse(future.done())
        until(future.done, publish)
        self.assertTrue(future.result().success, future.result().message)
        (session,) = (root / "captures").iterdir()
        with (session / "pairs.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        for camera in images:
            actual = cv2.imread(str(session / rows[0][camera]), cv2.IMREAD_UNCHANGED)
            expected = images[camera][:, :, ::-1] if camera == "camera_1" else images[camera]
            np.testing.assert_array_equal(actual, expected)
        metadata = json.loads((session / "metadata/000001.json").read_text())
        self.assertEqual(metadata["cameras"]["camera_1"]["camera_info"]["k"], [0.0] * 9)
        snapshot = metadata["cameras"]["camera_1"]["driver_parameters"]["values"]
        self.assertEqual(snapshot["exposure_time"], 3000.0)
        self.assertEqual(snapshot["serial_number"], "test_camera_1")
        self.assertGreaterEqual(
            metadata["images"]["camera_1"]["stamp_ns"], metadata["request_ros_ns"]
        )
        parameter_client = AsyncParameterClient(node, "/stereo_capture")
        until(parameter_client.services_are_ready)
        change = parameter_client.set_parameters([Parameter("split", value="validation")])
        until(change.done)
        self.assertTrue(change.result().results[0].successful)
        requests = [client.call_async(Trigger.Request()) for _ in range(2)]
        until(lambda: any(f.done() for f in requests))
        failed = next(f for f in requests if f.done())
        self.assertFalse(failed.result().success)
        self.assertIn("already in progress", failed.result().message)
        until(lambda: all(f.done() for f in requests), publish)
        self.assertEqual(sum(f.result().success for f in requests), 1)
        missing = client.call_async(Trigger.Request())
        until(missing.done, lambda: publish(("camera_1",)))
        self.assertFalse(missing.result().success)
        self.assertIn("timed out", missing.result().message)
        with (session / "pairs.csv").open(newline="") as stream:
            self.assertEqual(
                [row["split"] for row in csv.DictReader(stream)], ["train", "validation"]
            )


if __name__ == "__main__":
    unittest.main()
