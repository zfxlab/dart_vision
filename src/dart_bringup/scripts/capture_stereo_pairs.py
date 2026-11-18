#!/usr/bin/env python3
"""Preview two ROS image streams and save numbered calibration pairs."""

import copy
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from stereo_capture_core import CAMERAS, Frame, PairBuffer, SessionWriter

DRIVER_PARAMETERS = (
    "serial_number",
    "transport",
    "exposure_time",
    "auto_exposure",
    "gain",
    "auto_gain",
    "frame_rate",
    "trigger_mode",
    "output_encoding",
    "sdk_buffer_count",
    "grab_timeout_ms",
)


class StereoCapture(Node):
    def __init__(self):
        super().__init__("stereo_capture")
        fixed = ParameterDescriptor(read_only=True)

        def setting(name, default):
            return self.declare_parameter(name, default, fixed).value

        self.preview_enabled = setting("preview_enabled", True)
        self.preview_scale = setting("preview_scale", 0.5)
        max_delta = setting("max_pair_delta_s", 0.01)
        max_age = setting("max_image_age_s", 0.2)
        self.capture_timeout = setting("capture_timeout_s", 2.0)
        queue_size = setting("queue_size", 10)
        self.output_root = setting("output_root", "captures")
        self.require_driver_parameters = setting("require_driver_parameters", False)
        self.declare_parameter("split", "train")
        self.specs = {}
        for camera, namespace in zip(CAMERAS, ("top_camera", "bottom_camera"), strict=True):
            self.specs[camera] = {
                "image_topic": setting(f"{camera}.image_topic", f"/{namespace}/image_raw"),
                "camera_info_topic": setting(
                    f"{camera}.camera_info_topic", f"/{namespace}/camera_info"
                ),
                "frame_id": setting(f"{camera}.frame_id", f"{namespace}_optical_frame"),
                "configured_serial_number": setting(f"{camera}.serial_number", ""),
                "driver_node": setting(f"{camera}.driver_node", f"/{namespace}/driver"),
            }
        if not all(math.isfinite(x) and x > 0 for x in (max_age, self.capture_timeout)):
            raise ValueError("Image age and capture timeout must be finite and positive")
        if not math.isfinite(max_delta) or max_delta < 0:
            raise ValueError("Pair time difference must be finite and nonnegative")
        if not math.isfinite(self.preview_scale) or not 0 < self.preview_scale <= 1:
            raise ValueError("preview_scale must be in (0, 1]")
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        if self.get_parameter("split").value not in ("train", "validation"):
            raise ValueError("split must be train or validation")
        if any(not spec["image_topic"] or not spec["frame_id"] for spec in self.specs.values()):
            raise ValueError("Image topics and expected frame IDs must be set")
        for key in ("image_topic", "frame_id"):
            if self.specs[CAMERAS[0]][key] == self.specs[CAMERAS[1]][key]:
                raise ValueError(f"The cameras must have different {key} values")
        serials = [spec["configured_serial_number"] for spec in self.specs.values()]
        if serials[0] and serials[0] == serials[1]:
            raise ValueError("The cameras must have different serial numbers")
        if self.preview_enabled and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            raise RuntimeError("No display found; set preview_enabled:=false and use ~/capture")
        self.bridge = CvBridge()
        self.buffer = PairBuffer(queue_size, round(max_delta * 1e9), round(max_age * 1e9))
        self.latest = {}
        self.infos = {}
        self.indices = dict.fromkeys(CAMERAS, 0)
        self.driver_snapshots = {}
        self.driver_queries = {}
        self.driver_clients = {
            camera: AsyncParameterClient(self, spec["driver_node"])
            for camera, spec in self.specs.items()
            if spec["driver_node"]
        }
        self.pending = None
        self.write_future = None
        self.status = "Keep cameras and board still, then press S"
        self.next_preview_ns = 0
        self.saved = 0
        self.previous_ros_ns = self.get_clock().now().nanoseconds
        self.writer = SessionWriter(
            self.output_root,
            {
                "cameras": self.specs,
                "max_pair_delta_s": max_delta,
                "max_image_age_s": max_age,
                "capture_timeout_s": self.capture_timeout,
                "queue_size": queue_size,
                "require_driver_parameters": self.require_driver_parameters,
            },
        )
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="stereo_png_writer")
        self.subscriptions_ = []
        for camera, spec in self.specs.items():
            self.subscriptions_.append(
                self.create_subscription(
                    Image,
                    spec["image_topic"],
                    lambda message, camera=camera: self.image_callback(camera, message),
                    qos_profile_sensor_data,
                )
            )
            if spec["camera_info_topic"]:
                self.subscriptions_.append(
                    self.create_subscription(
                        CameraInfo,
                        spec["camera_info_topic"],
                        lambda message, camera=camera: self.infos.__setitem__(camera, message),
                        qos_profile_sensor_data,
                    )
                )
        self.service_group = ReentrantCallbackGroup()
        self.capture_service = self.create_service(
            Trigger,
            "~/capture",
            self.capture_callback,
            callback_group=self.service_group,
        )
        self.timer = self.create_timer(0.02, self.tick)
        self.parameter_timer = self.create_timer(0.25, self.sample_parameters)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.window_name = "Stereo calibration capture"
        if self.preview_enabled:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Capture session: {self.writer.path}")
        self.get_logger().info(
            "S: save pair; T: training; V: validation; Q: quit; service: ~/capture"
        )

    @staticmethod
    def validate_parameters(parameters):
        for parameter in parameters:
            if parameter.name == "split" and parameter.value not in ("train", "validation"):
                return SetParametersResult(
                    successful=False, reason="split must be train or validation"
                )
        return SetParametersResult(successful=True)

    def image_callback(self, camera, message):
        spec = self.specs[camera]
        if message.header.frame_id != spec["frame_id"]:
            self.status = f"{camera}: unexpected frame {message.header.frame_id!r}"
            return
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp_ns <= 0 or message.encoding not in ("mono8", "bgr8", "rgb8"):
            self.status = f"{camera}: invalid timestamp or unsupported encoding"
            return
        try:
            pixels = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            if message.encoding == "rgb8":
                pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            pixels = pixels.copy()
        except (ValueError, TypeError, CvBridgeError, cv2.error) as error:
            self.status = f"{camera}: {error}"
            return
        if (
            pixels.dtype != np.uint8
            or pixels.size == 0
            or pixels.shape[:2] != (message.height, message.width)
        ):
            self.status = f"{camera}: invalid image dimensions or pixel type"
            return
        self.indices[camera] += 1
        frame = Frame(
            pixels,
            stamp_ns,
            time.monotonic_ns(),
            self.get_clock().now().nanoseconds,
            self.indices[camera],
            message.header.frame_id,
            message.encoding,
            "mono8" if message.encoding == "mono8" else "bgr8",
        )
        self.latest[camera] = frame
        self.buffer.add(camera, frame)

    def sample_parameters(self):
        for camera, client in self.driver_clients.items():
            query = self.driver_queries.get(camera)
            if query is not None:
                future, started_ns = query
                if not future.done():
                    if time.monotonic_ns() - started_ns < 2 * 10**9:
                        continue
                    future.cancel()
                    del self.driver_queries[camera]
                    continue
                try:
                    response = future.result()
                    self.driver_snapshots[camera] = {
                        "sampled_ros_ns": self.get_clock().now().nanoseconds,
                        "sampled_mono_ns": time.monotonic_ns(),
                        "values": dict(
                            zip(
                                DRIVER_PARAMETERS,
                                [parameter_value_to_python(value) for value in response.values],
                                strict=True,
                            )
                        ),
                    }
                except Exception as error:
                    self.get_logger().debug(f"Cannot sample {camera} driver parameters: {error}")
                del self.driver_queries[camera]
            if client.services_are_ready():
                self.driver_queries[camera] = (
                    client.get_parameters(list(DRIVER_PARAMETERS)),
                    time.monotonic_ns(),
                )

    def begin_capture(self):
        if self.pending is not None:
            return None, "A capture is already in progress"
        for camera, snapshot in self.driver_snapshots.items():
            values = snapshot["values"]
            expected = self.specs[camera]["configured_serial_number"]
            if expected and values.get("serial_number") != expected:
                return None, f"{camera}: driver serial number differs from configuration"
            if values.get("trigger_mode") not in (None, "off"):
                return None, f"{camera}: this capture tool requires free-run trigger_mode=off"
        future = Future()
        self.pending = {
            "future": future,
            "ros_ns": self.get_clock().now().nanoseconds,
            "mono_ns": time.monotonic_ns(),
            "split": self.get_parameter("split").value,
        }
        self.status = f"Waiting for a new {self.pending['split']} pair"
        return future, ""

    async def capture_callback(self, _request, response):
        future, error = self.begin_capture()
        if future is None:
            response.success, response.message = False, error
        else:
            response.success, response.message = await future
        return response

    def finish(self, success, message):
        pending, self.pending = self.pending, None
        self.write_future = None
        self.status = message
        if pending is not None and not pending["future"].done():
            pending["future"].set_result((success, message))
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)

    def tick(self):
        now_ros_ns = self.get_clock().now().nanoseconds
        now_mono_ns = time.monotonic_ns()
        if now_ros_ns < self.previous_ros_ns:
            self.buffer.clear()
            self.latest.clear()
            if self.pending and self.write_future is None:
                self.finish(False, "Clock moved backwards; capture cancelled")
        self.previous_ros_ns = now_ros_ns
        if self.pending is None:
            return
        if self.write_future is not None:
            if self.write_future.done():
                try:
                    pair_id = self.write_future.result()
                except Exception as error:
                    self.finish(False, f"Could not save pair: {error}")
                else:
                    self.buffer.consume(self.pending["pair"])
                    self.saved += 1
                    self.finish(
                        True, f"Saved {pair_id} ({self.pending['split']}) in {self.writer.path}"
                    )
            return
        if now_mono_ns - self.pending["mono_ns"] > self.capture_timeout * 1e9:
            self.finish(
                False,
                "Capture timed out: need fresh matching frames and required driver parameters",
            )
            return
        if self.require_driver_parameters and any(
            camera not in self.driver_snapshots
            or now_mono_ns - self.driver_snapshots[camera]["sampled_mono_ns"] > 3 * 10**9
            for camera in CAMERAS
        ):
            self.status = "Waiting for camera driver parameters"
            return
        # Parameter services may become available after the request was accepted.
        for camera, snapshot in self.driver_snapshots.items():
            expected = self.specs[camera]["configured_serial_number"]
            if expected and snapshot["values"].get("serial_number") != expected:
                self.finish(False, f"{camera}: driver serial number differs from configuration")
                return
            if snapshot["values"].get("trigger_mode") not in (None, "off"):
                self.finish(False, f"{camera}: free-run trigger_mode=off required")
                return
        pair = self.buffer.select(
            now_ros_ns, now_mono_ns, self.pending["ros_ns"], self.pending["mono_ns"]
        )
        if pair is None:
            return
        metadata = {
            "schema_version": 1,
            "acquisition_mode": "free_run_receive_time_pairing",
            "request_ros_ns": self.pending["ros_ns"],
            "request_mono_ns": self.pending["mono_ns"],
            "cameras": {},
        }
        for camera, frame in pair.items():
            info = self.infos.get(camera)
            compatible = (
                info is not None
                and info.header.frame_id == frame.frame_id
                and (info.width, info.height) == (frame.image.shape[1], frame.image.shape[0])
            )
            metadata["cameras"][camera] = {
                **self.specs[camera],
                "camera_info": message_to_ordereddict(info) if compatible else None,
                "driver_parameters": copy.deepcopy(self.driver_snapshots.get(camera)),
            }
        self.pending["pair"] = pair
        self.status = "Writing two PNG images"
        self.write_future = self.worker.submit(
            self.writer.save, pair, self.pending["split"], metadata
        )

    def show_preview(self):
        if not self.preview_enabled:
            return True
        panes = []
        now_ns = time.monotonic_ns()
        # Keep the UI at 20 Hz so rendering does not delay high-rate image callbacks.
        if now_ns < self.next_preview_ns:
            return True
        self.next_preview_ns = now_ns + 50_000_000
        for camera in CAMERAS:
            frame = self.latest.get(camera)
            if frame is None:
                pane = np.zeros((270, 480, 3), dtype=np.uint8)
                label = f"{camera}: no image"
                color = (0, 0, 255)
            else:
                image = frame.image
                if image.ndim == 2:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                pane = cv2.resize(image, None, fx=self.preview_scale, fy=self.preview_scale)
                age_ns = now_ns - frame.received_mono_ns
                label = f"{camera}: {age_ns / 1e6:.0f} ms old"
                color = (0, 220, 0) if age_ns <= self.buffer.max_age_ns else (0, 0, 255)
            cv2.putText(pane, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            panes.append(pane)
        height = max(pane.shape[0] for pane in panes)
        panes = [
            cv2.copyMakeBorder(pane, 0, height - pane.shape[0], 0, 0, cv2.BORDER_CONSTANT)
            for pane in panes
        ]
        image = np.hstack(panes)
        image = cv2.copyMakeBorder(image, 0, 100, 0, 0, cv2.BORDER_CONSTANT)
        delta = "--"
        if all(camera in self.latest for camera in CAMERAS):
            delta = f"{abs(self.latest[CAMERAS[0]].stamp_ns - self.latest[CAMERAS[1]].stamp_ns) / 1e6:.2f}"
        lines = [
            f"Saved: {self.saved} | split: {self.get_parameter('split').value} | receive delta: {delta} ms",
            self.status[:100],
            "S save | T train | V validation | Q quit",
        ]
        for index, line in enumerate(lines):
            cv2.putText(
                image,
                line,
                (8, height + 24 + index * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (230, 230, 230),
                1,
            )
        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            return False
        if key in (ord("s"), ord("S")):
            _, error = self.begin_capture()
            if error:
                self.status = error
        elif key in (ord("t"), ord("T"), ord("v"), ord("V")):
            self.set_parameters(
                [Parameter("split", value="train" if chr(key).lower() == "t" else "validation")]
            )
        return cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) >= 1

    def close(self):
        self.worker.shutdown(wait=True)
        if self.write_future is not None:
            self.tick()
        elif self.pending is not None:
            self.finish(False, "Capture cancelled on exit")
        if self.preview_enabled:
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = None
    try:
        node = StereoCapture()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.show_preview():
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
