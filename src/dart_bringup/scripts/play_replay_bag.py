#!/usr/bin/env python3
"""Play all recorded topics or selected inputs, retaining range initialization data."""

import argparse
import hashlib
import math
import signal
import subprocess
import sys
import tempfile
import time
from collections import deque
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message, serialize_message
from record_bag import resolve_topics
from rosidl_runtime_py import get_interface_path
from tf2_msgs.msg import TFMessage


def run_player(command, args, temporary):
    """Start a recorder before releasing a paused player, then finalize its session."""
    recorder = None
    child = None
    node = None
    stop_requested = False
    outcome = {"status": "failed", "reason": "Replay did not finish"}
    outcome_path = Path(temporary) / "replay_outcome.yaml"

    def stop(signum, _frame):
        nonlocal stop_requested
        stop_requested = True
        for process in (child,):
            if process is not None and process.poll() is None:
                process.send_signal(signum)

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        if args.start_recording:
            import rclpy
            from rosbag2_interfaces.srv import Resume
            from rosgraph_msgs.msg import Clock

            with args.recording_config.open(encoding="utf-8") as stream:
                config = yaml.safe_load(stream)["recording"]
            if args.recording_profile not in ("inputs", "full"):
                raise ValueError("recording profile must be inputs or full")
            replay_context = {
                "mode": "recompute",
                "source_bag": str(args.bag.resolve()),
                "source_metadata_sha256": hashlib.sha256(
                    (args.bag / "metadata.yaml").read_bytes()
                ).hexdigest(),
                "start_seconds": args.start,
                "end_seconds": args.end,
                "rate": float(args.rate),
                "loop": False,
                "input_topics": args.topics,
                "qos_overrides": yaml.safe_load(Path(args.qos).read_text(encoding="utf-8")),
                "runtime": {
                    "use_sim_time": True,
                    "start_driver": False,
                    "start_serial": False,
                    "start_description": False,
                },
            }
            source_manifest = args.bag.parent / "manifest.yaml"
            if source_manifest.is_file():
                with source_manifest.open(encoding="utf-8") as stream:
                    replay_context["source_manifest"] = yaml.safe_load(stream)
            context_path = Path(temporary) / "replay_context.yaml"
            snapshot_ready = Path(temporary) / "snapshot_ready"
            with context_path.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(replay_context, stream, allow_unicode=True)
            recorder = subprocess.Popen(
                [
                    sys.executable,
                    str(Path(__file__).resolve().with_name("record_bag.py")),
                    "--config",
                    str(args.recording_config),
                    "--profile",
                    args.recording_profile,
                    "--use-sim-time",
                    "--replay-context",
                    str(context_path),
                    "--replay-outcome",
                    str(outcome_path),
                    "--snapshot-ready-file",
                    str(snapshot_ready),
                    "--snapshot-required-nodes",
                    *args.required_nodes,
                ],
                start_new_session=True,
            )
            command.append("--start-paused")
            rclpy.init()
            node = rclpy.create_node("replay_recording_coordinator")
            resume = node.create_client(Resume, "/rosbag2_player/resume")
            # Per-message clock has no ticks while paused. Seed the exact start
            # time so the sim-time recorder can discover subscriptions first.
            clock_publisher = node.create_publisher(Clock, "/clock", 10)
            initial_clock = Clock()
            initial_ns = rosbag2_py.MetadataIo().read_metadata(command[3]).starting_time.nanoseconds
            initial_clock.clock.sec, initial_clock.clock.nanosec = divmod(initial_ns, 10**9)
        child = subprocess.Popen(command, start_new_session=True)
        if stop_requested:
            stop(signal.SIGINT, None)
        if args.start_recording:
            deadline = time.monotonic() + 30.0
            required = set(resolve_topics(config["profiles"], args.recording_profile))
            # Topics without a producer (e.g. /joint_states with description disabled)
            # are optional; algorithm outputs and replay inputs must be discovered.
            required &= set(args.topics) | {
                "/left_camera/detection",
                "/right_camera/detection",
                "/stereo_target",
                "/aim_command",
            }
            while not stop_requested:
                clock_publisher.publish(initial_clock)
                if child.poll() is not None or recorder.poll() is not None:
                    raise RuntimeError("Player or recorder exited before recording was ready")
                ready = resume.service_is_ready() and all(
                    any(
                        info.node_name == "rosbag2_recorder"
                        for info in node.get_subscriptions_info_by_topic(topic)
                    )
                    for topic in required
                )
                ready = ready and set(args.required_nodes).issubset(
                    f"{namespace.rstrip('/')}/{name}"
                    for name, namespace in node.get_node_names_and_namespaces()
                )
                if ready and snapshot_ready.is_file():
                    future = resume.call_async(Resume.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                    if not future.done() or future.exception() is not None:
                        raise RuntimeError("Failed to resume player after recorder discovery")
                    print("Pipeline and recorder ready; playback resumed", flush=True)
                    node.destroy_publisher(clock_publisher)
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        "Timed out waiting for pipeline, recorder subscriptions or parameter snapshot"
                    )
                rclpy.spin_once(node, timeout_sec=0.1)
        while child.poll() is None:
            if recorder is not None and recorder.poll() is not None and not stop_requested:
                raise RuntimeError("Recorder exited during playback")
            time.sleep(0.1)
        result = child.returncode
        if recorder is not None and not stop_requested and result == 0:
            # 让最后一帧的算法回调和 DDS 传输完成，再关闭录制文件。
            time.sleep(1.0)
        outcome = {
            "status": "interrupted"
            if stop_requested
            else ("complete" if result == 0 else "failed"),
            "player_exit_code": result,
        }
        return 0 if stop_requested else result
    except Exception as error:
        outcome = {"status": "failed", "reason": str(error)}
        raise
    finally:
        try:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5)
        finally:
            try:
                if recorder is not None:
                    if args.failure_file and args.failure_file.is_file():
                        outcome = {
                            "status": "failed",
                            "reason": args.failure_file.read_text(encoding="utf-8"),
                        }
                    with outcome_path.open("w", encoding="utf-8") as stream:
                        yaml.safe_dump(outcome, stream)
                    if recorder.poll() is None:
                        recorder.send_signal(signal.SIGINT)
                    recorder.wait(timeout=40)
                    if recorder.returncode != 0:
                        raise RuntimeError(f"Recorder failed with exit code {recorder.returncode}")
            finally:
                if node is not None:
                    node.destroy_node()
                    rclpy.shutdown()
                for sig, handler in previous.items():
                    signal.signal(sig, handler)


def report_interface_mismatches(source, topics):
    """Warn about unavailable or changed top-level .msg fields without converting data."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source)), rosbag2_py.ConverterOptions("", ""))
    try:
        selected = {
            t.type for t in reader.get_all_topics_and_types() if topics is None or t.name in topics
        }

        def fields(text):
            return [
                " ".join(line.split("#", 1)[0].split())
                for line in text.splitlines()
                if line.split("#", 1)[0].strip()
            ]

        for definition in reader.get_all_message_definitions():
            if definition.topic_type not in selected or definition.encoding != "ros2msg":
                continue
            try:
                current = Path(get_interface_path(definition.topic_type)).read_text(
                    encoding="utf-8"
                )
            except (LookupError, OSError):
                reason = "interface not installed"
            else:
                original = definition.encoded_message_definition.split("\n===", 1)[0]
                if fields(original) == fields(current):
                    continue
                reason = "recorded fields differ from the installed interface"
            print(
                f"WARNING: {definition.topic_type}: {reason}; use compatible interfaces to inspect "
                "these historical messages. Recorded bytes are preserved, not converted.",
                file=sys.stderr,
                flush=True,
            )
    finally:
        reader.close()


def parse_range(start, end, duration_ns):
    if not math.isfinite(start) or start < 0:
        raise ValueError("start must be a finite, non-negative number of seconds")
    if not math.isfinite(end) or (end != -1 and end <= start):
        raise ValueError("end must be greater than start, or -1 for the bag end")
    start_ns = round(start * 1_000_000_000)
    if start_ns >= duration_ns:
        raise ValueError("start must be before the bag end")
    end_ns = duration_ns if end == -1 else round(end * 1_000_000_000)
    if end_ns > duration_ns:
        raise ValueError("end must not exceed the bag duration")
    if end_ns <= start_ns:
        raise ValueError("selected range must be at least one nanosecond")
    return start_ns, end_ns


def write_segment(source, destination, topics, start, end):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source)), rosbag2_py.ConverterOptions("", ""))
    metadata = reader.get_metadata()
    start_ns, end_ns = parse_range(start, end, metadata.duration.nanoseconds)
    origin = metadata.starting_time.nanoseconds
    first, last = origin + start_ns, origin + end_ns
    topic_metadata = [
        t for t in reader.get_all_topics_and_types() if topics is None or t.name in topics
    ]

    # 静态变换可能只在包开头出现；保留起点前每个 child 的最新值。
    static_transforms = {}
    latest_state = {}
    latest_dynamic = {}
    dynamic_history = deque()
    initialization_topics = [
        t.name
        for t in topic_metadata
        if t.name in ("/tf", "/tf_static", "/controller_state")
        or t.type == "sensor_msgs/msg/CameraInfo"
    ]
    if initialization_topics and start_ns > 0:
        reader.set_filter(rosbag2_py.StorageFilter(topics=initialization_topics))
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if stamp >= first:
                break
            if topic in ("/tf", "/tf_static"):
                for transform in deserialize_message(data, TFMessage).transforms:
                    if topic == "/tf_static":
                        static_transforms[transform.child_frame_id] = transform
                    else:
                        latest_dynamic[transform.child_frame_id] = transform
                        dynamic_history.append((stamp, transform))
                while dynamic_history and dynamic_history[0][0] < stamp - 500_000_000:
                    dynamic_history.popleft()
            else:
                latest_state[topic] = data

    reader.close()
    initialization = []
    if static_transforms:
        message = TFMessage(transforms=list(static_transforms.values()))
        initialization.append(("/tf_static", serialize_message(message)))
    if latest_dynamic:
        # 保留每个 frame 的最新状态以及起点前 0.5 秒的动态 TF，供时间插值。
        transforms = {
            (t.child_frame_id, t.header.stamp.sec, t.header.stamp.nanosec): t
            for t in [
                *latest_dynamic.values(),
                *(t for stamp, t in dynamic_history if stamp >= first - 500_000_000),
            ]
        }
        message = TFMessage(
            transforms=sorted(
                transforms.values(), key=lambda t: (t.header.stamp.sec, t.header.stamp.nanosec)
            )
        )
        initialization.append(("/tf", serialize_message(message)))
    initialization.extend(latest_state.items())
    with tempfile.TemporaryDirectory(prefix="dart_range_export_") as temporary:
        root = Path(temporary)

        def rewrite(sources, target, bounded):
            options = {"uri": str(target), "storage_id": "mcap"}
            if topics is None:
                options["all_topics"] = True
            else:
                options["topics"] = topics
            if bounded:
                options.update(start_time_ns=first, end_time_ns=last)
            options_path = root / "convert.yaml"
            with options_path.open("w", encoding="utf-8") as stream:
                yaml.safe_dump({"output_bags": [options]}, stream)
            rosbag2_py.bag_rewrite(
                [rosbag2_py.StorageOptions(uri=str(path)) for path in sources], str(options_path)
            )

        # C++ rewrite 保留源包中的消息描述；Python writer 会根据当前接口重建描述，
        # 不适合直接导出结构已变更的历史算法消息。
        body = root / "body" if initialization else destination
        rewrite([source], body, bounded=True)
        if rosbag2_py.MetadataIo().read_metadata(str(body)).message_count == 0:
            raise ValueError("selected range contains no recorded messages for the selected topics")
        if initialization:
            bootstrap = root / "bootstrap"
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bootstrap), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""),
            )
            try:
                names = {topic for topic, _ in initialization}
                for topic in topic_metadata:
                    if topic.name in names:
                        writer.create_topic(topic)
                for topic, data in initialization:
                    # Strictly precede boundary messages; equal timestamps have no
                    # stable ordering when rosbag2 merges separate storage readers.
                    writer.write(topic, data, first - 1)
            finally:
                writer.close()
            rewrite([body, bootstrap], destination, bounded=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--end", type=float, default=-1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--rate", default="1.0")
    parser.add_argument("--delay", default="3.0")
    parser.add_argument("--qos", required=True)
    parser.add_argument("--start-recording", action="store_true")
    parser.add_argument("--recording-config", type=Path)
    parser.add_argument("--recording-profile", default="full", choices=["inputs", "full"])
    parser.add_argument("--required-nodes", nargs="*", default=[])
    parser.add_argument("--failure-file", type=Path)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--topics", nargs="+")
    selection.add_argument(
        "--all-topics", action="store_true", help="Preserve all topics in the source bag"
    )
    args = parser.parse_args()
    if args.start_recording and (args.loop or args.all_topics or args.recording_config is None):
        parser.error(
            "Recording requires selected recompute inputs, --recording-config and no --loop"
        )
    report_interface_mismatches(args.bag, args.topics)
    with tempfile.TemporaryDirectory(prefix="dart_replay_segment_") as temporary:
        bag = args.bag
        if args.start != 0 or args.end != -1:
            bag = Path(temporary) / "bag"
            try:
                write_segment(args.bag, bag, args.topics, args.start, args.end)
            except ValueError as error:
                parser.error(str(error))
            print(
                f"Playing range {args.start}s to {args.end if args.end != -1 else 'end'}s",
                flush=True,
            )
        command = [
            "ros2",
            "bag",
            "play",
            str(bag),
            "--clock-topics-all",
            "--rate",
            args.rate,
            "--delay",
            args.delay,
            "--qos-profile-overrides-path",
            args.qos,
        ]
        if args.loop:
            command.append("--loop")
        if args.topics is not None:
            command.extend(["--topics", *args.topics])
        return run_player(command, args, temporary)


if __name__ == "__main__":
    raise SystemExit(main())
