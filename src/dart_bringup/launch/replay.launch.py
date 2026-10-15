"""Review recorded topics or recompute with the current vision pipeline."""

import importlib.util
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def replay_actions(context):
    share = Path(get_package_share_directory("dart_bringup"))
    bag = Path(LaunchConfiguration("bag").perform(context)).expanduser().resolve()
    if (bag / "bag" / "metadata.yaml").is_file():
        bag /= "bag"
    if not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"Bag metadata not found: {bag}")
    mode = LaunchConfiguration("mode").perform(context)
    recording = IfCondition(LaunchConfiguration("start_recording")).evaluate(context)
    if recording and mode != "recompute":
        raise RuntimeError("start_recording is only supported in recompute mode")
    with (bag / "metadata.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    available = {
        entry["topic_metadata"]["name"]
        for entry in metadata["topics_with_message_count"]
        if entry["message_count"] > 0
    }
    config_path = Path(LaunchConfiguration("recording_config_file").perform(context))
    # Use the same profile inheritance/validation as the recorder and exporter.
    from ament_index_python.packages import get_package_prefix

    spec = importlib.util.spec_from_file_location(
        "dart_record_bag",
        Path(get_package_prefix("dart_bringup")) / "lib/dart_bringup/record_bag.py",
    )
    recorder_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recorder_module)
    topics = recorder_module.resolve_topics(
        recorder_module.load_configuration(config_path)["profiles"], "inputs"
    )
    loop_arguments = (
        ["--loop"] if IfCondition(LaunchConfiguration("loop")).evaluate(context) else []
    )
    if recording and loop_arguments:
        raise RuntimeError("Recording requires loop:=false; use one output session per replay run")
    recording_arguments = []
    if recording:
        recording_arguments = [
            "--start-recording",
            "--recording-config",
            str(config_path),
            "--recording-profile",
            LaunchConfiguration("recording_profile"),
        ]
    if mode not in ("recorded", "recompute"):
        raise RuntimeError(f"Unknown replay mode: {mode}")
    if mode == "recompute" and (missing := set(topics) - available):
        raise RuntimeError(
            f"Required recompute inputs missing or empty: {', '.join(sorted(missing))}"
        )
    topic_arguments = ["--topics", *topics] if mode == "recompute" else ["--all-topics"]
    actions = []
    failure_directory = tempfile.TemporaryDirectory(prefix="dart_replay_status_")
    failure_file = Path(failure_directory.name) / "failure.txt"
    if mode == "recompute":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "vision_system.launch.py")),
                launch_arguments={
                    "use_sim_time": "true",
                    "start_driver": "false",
                    "start_description": "false",
                    "start_serial": "false",
                    "start_recording": "false",
                }.items(),
            )
        )
    player = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "run",
            "dart_bringup",
            "play_replay_bag.py",
            str(bag),
            "--failure-file",
            str(failure_file),
            "--required-nodes",
            *(
                [
                    "/left_camera/green_light_detector",
                    "/right_camera/green_light_detector",
                    "/camera/stereo_triangulator",
                    "/aiming",
                    "/left_camera/replay_decoder",
                    "/right_camera/replay_decoder",
                ]
                if mode == "recompute"
                else []
            ),
            *loop_arguments,
            *recording_arguments,
            "--start",
            LaunchConfiguration("start"),
            "--end",
            LaunchConfiguration("end"),
            "--delay",
            LaunchConfiguration("delay"),
            "--rate",
            LaunchConfiguration("rate"),
            "--qos",
            str(share / "config" / "rosbag" / "replay_qos.yaml"),
            *topic_arguments,
        ],
        output="screen",
        sigterm_timeout="50",
        sigkill_timeout="10",
    )

    def process_exited(event, launch_context):
        # Retain the directory until the launch handlers have finished; the player
        # reads the failure reason before its recorder closes the manifest.
        _ = failure_directory
        if launch_context.is_shutdown:
            return []
        if event.action is player:
            if event.returncode != 0:
                raise RuntimeError(f"Replay failed with exit code {event.returncode}")
            return (
                [EmitEvent(event=Shutdown(reason="Recorded recompute finished"))]
                if recording
                else []
            )
        if isinstance(event.action, Node):
            reason = f"Required replay node exited: {event.process_name} ({event.returncode})"
            failure_file.write_text(reason, encoding="utf-8")
            raise RuntimeError(reason)
        return []

    actions.insert(0, RegisterEventHandler(OnProcessExit(on_exit=process_exited)))
    return [
        *actions,
        *[
            Node(
                package="image_transport",
                executable="republish",
                namespace=namespace,
                name="replay_decoder",
                parameters=[
                    {"use_sim_time": True, "in_transport": "compressed", "out_transport": "raw"}
                ],
                remappings=[("in/compressed", "image_raw/compressed"), ("out", "image_raw")],
                output="screen",
            )
            for namespace in ("left_camera", "right_camera")
            if f"/{namespace}/image_raw/compressed" in available
        ],
        player,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="Input bag or recording session directory"),
            DeclareLaunchArgument(
                "mode",
                default_value="recompute",
                choices=["recorded", "recompute"],
                description="Review all recorded topics, or run current algorithms on inputs",
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument(
                "start", default_value="0.0", description="Start offset in bag seconds"
            ),
            DeclareLaunchArgument(
                "end",
                default_value="-1.0",
                description="End offset in bag seconds; -1 means bag end",
            ),
            DeclareLaunchArgument("start_recording", default_value="false"),
            DeclareLaunchArgument(
                "recording_config_file",
                default_value=str(
                    Path(get_package_share_directory("dart_bringup"))
                    / "config/rosbag/recording.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "recording_profile", default_value="full", choices=["inputs", "full"]
            ),
            DeclareLaunchArgument(
                "loop",
                default_value=PythonExpression(
                    [
                        "'false' if '",
                        LaunchConfiguration("start_recording"),
                        "'.lower() in ('true', '1') else 'true'",
                    ]
                ),
                description="Loop playback; defaults to false when recording",
            ),
            DeclareLaunchArgument(
                "delay", default_value="3.0", description="Discovery delay in seconds"
            ),
            OpaqueFunction(function=replay_actions),
        ]
    )
