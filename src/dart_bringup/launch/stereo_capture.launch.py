"""Camera drivers and a calibration-pair capture window, without the aiming pipeline."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("dart_bringup"), "config", "camera"])
    capture = Node(
        package="dart_bringup",
        executable="capture_stereo_pairs.py",
        name="stereo_capture",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "output_root": LaunchConfiguration("output_root"),
                "preview_enabled": ParameterValue(
                    LaunchConfiguration("preview_enabled"), value_type=bool
                ),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_driver", default_value="true"),
            DeclareLaunchArgument("preview_enabled", default_value="true"),
            DeclareLaunchArgument("output_root", default_value="captures"),
            DeclareLaunchArgument(
                "params_file", default_value=PathJoinSubstitution([config, "stereo_capture.yaml"])
            ),
            DeclareLaunchArgument(
                "cameras_file", default_value=PathJoinSubstitution([config, "cameras.yaml"])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("hik_camera_driver"), "launch", "multi_camera.launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_driver")),
                launch_arguments={
                    "cameras_file": LaunchConfiguration("cameras_file"),
                    "use_composition": "true",
                }.items(),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=capture,
                    on_exit=[EmitEvent(event=Shutdown(reason="Calibration capture exited"))],
                )
            ),
            capture,
        ]
    )
