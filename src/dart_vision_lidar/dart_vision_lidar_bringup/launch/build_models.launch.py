"""Run the explicit offline Mid-70 model build once and then exit."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the model builder with an ordinary YAML job file."""
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("dart_vision_lidar_bringup"),
            "config",
            "model_build_example.yaml",
        ]
    )
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Ordinary YAML consumed by dart_vision_model_builder",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "dart_vision_lidar_model",
                    "dart_vision_model_builder",
                    "--config",
                    config_file,
                ],
                output="screen",
            ),
        ]
    )
