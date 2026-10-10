from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("dart_bringup"), "config", "recording.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="compressed",
                description="Recording profile from recording.yaml",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Rosbag recording configuration YAML file",
            ),
            ExecuteProcess(
                cmd=[
                    FindExecutable(name="ros2"),
                    "run",
                    "dart_bringup",
                    "record_bag.py",
                    "--config",
                    LaunchConfiguration("config_file"),
                    "--profile",
                    LaunchConfiguration("profile"),
                ],
                output="screen",
            ),
        ]
    )
