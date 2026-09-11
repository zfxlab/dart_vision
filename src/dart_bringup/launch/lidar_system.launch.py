# The lidar state machine needs description and the estimator's observation window.
# Use the complete system with physical devices disabled for recorded cloud debugging.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("dart_bringup"), "launch", "system.launch.py"]
                    )
                ),
                launch_arguments={
                    "start_drivers": "false",
                    "start_serial": "false",
                    "lidar_timestamp_source": "header",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
