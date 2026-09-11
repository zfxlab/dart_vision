from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_driver", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("dart_bringup"),
                            "launch",
                            "common",
                            "camera.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    k: LaunchConfiguration(k) for k in ("use_sim_time", "start_driver")
                }.items(),
            ),
        ]
    )
