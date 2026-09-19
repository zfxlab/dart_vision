from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration("use_sim_time")

    def include(package, path, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(package), "launch", path])
            ),
            **kwargs,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_driver", default_value="true"),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument("start_serial", default_value="true"),
            include(
                "dart_bringup",
                "camera_system.launch.py",
                launch_arguments={
                    "use_sim_time": sim,
                    "start_driver": LaunchConfiguration("start_driver"),
                    "start_description": LaunchConfiguration("start_description"),
                }.items(),
            ),
            include(
                "dart_serial",
                "serial.launch.py",
                condition=IfCondition(LaunchConfiguration("start_serial")),
                launch_arguments={"use_sim_time": sim}.items(),
            ),
            Node(
                package="dart_aiming",
                executable="aiming_node",
                name="aiming",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("dart_bringup"), "config", "aiming.yaml"]
                    ),
                    {"use_sim_time": ParameterValue(sim, value_type=bool)},
                ],
            ),
        ]
    )
