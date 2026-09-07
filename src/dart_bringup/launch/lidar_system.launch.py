from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    start_description = LaunchConfiguration("start_description")
    launch_directory = PathJoinSubstitution(
        [FindPackageShare("dart_bringup"), "launch", "common"]
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dart_description"), "launch", "description.launch.py"]
            )
        ),
        condition=IfCondition(start_description),
    )

    online = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_directory, "lidar_online.launch.py"])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'online'"])),
    )
    offline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_directory, "lidar_offline.launch.py"])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'offline'"])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="online",
                description="LiDAR pipeline profile: online or offline",
                choices=["online", "offline"],
            ),
            DeclareLaunchArgument(
                "start_description",
                default_value="false",
                description="Publish the site-selected robot description and static TF",
            ),
            description,
            online,
            offline,
        ]
    )
