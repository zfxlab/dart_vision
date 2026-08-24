"""Optionally launch the bundled Livox driver, localization, and RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _driver_condition(expected_input_type):
    start_driver = LaunchConfiguration("start_livox_driver")
    input_type = LaunchConfiguration("input_type")
    return IfCondition(
        PythonExpression(
            [
                "'",
                start_driver,
                "'.lower() == 'true' and '",
                input_type,
                f"' == '{expected_input_type}'",
            ]
        )
    )


def generate_launch_description():
    """Compose an opt-in legacy Livox driver with the deployment launch."""
    bringup_share = FindPackageShare("dart_vision_lidar_bringup")
    livox_share = FindPackageShare("livox_ros2_driver")
    default_params = PathJoinSubstitution([bringup_share, "config", "mid70_localization.yaml"])

    launch_argument_defaults = {
        "params_file": default_params,
        "input_type": "pointcloud2",
        "pointcloud2_topic": "",
        "livox_custom_topic": "",
        "output_frame": "",
        "base_open_pcd": "",
        "base_closed_pcd": "",
        "armor_pcd": "",
        "use_rviz": "false",
    }
    declarations = [
        DeclareLaunchArgument(
            name,
            default_value=default,
            description=f"Forwarded localization launch argument: {name}",
        )
        for name, default in launch_argument_defaults.items()
    ]
    declarations.append(
        DeclareLaunchArgument(
            "start_livox_driver",
            default_value="false",
            choices=["true", "false"],
            description="Start the bundled legacy Livox driver",
        )
    )

    pointcloud2_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, "launch", "livox_lidar_launch.py"])
        ),
        condition=_driver_condition("pointcloud2"),
    )
    custom_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, "launch", "livox_lidar_msg_launch.py"])
        ),
        condition=_driver_condition("livox_custom"),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "mid70_localization.launch.py"])
        ),
        launch_arguments={
            name: LaunchConfiguration(name) for name in launch_argument_defaults
        }.items(),
    )

    return LaunchDescription([*declarations, pointcloud2_driver, custom_driver, localization])
