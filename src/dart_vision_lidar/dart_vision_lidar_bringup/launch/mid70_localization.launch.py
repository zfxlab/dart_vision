"""Launch Mid-70 localization and, optionally, its RViz deployment view."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_STRING_PARAMETER_ARGUMENTS = {
    "input_type": "input_type",
    "pointcloud2_topic": "pointcloud2_topic",
    "livox_custom_topic": "livox_custom_topic",
    "output_frame": "output_frame",
    "base_open_pcd": "model.base_open_pcd",
    "base_closed_pcd": "model.base_closed_pcd",
    "armor_pcd": "model.armor_pcd",
}

_PACKAGED_MODEL_FILES = {
    "base_open_pcd": "base_open_fixed.pcd",
    "base_closed_pcd": "base_closed_fixed.pcd",
    "armor_pcd": "moving_armor.pcd",
}


def _launch_setup(context):
    parameter_overrides = {}
    for argument_name, parameter_name in _STRING_PARAMETER_ARGUMENTS.items():
        value = LaunchConfiguration(argument_name).perform(context)
        if not value and argument_name in _PACKAGED_MODEL_FILES:
            value = PathJoinSubstitution(
                [
                    FindPackageShare("dart_vision_lidar_model"),
                    "models",
                    "processed",
                    _PACKAGED_MODEL_FILES[argument_name],
                ]
            ).perform(context)
        if value:
            parameter_overrides[parameter_name] = value

    localization = Node(
        package="dart_vision_lidar_localization",
        executable="mid70_localization_node",
        name="mid70_localization_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"), parameter_overrides],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("dart_vision_lidar_bringup"),
            "rviz",
            "mid70_localization.rviz",
        ]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="mid70_localization_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return [localization, rviz]


def generate_launch_description():
    """Declare deployment overrides and create localization/RViz actions."""
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("dart_vision_lidar_bringup"),
            "config",
            "mid70_localization.yaml",
        ]
    )

    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Localization parameter YAML file",
        ),
        DeclareLaunchArgument(
            "input_type",
            default_value="",
            description="pointcloud2 or livox_custom; empty uses params_file",
        ),
        DeclareLaunchArgument(
            "pointcloud2_topic",
            default_value="",
            description="PointCloud2 topic; empty uses params_file",
        ),
        DeclareLaunchArgument(
            "livox_custom_topic",
            default_value="",
            description="livox_interfaces/CustomMsg topic; empty uses params_file",
        ),
        DeclareLaunchArgument(
            "output_frame",
            default_value="",
            description="Result frame; empty uses params_file",
        ),
        DeclareLaunchArgument(
            "base_open_pcd",
            default_value="",
            description="Open-state fixed-base PCD; empty uses packaged model",
        ),
        DeclareLaunchArgument(
            "base_closed_pcd",
            default_value="",
            description="Closed-state fixed-base PCD; empty uses packaged model",
        ),
        DeclareLaunchArgument(
            "armor_pcd",
            default_value="",
            description="Moving-armor PCD; empty uses packaged model",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            choices=["true", "false"],
            description="Start RViz with the Mid-70 debug display",
        ),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_setup)])
