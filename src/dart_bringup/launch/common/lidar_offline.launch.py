from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_preprocessor = LaunchConfiguration("start_preprocessor")
    start_localization = LaunchConfiguration("start_localization")
    preprocessing_params_file = LaunchConfiguration("preprocessing_params_file")
    accumulation_params_file = LaunchConfiguration("accumulation_params_file")
    localization_params_file = LaunchConfiguration("localization_params_file")

    bringup_share = FindPackageShare("dart_bringup")
    default_preprocessing_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "offline", "preprocessing.yaml"]
    )
    default_accumulation_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "offline", "accumulation.yaml"]
    )
    default_localization_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "offline", "localization.yaml"]
    )

    preprocessor = Node(
        package="dart_lidar_preprocessing",
        executable="livox_preprocessor_node",
        name="livox_preprocessor",
        output="screen",
        condition=IfCondition(start_preprocessor),
        parameters=[preprocessing_params_file],
    )
    accumulator = Node(
        package="dart_lidar_accumulation",
        executable="lidar_accumulator_node",
        name="lidar_accumulator",
        output="screen",
        parameters=[accumulation_params_file],
    )
    localization = Node(
        package="dart_lidar_localization",
        executable="lidar_localization_node",
        name="lidar_localization",
        output="screen",
        condition=IfCondition(start_localization),
        parameters=[localization_params_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_preprocessor",
                default_value="true",
                description="Disable when the bag already contains lidar/preprocessed",
            ),
            DeclareLaunchArgument(
                "start_localization",
                default_value="true",
                description="Start unified base and moving-module localization after model paths are configured",
            ),
            DeclareLaunchArgument(
                "preprocessing_params_file", default_value=default_preprocessing_params
            ),
            DeclareLaunchArgument(
                "accumulation_params_file", default_value=default_accumulation_params
            ),
            DeclareLaunchArgument(
                "localization_params_file", default_value=default_localization_params
            ),
            preprocessor,
            accumulator,
            localization,
        ]
    )
