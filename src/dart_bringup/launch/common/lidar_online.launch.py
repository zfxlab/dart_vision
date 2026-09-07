from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_driver = LaunchConfiguration("start_driver")
    start_localization = LaunchConfiguration("start_localization")
    publish_freq = LaunchConfiguration("publish_freq")
    driver_params_file = LaunchConfiguration("driver_params_file")
    lidar_config_file = LaunchConfiguration("lidar_config_file")
    preprocessing_params_file = LaunchConfiguration("preprocessing_params_file")
    accumulation_params_file = LaunchConfiguration("accumulation_params_file")
    localization_params_file = LaunchConfiguration("localization_params_file")

    bringup_share = FindPackageShare("dart_bringup")
    default_driver_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "livox_driver.yaml"]
    )
    default_lidar_config = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "livox_lidar_config.json"]
    )
    default_preprocessing_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "online", "preprocessing.yaml"]
    )
    default_accumulation_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "online", "accumulation.yaml"]
    )
    default_localization_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "online", "localization.yaml"]
    )

    driver = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=IfCondition(start_driver),
        parameters=[
            driver_params_file,
            {
                "publish_freq": ParameterValue(publish_freq, value_type=float),
                "user_config_path": lidar_config_file,
            },
        ],
    )
    preprocessor = Node(
        package="dart_lidar_preprocessing",
        executable="livox_preprocessor_node",
        name="livox_preprocessor",
        output="screen",
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
                "start_driver",
                default_value="true",
                description="Start the Livox driver; disable when another process provides /livox/lidar",
            ),
            DeclareLaunchArgument(
                "publish_freq",
                default_value="20.0",
                description="Livox batch publication frequency in Hz",
            ),
            DeclareLaunchArgument(
                "start_localization",
                default_value="false",
                description="Start unified base and moving-module localization after model paths are configured",
            ),
            DeclareLaunchArgument("driver_params_file", default_value=default_driver_params),
            DeclareLaunchArgument("lidar_config_file", default_value=default_lidar_config),
            DeclareLaunchArgument(
                "preprocessing_params_file", default_value=default_preprocessing_params
            ),
            DeclareLaunchArgument(
                "accumulation_params_file", default_value=default_accumulation_params
            ),
            DeclareLaunchArgument(
                "localization_params_file", default_value=default_localization_params
            ),
            driver,
            preprocessor,
            accumulator,
            localization,
        ]
    )
