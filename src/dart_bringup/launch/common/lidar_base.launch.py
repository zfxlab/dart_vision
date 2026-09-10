from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_driver = LaunchConfiguration("start_driver")
    start_preprocessor = LaunchConfiguration("start_preprocessor")
    start_accumulator = LaunchConfiguration("start_accumulator")
    start_calibration = LaunchConfiguration("start_calibration")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_freq = LaunchConfiguration("publish_freq")
    driver_params_file = LaunchConfiguration("driver_params_file")
    lidar_config_file = LaunchConfiguration("lidar_config_file")
    preprocessing_params_file = LaunchConfiguration("preprocessing_params_file")
    accumulation_params_file = LaunchConfiguration("accumulation_params_file")
    calibration_params_file = LaunchConfiguration("calibration_params_file")

    bringup_share = FindPackageShare("dart_bringup")
    default_driver_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "livox_driver.yaml"]
    )
    default_lidar_config = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "livox_lidar_config.json"]
    )
    default_preprocessing_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "base", "preprocessing.yaml"]
    )
    default_accumulation_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "base", "accumulation.yaml"]
    )
    default_calibration_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "base", "base_calibration.yaml"]
    )

    common_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
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
                **common_time,
            },
        ],
    )
    preprocessor = Node(
        package="dart_lidar_preprocessing",
        executable="livox_preprocessor_node",
        name="livox_preprocessor",
        output="screen",
        condition=IfCondition(start_preprocessor),
        parameters=[preprocessing_params_file, common_time],
    )
    accumulator = Node(
        package="dart_lidar_accumulation",
        executable="lidar_accumulator_node",
        name="lidar_accumulator",
        output="screen",
        condition=IfCondition(start_accumulator),
        parameters=[accumulation_params_file, common_time],
    )
    calibration = Node(
        package="dart_lidar_localization",
        executable="base_calibration_node",
        name="base_calibration",
        output="screen",
        condition=IfCondition(start_calibration),
        parameters=[calibration_params_file, common_time],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_driver", default_value="true"),
            DeclareLaunchArgument("start_preprocessor", default_value="true"),
            DeclareLaunchArgument("start_accumulator", default_value="true"),
            DeclareLaunchArgument(
                "start_calibration",
                default_value="true",
                description="Start the node selected by mode; false runs preprocessing/accumulation only",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "publish_freq",
                default_value="20.0",
                description="Livox batch publication frequency in Hz",
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
                "calibration_params_file", default_value=default_calibration_params
            ),
            driver,
            preprocessor,
            accumulator,
            calibration
        ]
    )
