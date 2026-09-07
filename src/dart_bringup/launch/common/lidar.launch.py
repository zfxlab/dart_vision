from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    publish_freq = LaunchConfiguration("publish_freq")
    driver_params_file = LaunchConfiguration("driver_params_file")
    lidar_config_file = LaunchConfiguration("lidar_config_file")

    bringup_share = FindPackageShare("dart_bringup")
    default_driver_params = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "livox_driver.yaml"]
    )
    default_lidar_config = PathJoinSubstitution(
        [bringup_share, "config", "lidar", "livox_lidar_config.json"]
    )

    driver = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            driver_params_file,
            {
                "publish_freq": ParameterValue(publish_freq, value_type=float),
                "user_config_path": lidar_config_file,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_freq",
                default_value="20.0",
                description="Livox batch publication frequency in Hz",
            ),
            DeclareLaunchArgument("driver_params_file", default_value=default_driver_params),
            DeclareLaunchArgument("lidar_config_file", default_value=default_lidar_config),
            driver,
        ]
    )
