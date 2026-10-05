import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_directory = PathJoinSubstitution(
        [FindPackageShare("dart_bringup"), "config", "lidar"]
    )
    default_driver_params_file = PathJoinSubstitution(
        [config_directory, "livox_driver.yaml"]
    )
    default_lidar_config_file = PathJoinSubstitution(
        [config_directory, "livox_lidar_config.json"]
    )

    driver_params_file = LaunchConfiguration("livox_driver_params_file")
    lidar_config_file = LaunchConfiguration("livox_lidar_config_file")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )

    livox_driver = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            driver_params_file,
            {
                "user_config_path": ParameterValue(
                    lidar_config_file, value_type=str
                ),
                "cmdline_input_bd_code": "",
                "use_sim_time": use_sim_time,
            },
        ],
    )

    dart_base_to_lidar_mount = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="dart_base_to_lidar_mount_tf",
        output="screen",
        arguments=[
            "--x",
            "0.3787",
            "--y",
            "0.1718",
            "--z",
            "0.393",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            str(math.radians(-7.8)),
            "--frame-id",
            "dart_base_link",
            "--child-frame-id",
            "lidar_mount_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    lidar_mount_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_mount_to_livox_tf",
        output="screen",
        arguments=[
            "--x",
            "0.0323",
            "--y",
            "0.0",
            "--z",
            "0.0324",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "lidar_mount_link",
            "--child-frame-id",
            "livox_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "livox_driver_params_file",
                default_value=default_driver_params_file,
                description="Livox ROS parameter YAML file",
            ),
            DeclareLaunchArgument(
                "livox_lidar_config_file",
                default_value=default_lidar_config_file,
                description="Livox device configuration JSON file",
            ),
            livox_driver,
            dart_base_to_lidar_mount,
            lidar_mount_to_livox,
        ]
    )
