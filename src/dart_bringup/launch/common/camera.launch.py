# 相机驱动 + 相机识别

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_driver = LaunchConfiguration("start_driver")
    cameras_file = LaunchConfiguration("cameras_file")
    green_light_detector_params_file = LaunchConfiguration("green_light_detector_params_file")
    green_light_detector_namespace = LaunchConfiguration("green_light_detector_namespace")
    green_light_detector_node_name = LaunchConfiguration("green_light_detector_node_name")

    default_cameras_file = PathJoinSubstitution(
        [FindPackageShare("dart_bringup"), "config", "camera", "cameras.yaml"]
    )
    default_green_light_detector_params_file = PathJoinSubstitution(
        [
            FindPackageShare("dart_bringup"),
            "config",
            "camera",
            "green_light_detector.yaml",
        ]
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hik_camera_driver"),
                    "launch",
                    "multi_camera.launch.py",
                ]
            )
        ),
        condition=IfCondition(start_driver),
        launch_arguments={"cameras_file": cameras_file}.items(),
    )

    green_light_detector_node = Node(
        package="dart_camera",
        executable="green_light_detector_node",
        namespace=green_light_detector_namespace,
        name=green_light_detector_node_name,
        output="screen",
        emulate_tty=True,
        parameters=[green_light_detector_params_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_driver",
                default_value="true",
                description="Start camera drivers; disable for recorded or external images",
            ),
            DeclareLaunchArgument(
                "cameras_file",
                default_value=default_cameras_file,
                description="Multi-camera configuration YAML file",
            ),
            DeclareLaunchArgument(
                "green_light_detector_params_file",
                default_value=default_green_light_detector_params_file,
                description="Green-light detector parameter YAML file",
            ),
            DeclareLaunchArgument(
                "green_light_detector_namespace",
                default_value="camera",
                description="Green-light detector node namespace",
            ),
            DeclareLaunchArgument(
                "green_light_detector_node_name",
                default_value="green_light_detector",
                description="Green-light detector node name",
            ),
            camera_launch,
            green_light_detector_node,
        ]
    )
