from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    node_name = LaunchConfiguration("node_name")

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("dart_camera"),
            "config",
            "green_light_detector.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Green-light detector parameter YAML file",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="camera",
                description="Green-light detector node namespace",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="green_light_detector",
                description="Green-light detector node name",
            ),
            Node(
                package="dart_camera",
                executable="green_light_detector_node",
                namespace=namespace,
                name=node_name,
                output="screen",
                emulate_tty=True,
                parameters=[params_file],
            ),
        ]
    )
