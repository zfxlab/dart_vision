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
        [FindPackageShare("dart_serial"), "config", "serial.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Serial node parameter YAML file",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Serial node namespace",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="serial_node",
                description="Serial node name",
            ),
            Node(
                package="dart_serial",
                executable="serial_node",
                namespace=namespace,
                name=node_name,
                output="screen",
                emulate_tty=True,
                parameters=[params_file],
            ),
        ]
    )
