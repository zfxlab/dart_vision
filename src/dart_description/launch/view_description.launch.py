from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    description_share = FindPackageShare("dart_description")

    xacro_file = PathJoinSubstitution(
        [
            description_share,
            "urdf",
            "dart_system.urdf.xacro",
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            description_share,
            "rviz",
            "description.rviz",
        ]
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    description_parameter = {"robot_description": robot_description}

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dart_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            description_launch,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[description_parameter],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
