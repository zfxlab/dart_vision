from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_offset_names = [
        "base_offset_x",
        "base_offset_y",
        "base_offset_z",
        "base_offset_roll",
        "base_offset_pitch",
        "base_offset_yaw",
    ]

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("dart_description"),
            "urdf",
            "dart_system.urdf.xacro",
        ]
    )

    xacro_command = ["xacro ", xacro_file]
    for name in base_offset_names:
        xacro_command.extend(
            [" ", name, ":=", LaunchConfiguration(name)]
        )

    robot_description = ParameterValue(
        Command(xacro_command),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            *[
                DeclareLaunchArgument(name, default_value="0.0")
                for name in base_offset_names
            ],
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
