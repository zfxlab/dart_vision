from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("dart_bringup")

    def path(*parts):
        return PathJoinSubstitution([share, *parts])

    time = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        )
    }
    include = lambda p, args: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(p), launch_arguments=args.items()
    )
    nodes = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_drivers", default_value="true"),
        DeclareLaunchArgument("start_serial", default_value="true"),
        DeclareLaunchArgument(
            "site_file", default_value=path("config", "site", "default.yaml")
        ),
        DeclareLaunchArgument(
            "system_params", default_value=path("config", "system.yaml")
        ),
        DeclareLaunchArgument("lidar_timestamp_source", default_value="receive"),
        include(
            PathJoinSubstitution(
                [
                    FindPackageShare("dart_description"),
                    "launch",
                    "description.launch.py",
                ]
            ),
            {
                "site_file": LaunchConfiguration("site_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ),
        include(
            path("launch", "common", "camera.launch.py"),
            {
                "start_driver": LaunchConfiguration("start_drivers"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ),
        Node(
            package="livox_ros2_driver",
            executable="livox_ros2_driver_node",
            name="livox_lidar_publisher",
            condition=IfCondition(LaunchConfiguration("start_drivers")),
            output="screen",
            parameters=[
                path("config", "lidar", "livox_driver.yaml"),
                time,
                {
                    "user_config_path": path(
                        "config", "lidar", "livox_lidar_config.json"
                    )
                },
            ],
        ),
        Node(
            package="dart_serial",
            executable="serial_node",
            name="serial_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_serial")),
            parameters=[path("config", "serial.yaml"), time],
        ),
    ]
    for package, name in [
        ("dart_lidar", "lidar"),
        ("dart_target_estimation", "target_estimation"),
        ("dart_aiming", "aiming"),
    ]:
        parameters = [LaunchConfiguration("system_params"), time]
        if name == "lidar":
            parameters.append(
                {
                    "base_model": path("models", "pcd", "base_pnx_5mm.pcd"),
                    "module_model": PathJoinSubstitution(
                        [
                            FindPackageShare("dart_description"),
                            "meshes",
                            "pnx_dart_detection_module_link.stl",
                        ]
                    ),
                    "timestamp_source": LaunchConfiguration("lidar_timestamp_source"),
                }
            )
        nodes.append(
            Node(
                package=package,
                executable=package + "_node",
                name=name,
                output="screen",
                parameters=parameters,
            )
        )
    return LaunchDescription(nodes)
