from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("dart_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_description = LaunchConfiguration("start_description")
    start_serial = LaunchConfiguration("start_serial")
    start_driver = LaunchConfiguration("start_driver")
    site_file = LaunchConfiguration("site_file")
    mode = LaunchConfiguration("mode")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dart_description"), "launch", "description.launch.py"]
            )
        ),
        condition=IfCondition(start_description),
        launch_arguments={"use_sim_time": use_sim_time, "site_file": site_file}.items(),
    )
    serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("dart_serial"), "launch", "serial.launch.py"])
        ),
        condition=IfCondition(start_serial),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "common", "lidar_calibration.launch.py"])
        ),
        condition=LaunchConfigurationEquals("mode", "base"),
        launch_arguments={
            "start_driver": start_driver,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "common", "lidar_localization.launch.py"])
        ),
        condition=LaunchConfigurationEquals("mode", "module"),
        launch_arguments={
            "start_driver": start_driver,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode", default_value="module", choices=["base", "module"]
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument(
                "start_serial",
                default_value="true",
                description="Publish launcher_yaw_joint; disable for rosbag playback",
            ),
            DeclareLaunchArgument(
                "start_driver",
                default_value="true",
                description="Disable when rosbag or another process publishes /livox/lidar",
            ),
            DeclareLaunchArgument(
                "site_file",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "site", "default.yaml"]
                ),
            ),
            description,
            serial,
            calibration,
            localization
        ]
    )
