# 中间相机驱动与独立绿灯检测器；不启动机器人描述或双目处理。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cameras_file = LaunchConfiguration("cameras_file")
    detector_params_file = LaunchConfiguration("green_light_detector_params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    config_directory = PathJoinSubstitution(
        [FindPackageShare("dart_bringup"), "config", "camera"]
    )
    default_cameras_file = PathJoinSubstitution(
        [config_directory, "center_camera.yaml"]
    )
    default_detector_params_file = PathJoinSubstitution(
        [config_directory, "center_green_light_detector.yaml"]
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("hik_camera_driver"), "launch", "multi_camera.launch.py"]
            )
        ),
        launch_arguments={"cameras_file": cameras_file, "use_composition": "true"}.items(),
    )

    detector_component = LoadComposableNodes(
        target_container="/center_camera/camera_pipeline",
        composable_node_descriptions=[
            ComposableNode(
                package="dart_camera",
                plugin="dart_vision::camera::GreenLightDetectorNode",
                namespace="/center_camera",
                name="green_light_detector",
                parameters=[detector_params_file, {"use_sim_time": use_sim_time}],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "cameras_file",
                default_value=default_cameras_file,
                description="Center-camera configuration YAML file",
            ),
            DeclareLaunchArgument(
                "green_light_detector_params_file",
                default_value=default_detector_params_file,
                description="Center-camera green-light detector parameter YAML file",
            ),
            camera_launch,
            detector_component,
        ]
    )
