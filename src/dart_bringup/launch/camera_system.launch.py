# 可选机器人描述、双相机驱动、左右绿灯检测与双目三角测量

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_driver = LaunchConfiguration("start_driver")
    cameras_file = LaunchConfiguration("cameras_file")
    detector_params_file = LaunchConfiguration("green_light_detector_params_file")
    stereo_params_file = LaunchConfiguration("stereo_triangulator_params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    config_directory = PathJoinSubstitution([FindPackageShare("dart_bringup"), "config", "camera"])
    default_cameras_file = PathJoinSubstitution([config_directory, "cameras.yaml"])
    default_detector_params_file = PathJoinSubstitution(
        [config_directory, "green_light_detector.yaml"]
    )
    default_stereo_params_file = PathJoinSubstitution(
        [config_directory, "stereo_triangulator.yaml"]
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dart_description"), "launch", "description.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_description")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("hik_camera_driver"), "launch", "multi_camera.launch.py"]
            )
        ),
        condition=IfCondition(start_driver),
        launch_arguments={"cameras_file": cameras_file, "use_composition": "true"}.items(),
    )

    standalone_detector_nodes = [
        Node(
            package="dart_camera",
            executable="green_light_detector_node",
            namespace=namespace,
            name="green_light_detector",
            output="screen",
            emulate_tty=True,
            parameters=[detector_params_file, {"use_sim_time": use_sim_time}],
            condition=UnlessCondition(start_driver),
        )
        for namespace in ("left_camera", "right_camera")
    ]

    detector_components = [
        LoadComposableNodes(
            target_container=f"/{namespace}/camera_pipeline",
            condition=IfCondition(start_driver),
            composable_node_descriptions=[
                ComposableNode(
                    package="dart_camera",
                    plugin="dart_vision::camera::GreenLightDetectorNode",
                    namespace=f"/{namespace}",
                    name="green_light_detector",
                    parameters=[detector_params_file, {"use_sim_time": use_sim_time}],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        )
        for namespace in ("left_camera", "right_camera")
    ]

    stereo_node = Node(
        package="dart_stereo",
        executable="stereo_node",
        namespace="camera",
        name="stereo_triangulator",
        output="screen",
        emulate_tty=True,
        parameters=[stereo_params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "start_description",
                default_value="true",
                description="Start dart_description to publish the robot TF",
            ),
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
                default_value=default_detector_params_file,
                description="Green-light detector parameter YAML file",
            ),
            DeclareLaunchArgument(
                "stereo_triangulator_params_file",
                default_value=default_stereo_params_file,
                description="Stereo frame and triangulation parameter YAML file",
            ),
            description_launch,
            camera_launch,
            *standalone_detector_nodes,
            *detector_components,
            stereo_node,
        ]
    )
