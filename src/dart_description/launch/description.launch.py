import math
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

BASE_OFFSET_NAMES = [
    "base_offset_x",
    "base_offset_y",
    "base_offset_z",
    "base_offset_roll",
    "base_offset_pitch",
    "base_offset_yaw",
]


def _read_vector(document, color, key):
    value = document.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError(f"site {color}.{key} must be a three-element list")

    result = []
    for element in value:
        if isinstance(element, bool) or not isinstance(element, (int, float)):
            raise RuntimeError(f"site {color}.{key} must contain only numbers")
        number = float(element)
        if not math.isfinite(number):
            raise RuntimeError(f"site {color}.{key} contains a non-finite value")
        result.append(number)
    return result


def _load_site(path):
    if not os.path.isfile(path):
        raise RuntimeError(f"site file does not exist: {path}")

    try:
        with open(path, encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"failed to read site file {path}: {error}") from error

    if not isinstance(document, dict):
        raise RuntimeError("site file root must be a map")
    if not isinstance(document.get("is_red"), bool):
        raise RuntimeError("site is_red must be true or false")

    transforms = {}
    for color in ("red", "blue"):
        transform = document.get(color)
        if not isinstance(transform, dict):
            raise RuntimeError(f"site file must contain a {color} map")
        transforms[color] = _read_vector(transform, color, "xyz_m") + _read_vector(
            transform, color, "rpy_rad"
        )

    selected_color = "red" if document["is_red"] else "blue"
    return selected_color, transforms[selected_color]


def _launch_setup(context):
    site_path = os.path.abspath(
        os.path.expanduser(LaunchConfiguration("site_file").perform(context))
    )
    color, offsets = _load_site(site_path)

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("dart_description"), "urdf", "dart_system.urdf.xacro"]
    )
    xacro_command = ["xacro ", xacro_file]
    for name, value in zip(BASE_OFFSET_NAMES, offsets):
        xacro_command.extend([" ", name, ":=", repr(value)])

    get_logger("dart_description").info(
        "Loaded %s site from %s: xyz=[%.6f, %.6f, %.6f] m, "
        "rpy=[%.6f, %.6f, %.6f] rad" % (color, site_path, *offsets)
    )

    robot_description = ParameterValue(Command(xacro_command), value_type=str)
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )
    ]


def generate_launch_description():
    default_site_file = PathJoinSubstitution(
        [
            FindPackageShare("dart_description"),
            "config",
            "site",
            "default.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "site_file",
                default_value=default_site_file,
                description="Red/blue site YAML loaded before xacro is evaluated",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
