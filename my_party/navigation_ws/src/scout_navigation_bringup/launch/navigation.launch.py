from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def _launch_navigation(context):
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    map_yaml = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    params_argument = LaunchConfiguration("params_file").perform(context)
    params_file = (
        Path(params_argument).expanduser().resolve()
        if params_argument
        else map_yaml.parent / "nav2_params.yaml"
    )
    if not map_yaml.is_file():
        raise RuntimeError(f"Nav2 map YAML does not exist: {map_yaml}")
    if not params_file.is_file():
        raise RuntimeError(
            f"Per-map Nav2 parameters do not exist: {params_file}"
        )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            str(params_file),
            {
                "yaml_filename": str(map_yaml),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": str(params_file),
            "use_composition": "False",
        }.items(),
    )

    return [map_server, map_lifecycle_manager, navigation]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=(
                    "/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01/nav2_map.yaml"
                ),
                description="Absolute path to the Nav2 map YAML.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description=(
                    "Optional Nav2 parameter YAML override; defaults to "
                    "<map YAML directory>/nav2_params.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically activate lifecycle nodes.",
            ),
            OpaqueFunction(function=_launch_navigation),
        ]
    )
