from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("scout_navigation_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "yaml_filename": map_yaml,
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
            "params_file": params_file,
            "use_composition": "False",
        }.items(),
    )

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
                default_value=os.path.join(
                    bringup_share, "config", "nav2_params.yaml"
                ),
                description="Absolute path to the Nav2 parameter file.",
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
            map_server,
            map_lifecycle_manager,
            navigation,
        ]
    )
