"""Start only the sensors and localization stack for a saved 3D map test."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_relocalization_test(context):
    map_dir = Path(LaunchConfiguration("map_dir").perform(context)).expanduser().resolve()
    if not map_dir.is_dir():
        raise RuntimeError(f"Map directory does not exist: {map_dir}")

    pcd_path = map_dir / "map.pcd"
    if not pcd_path.is_file():
        raise RuntimeError(f"3D map does not exist: {pcd_path}")

    pose_file_argument = LaunchConfiguration("initial_pose_file").perform(context)
    pose_file = (
        Path(pose_file_argument).expanduser().resolve()
        if pose_file_argument
        else map_dir / "initial_pose.yaml"
    )

    fastlio_share = get_package_share_directory("fastlio2")
    localizer_share = get_package_share_directory("localizer")
    livox_share = get_package_share_directory("livox_ros_driver2")
    plugins_share = get_package_share_directory("scout_navigation_plugins")
    bringup_share = get_package_share_directory("scout_navigation_bringup")

    start_livox = LaunchConfiguration("start_livox")
    start_robot_tf = LaunchConfiguration("start_robot_tf")
    start_rviz = LaunchConfiguration("start_rviz")

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360s_launch.py")
        ),
        condition=IfCondition(start_livox),
    )

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plugins_share, "launch", "robot_tf.launch.py")
        ),
        condition=IfCondition(start_robot_tf),
    )

    fastlio = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": os.path.join(fastlio_share, "config", "lio.yaml")}],
    )

    localizer = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": os.path.join(localizer_share, "config", "localizer.yaml")}
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="relocalization_rviz",
        output="screen",
        arguments=["-d", os.path.join(localizer_share, "rviz", "localizer.rviz")],
        condition=IfCondition(start_rviz),
    )

    relocalization_gate = Node(
        package="scout_navigation_bringup",
        executable="relocalize_and_wait.py",
        name="relocalization_gate",
        output="screen",
        parameters=[
            {
                "pcd_path": str(pcd_path),
                "pose_file": str(pose_file),
                "x": float(LaunchConfiguration("initial_x").perform(context)),
                "y": float(LaunchConfiguration("initial_y").perform(context)),
                "z": float(LaunchConfiguration("initial_z").perform(context)),
                "yaw": float(LaunchConfiguration("initial_yaw").perform(context)),
                "pitch": float(LaunchConfiguration("initial_pitch").perform(context)),
                "roll": float(LaunchConfiguration("initial_roll").perform(context)),
                "timeout": float(LaunchConfiguration("relocalization_timeout").perform(context)),
            }
        ],
    )

    return [
        LogInfo(
            msg=(
                f"Relocalization test map: {pcd_path}; initial pose file: {pose_file}. "
                "PGO, Nav2, and the Scout base are intentionally not started."
            )
        ),
        livox_driver,
        robot_tf,
        fastlio,
        localizer,
        rviz,
        relocalization_gate,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_dir",
                description="Directory containing map.pcd and optional initial_pose.yaml.",
            ),
            DeclareLaunchArgument("initial_pose_file", default_value=""),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pitch", default_value="0.0"),
            DeclareLaunchArgument("initial_roll", default_value="0.0"),
            DeclareLaunchArgument(
                "relocalization_timeout",
                default_value="120.0",
                description="Maximum seconds to wait for valid localization.",
            ),
            DeclareLaunchArgument(
                "start_livox",
                default_value="true",
                description="Start the Livox MID-360s driver.",
            ),
            DeclareLaunchArgument(
                "start_robot_tf",
                default_value="true",
                description="Publish the static body to base_link transform.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the localizer view.",
            ),
            OpaqueFunction(function=_launch_relocalization_test),
        ]
    )
