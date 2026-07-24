import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_navigation_system(context):
    map_dir = Path(LaunchConfiguration("map_dir").perform(context)).expanduser().resolve()
    if not map_dir.is_dir():
        raise RuntimeError(f"Map directory does not exist: {map_dir}")

    pcd_path = map_dir / "map.pcd"
    nav2_map = map_dir / "nav2_map.yaml"
    for required_file in (pcd_path, nav2_map):
        if not required_file.is_file():
            raise RuntimeError(f"Required navigation map does not exist: {required_file}")

    with nav2_map.open("r", encoding="utf-8") as stream:
        nav2_map_metadata = yaml.safe_load(stream) or {}
    image_value = nav2_map_metadata.get("image")
    if not image_value:
        raise RuntimeError(f"Nav2 map YAML has no image field: {nav2_map}")
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = nav2_map.parent / image_path
    if not image_path.is_file():
        raise RuntimeError(f"Nav2 map image does not exist: {image_path}")

    pose_file_argument = LaunchConfiguration("initial_pose_file").perform(context)
    pose_file = (
        Path(pose_file_argument).expanduser().resolve()
        if pose_file_argument
        else map_dir / "initial_pose.yaml"
    )

    fastlio_share = get_package_share_directory("fastlio2")
    localizer_share = get_package_share_directory("localizer")
    livox_share = get_package_share_directory("livox_ros_driver2")
    scout_base_share = get_package_share_directory("scout_base")
    scout_plugins_share = get_package_share_directory("scout_navigation_plugins")
    nav_bringup_share = get_package_share_directory("scout_navigation_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    start_livox = LaunchConfiguration("start_livox")
    start_base = LaunchConfiguration("start_base")
    start_localization = LaunchConfiguration("start_localization")
    start_rviz = LaunchConfiguration("start_rviz")
    can_port = LaunchConfiguration("can_port")

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360s_launch.py")
        ),
        condition=IfCondition(start_livox),
    )

    scout_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scout_base_share, "launch", "scout_mini_base.launch.py")
        ),
        condition=IfCondition(start_base),
        launch_arguments={
            "port_name": can_port,
            "publish_odom_tf": "false",
        }.items(),
    )

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scout_plugins_share, "launch", "robot_tf.launch.py")
        )
    )

    fastlio = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[
            {"config_path": os.path.join(fastlio_share, "config", "lio.yaml")}
        ],
        condition=IfCondition(start_localization),
    )

    localizer = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {
                "config_path": os.path.join(
                    localizer_share, "config", "localizer.yaml"
                )
            }
        ],
        condition=IfCondition(start_localization),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_rviz",
        output="screen",
        arguments=[
            "-d",
            os.path.join(nav2_bringup_share, "rviz", "nav2_default_view.rviz"),
        ],
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
                "pitch": float(
                    LaunchConfiguration("initial_pitch").perform(context)
                ),
                "roll": float(LaunchConfiguration("initial_roll").perform(context)),
                "timeout": float(
                    LaunchConfiguration("relocalization_timeout").perform(context)
                ),
            }
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "map": str(nav2_map),
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )
    start_nav2 = _as_bool(LaunchConfiguration("start_nav2").perform(context))

    def on_relocalization_exit(event, _context):
        if event.returncode == 0:
            success_message = (
                f"Relocalization succeeded for {pcd_path}; "
                f"starting Nav2 with {nav2_map}"
                if start_nav2
                else f"Relocalization succeeded for {pcd_path}"
            )
            actions = [
                LogInfo(msg=success_message)
            ]
            if start_nav2:
                actions.append(nav2)
            else:
                actions.append(LogInfo(msg="start_nav2=false; Nav2 remains stopped"))
            return actions

        return [
            LogInfo(
                msg=(
                    "ERROR: Relocalization gate failed with exit code "
                    f"{event.returncode}; Nav2 will not start"
                )
            ),
            EmitEvent(
                event=Shutdown(reason="Relocalization failed; Nav2 startup blocked")
            ),
        ]

    gate_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=relocalization_gate,
            on_exit=on_relocalization_exit,
        )
    )

    startup_message = LogInfo(
        msg=(
            f"Navigation map directory: {map_dir}; 3D map: {pcd_path}; "
            f"2D map: {nav2_map}; initial pose file: {pose_file}"
        )
    )

    return [
        gate_exit_handler,
        startup_message,
        livox_driver,
        scout_base,
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
                description=(
                    "Map folder containing map.pcd and nav2_map.yaml. "
                    "initial_pose.yaml is loaded automatically when present."
                ),
            ),
            DeclareLaunchArgument(
                "initial_pose_file",
                default_value="",
                description=(
                    "Optional pose YAML; defaults to <map_dir>/initial_pose.yaml."
                ),
            ),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pitch", default_value="0.0"),
            DeclareLaunchArgument("initial_roll", default_value="0.0"),
            DeclareLaunchArgument(
                "relocalization_timeout",
                default_value="120.0",
                description="Maximum seconds to wait for valid relocalization.",
            ),
            DeclareLaunchArgument(
                "start_livox",
                default_value="true",
                description="Start the Livox MID-360s driver.",
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="true",
                description="Start the Scout Mini base driver.",
            ),
            DeclareLaunchArgument(
                "start_localization",
                default_value="true",
                description=(
                    "Start FAST-LIO2 and localizer. Set false only when an "
                    "external localization stack already provides the services."
                ),
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the Nav2 default view.",
            ),
            DeclareLaunchArgument(
                "start_nav2",
                default_value="true",
                description="Start Nav2 after relocalization becomes valid.",
            ),
            DeclareLaunchArgument(
                "can_port",
                default_value="can0",
                description="Scout Mini SocketCAN interface.",
            ),
            OpaqueFunction(function=_launch_navigation_system),
        ]
    )
