from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_conversion(context):
    map_dir = Path(LaunchConfiguration("map_dir").perform(context)).expanduser().resolve()
    params_file = Path(
        LaunchConfiguration("params_file").perform(context)
    ).expanduser().resolve()
    output_name = LaunchConfiguration("output_name").perform(context)
    save_timeout = LaunchConfiguration("save_timeout").perform(context)
    remove_self_points = _as_bool(
        LaunchConfiguration("remove_self_points").perform(context)
    )

    if not map_dir.is_dir():
        raise RuntimeError(f"Map directory does not exist: {map_dir}")
    if not params_file.is_file():
        raise RuntimeError(f"Projection parameter file does not exist: {params_file}")
    if not output_name or Path(output_name).name != output_name:
        raise RuntimeError("output_name must be a simple filename without directories")

    pcd_file = map_dir / "map.pcd"
    poses_file = map_dir / "poses.txt"
    patches_dir = map_dir / "patches"
    for required_file in (pcd_file, poses_file):
        if not required_file.is_file():
            raise RuntimeError(f"Required map input does not exist: {required_file}")
    if remove_self_points and (
        not patches_dir.is_dir() or not any(patches_dir.iterdir())
    ):
        raise RuntimeError(
            "remove_self_points=true requires a non-empty patches directory: "
            f"{patches_dir}"
        )

    projection = Node(
        package="pointcloud_map_projection",
        executable="pointcloud_map_projection_node",
        name="pointcloud_map_projection",
        output="screen",
        parameters=[
            str(params_file),
            {
                "pcd_file": str(pcd_file),
                "poses_file": str(poses_file),
                "patches_dir": str(patches_dir),
                "remove_self_points": remove_self_points,
            },
        ],
    )

    map_saver = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-t",
            "/map",
            "-f",
            str(map_dir / output_name),
            "--fmt",
            "png",
            "--ros-args",
            "-p",
            f"save_map_timeout:={save_timeout}",
            "-p",
            "map_subscribe_transient_local:=true",
        ],
        output="screen",
    )

    def on_map_saver_exit(event, _context):
        if event.returncode == 0:
            message = LogInfo(
                msg=f"2D Nav2 map saved to {map_dir / output_name}.yaml"
            )
            reason = "Map conversion completed"
        else:
            message = LogInfo(
                msg=f"ERROR: map_saver_cli failed with exit code {event.returncode}"
            )
            reason = "Map conversion failed"
        return [message, EmitEvent(event=Shutdown(reason=reason))]

    saver_exit_handler = RegisterEventHandler(
        OnProcessExit(target_action=map_saver, on_exit=on_map_saver_exit)
    )

    return [saver_exit_handler, projection, map_saver]


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory("scout_navigation_bringup"))
        / "config"
        / "map_projection_params.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_dir",
                description=(
                    "Map folder containing map.pcd and poses.txt; patches/ is "
                    "required only when remove_self_points=true."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Point-cloud projection tuning parameters.",
            ),
            DeclareLaunchArgument(
                "output_name",
                default_value="nav2_map",
                description="Output basename inside map_dir.",
            ),
            DeclareLaunchArgument(
                "remove_self_points",
                default_value="false",
                description="Rebuild from patches and remove Scout self-reflections.",
            ),
            DeclareLaunchArgument(
                "save_timeout",
                default_value="180.0",
                description="Seconds map_saver_cli waits for the projected map.",
            ),
            OpaqueFunction(function=_launch_conversion),
        ]
    )
