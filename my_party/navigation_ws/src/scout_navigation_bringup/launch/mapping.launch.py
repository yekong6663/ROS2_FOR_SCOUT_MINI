import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fastlio_share = get_package_share_directory("fastlio2")
    pgo_share = get_package_share_directory("pgo")
    livox_share = get_package_share_directory("livox_ros_driver2")
    scout_base_share = get_package_share_directory("scout_base")
    scout_plugins_share = get_package_share_directory("scout_navigation_plugins")

    start_livox = LaunchConfiguration("start_livox")
    start_base = LaunchConfiguration("start_base")
    start_robot_tf = LaunchConfiguration("start_robot_tf")
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
        ),
        condition=IfCondition(start_robot_tf),
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
    )

    pgo = Node(
        package="pgo",
        namespace="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": os.path.join(pgo_share, "config", "pgo.yaml")}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="mapping_rviz",
        output="screen",
        arguments=["-d", os.path.join(pgo_share, "rviz", "pgo.rviz")],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_livox",
                default_value="true",
                description="Start the Livox MID-360s driver.",
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="false",
                description=(
                    "Start Scout base for ROS teleoperation. Keep false when "
                    "using the original remote controller."
                ),
            ),
            DeclareLaunchArgument(
                "start_robot_tf",
                default_value="true",
                description="Publish the static body to base_link transform.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the PGO mapping view.",
            ),
            DeclareLaunchArgument(
                "can_port",
                default_value="can1",
                description="Scout Mini SocketCAN interface.",
            ),
            livox_driver,
            scout_base,
            robot_tf,
            fastlio,
            pgo,
            rviz,
        ]
    )
