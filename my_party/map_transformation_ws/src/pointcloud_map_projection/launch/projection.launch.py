from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("pointcloud_map_projection"),
        "config",
        "projection.yaml",
    )
    return LaunchDescription([
        Node(
            package="pointcloud_map_projection",
            executable="pointcloud_map_projection_node",
            name="pointcloud_map_projection",
            output="screen",
            parameters=[params_file],
        ),
    ])
