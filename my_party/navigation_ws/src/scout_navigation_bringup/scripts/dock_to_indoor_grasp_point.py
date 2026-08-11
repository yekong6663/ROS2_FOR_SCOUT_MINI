#!/usr/bin/env python3
"""Precisely approach the indoor_02 grasp point from its staging point.

Nav2 first settles at record point 1.  It then keeps the taught heading and
crawls straight to record point 2.  This prevents a late turn close to the
grasp fixture while retaining a narrow front point-cloud emergency stop.
"""

import sys


# Defaults are inserted before user parameters so measured re-teaching can
# still override any value from the command line.
DEFAULTS = [
    "--ros-args",
    "-p", "staging_x:=4.473",
    "-p", "staging_y:=-0.412",
    "-p", "staging_yaw:=-0.014",
    "-p", "goal_x:=5.317",
    "-p", "goal_y:=-0.423",
    "-p", "goal_yaw:=-0.017",
    "-p", "position_tolerance:=0.08",
    "-p", "yaw_tolerance:=0.08",
    "-p", "crawl_speed:=0.12",
    "-p", "crawl_timeout:=30.0",
    "-p", "max_yaw_rate:=0.14",
    "-p", "alignment_tolerance:=0.05",
    "-p", "front_safety_enabled:=true",
    "-p", "front_stop_distance:=0.35",
    "-p", "front_half_width:=0.16",
    "-p", "front_min_points:=4",
    "-p",
    "precision_behavior_tree:=/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/"
    "my_party/navigation_ws/src/scout_navigation_bringup/behavior_trees/"
    "navigate_to_pose_indoor_precision.xml",
]

sys.argv[1:1] = DEFAULTS

from dock_to_recorded_point3 import main  # noqa: E402


if __name__ == "__main__":
    main()
