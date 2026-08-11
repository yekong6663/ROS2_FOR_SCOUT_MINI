#!/usr/bin/env python3
"""Dock from indoor_02 placement staging point to the placement point.

Normal Nav2 first reaches record point 4 precisely.  The final 0.85 m is a
verified, fixed-heading forward crawl to record point 5, so Nav2 does not try
to turn inside the narrow placement area.  A narrow front point-cloud stop
zone remains active.
"""

import sys


# Insert defaults before user-supplied ROS arguments so a later explicit
# parameter can still override them for a measured re-teach.
DEFAULTS = [
    "--ros-args",
    "-p", "staging_x:=7.697",
    "-p", "staging_y:=-0.466",
    "-p", "staging_yaw:=3.141",
    "-p", "goal_x:=6.844",
    "-p", "goal_y:=-0.440",
    "-p", "goal_yaw:=-3.140",
    "-p", "position_tolerance:=0.08",
    "-p", "yaw_tolerance:=0.08",
    "-p", "crawl_speed:=0.10",
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
