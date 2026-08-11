#!/usr/bin/env bash
set -euo pipefail

# Compatibility command: the full route now stays in one ROS process so DDS
# discovery is paid only once instead of after every recorded target.
exec ros2 run scout_navigation_bringup run_indoor03_recorded_route.py "$@"
