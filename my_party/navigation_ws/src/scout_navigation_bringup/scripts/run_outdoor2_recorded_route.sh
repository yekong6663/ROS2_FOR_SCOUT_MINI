#!/usr/bin/env bash
set -euo pipefail

# outdoor_02 two-cycle mission: red-flag start, then two passes of photo-card
# recognition, grasp, target-box alignment and placement through one bridge.
# Temporary test switches:
#   RED_FLAG_START_ENABLED=0  skip the red-flag gate
#   ARM_HANDOFF_ENABLED=0    skip arm preflight, pickup and placement handoffs
exec ros2 run scout_navigation_bringup run_outdoor2_recorded_route.py "$@"
