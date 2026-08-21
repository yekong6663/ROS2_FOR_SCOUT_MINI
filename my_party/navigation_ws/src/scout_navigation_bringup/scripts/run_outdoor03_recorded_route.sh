#!/usr/bin/env bash
set -euo pipefail

# outdoor_03 uses maps/outdoor_03/recorded_poses.yaml.
# SKIP_OUTBOUND=1 skips point 1 and starts from turn point 2.
# RED_FLAG_START_ENABLED=0 skips the red-flag gate.
# ARM_HANDOFF_ENABLED=0 runs navigation only.
exec ros2 run scout_navigation_bringup run_outdoor03_recorded_route.py "$@"
