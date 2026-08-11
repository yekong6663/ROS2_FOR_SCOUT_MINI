#!/usr/bin/env bash
# indoor_02 current full route: precision pre-grasp -> slow precision grasp -> arm handoff.
# Later route stages are intentionally disabled until the turn and placement
# area have been retuned and revalidated.
set -euo pipefail

route_root="/home/nvidia/auto/ROS2_FOR_SCOUT_MINI"
grasp_handoff="/home/nvidia/auto/Robot_arm/source/scripts/run_navigation_grasp_handoff.sh"
red_flag_start_gate="/home/nvidia/auto/Robot_arm/source/scripts/wait_for_red_flag_start.sh"
# Grasp identity is now supplied by the printed photo card at the arm
# observation pose. Keep route arguments navigation-only and never forward a
# stale/manual item identity into the grasp handoff.
handoff_args=()

# Refuse before moving the base if the arm stack, target, fine-scan controller,
# or odometry needed for the handoff is unavailable.
"${grasp_handoff}" --preflight "${handoff_args[@]}"

# Keep Nav2 stopped until the eye-in-hand camera observes a genuinely waved
# red flag. Set RED_FLAG_START_ENABLED=0 only for explicit bench debugging.
if [[ "${RED_FLAG_START_ENABLED:-1}" == "1" ]]; then
  "${red_flag_start_gate}"
else
  echo "WARNING: red-flag start gate bypassed by RED_FLAG_START_ENABLED=0" >&2
fi

# Record point 1 is reached with ±0.08 m / ±0.08 rad precision.  From there
# the robot keeps the taught heading and moves straight at 0.08 m/s to record
# point 2, also with ±0.08 m / ±0.08 rad precision, at 0.12 m/s.  The final crawl ignores
# Nav2's local costmap but preserves a narrow point-cloud emergency stop.
ros2 run scout_navigation_bringup dock_to_indoor_grasp_point.py

# set -e guarantees this is reached only after both navigation actions report
# success. From here the distributed arm pipeline owns the operation.
"${grasp_handoff}" "${handoff_args[@]}"
