#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: record_outputs.sh [topic ...]

Record Planner outputs. With no topics, records the standard validation set.

Environment overrides:
  OUTPUT_BAG    default: /tmp/scanplanner_offline_outputs_YYYYmmdd_HHMMSS
  ROS_DOMAIN_ID default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

OUTPUT_BAG="${OUTPUT_BAG:-/tmp/scanplanner_offline_outputs_$(date +%Y%m%d_%H%M%S)}"
BODY_ODOM_TOPIC="${BODY_ODOM_TOPIC:-/body_state_estimation}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"

if [[ -e "${OUTPUT_BAG}" ]]; then
  echo "ERROR: output bag already exists: ${OUTPUT_BAG}" >&2
  exit 1
fi

topics=(
  /planning/bspline
  /planning/controller_bspline
  /planning/controller_target
  /optimal_list
  /init_list
  /a_star_list
  /global_list
  /goal_point
  /grid_map/occupancy
  /grid_map/occupancy_inflate
  /self_inflation
  "${BODY_ODOM_TOPIC}"
  "${CMD_VEL_TOPIC}"
  /planning/go2_execution_frozen
)

if [[ $# -gt 0 ]]; then
  topics=("$@")
fi

echo "Recording Planner outputs to ${OUTPUT_BAG}"
printf '  %s\n' "${topics[@]}"

exec ros2 bag record -o "${OUTPUT_BAG}" "${topics[@]}"
