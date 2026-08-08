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
  CLOUD_TOPIC   default: /cloud_registered_global
  BODY_ODOM_TOPIC default: /body_state_estimation_global
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

OUTPUT_BAG="${OUTPUT_BAG:-/tmp/scanplanner_offline_outputs_$(date +%Y%m%d_%H%M%S)}"
BODY_ODOM_TOPIC="${BODY_ODOM_TOPIC:-/body_state_estimation_global}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered_global}"

if [[ -e "${OUTPUT_BAG}" ]]; then
  echo "ERROR: output bag already exists: ${OUTPUT_BAG}" >&2
  exit 1
fi

topics=(
  /planning/bspline
  /planning/planner_heartbeat
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
  "${CLOUD_TOPIC}"
)

if [[ $# -gt 0 ]]; then
  topics=("$@")
fi

echo "Recording Planner outputs to ${OUTPUT_BAG}"
printf '  %s\n' "${topics[@]}"

exec ros2 bag record -o "${OUTPUT_BAG}" "${topics[@]}"
