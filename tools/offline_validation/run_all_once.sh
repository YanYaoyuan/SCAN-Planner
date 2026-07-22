#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: run_all_once.sh

Run one full offline validation:
  start Planner, replay input topics, record outputs, publish one goal, stop.

Environment overrides:
  BAG_PATH                default: /home/user/robot/data/gangbeng/rosbag2_1970_01_01-09_53_08
  PCD_MAP_FILE            default: repo/scans.pcd
  OUTPUT_BAG              default: /tmp/scanplanner_offline_outputs_YYYYmmdd_HHMMSS
  GOAL_X/GOAL_Y/GOAL_Z    default: validated sample goal
  RUN_SECONDS_AFTER_GOAL  default: 40
  WAIT_FOR_INPUT_SECONDS  default: 30
  ROS_DOMAIN_ID           default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

BAG_PATH="${BAG_PATH:-${DEFAULT_OFFLINE_BAG}}"
PCD_MAP_FILE="${PCD_MAP_FILE:-${DEFAULT_PCD_MAP}}"
OUTPUT_BAG="${OUTPUT_BAG:-/tmp/scanplanner_offline_outputs_$(date +%Y%m%d_%H%M%S)}"
WAIT_FOR_INPUT_SECONDS="${WAIT_FOR_INPUT_SECONDS:-30}"
RUN_SECONDS_AFTER_GOAL="${RUN_SECONDS_AFTER_GOAL:-40}"

require_path "${BAG_PATH}" "input bag"
require_path "${PCD_MAP_FILE}" "PCD map"

pids=()
cleanup() {
  trap - EXIT INT TERM
  if ((${#pids[@]} > 0)); then
    echo "Stopping background validation processes..."
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting one-shot offline validation:"
echo "  input bag=${BAG_PATH}"
echo "  pcd map=${PCD_MAP_FILE}"
echo "  output bag=${OUTPUT_BAG}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

PCD_MAP_FILE="${PCD_MAP_FILE}" "${SCRIPT_DIR}/start_planner_only.sh" &
pids+=("$!")
sleep 2

"${SCRIPT_DIR}/play_input_bag_only.sh" "${BAG_PATH}" &
pids+=("$!")
sleep 2

OUTPUT_BAG="${OUTPUT_BAG}" "${SCRIPT_DIR}/record_outputs.sh" &
pids+=("$!")

echo "Waiting for /body_state_estimation..."
if ! timeout "${WAIT_FOR_INPUT_SECONDS}" ros2 topic echo /body_state_estimation --once >/tmp/scanplanner_offline_body_odom.txt 2>&1; then
  echo "ERROR: /body_state_estimation did not arrive within ${WAIT_FOR_INPUT_SECONDS}s." >&2
  echo "See /tmp/scanplanner_offline_body_odom.txt for ros2 topic echo output." >&2
  exit 1
fi

"${SCRIPT_DIR}/publish_goal.sh"

echo "Running for ${RUN_SECONDS_AFTER_GOAL}s after goal..."
sleep "${RUN_SECONDS_AFTER_GOAL}"

cleanup
echo "Offline validation finished."
echo "Output bag: ${OUTPUT_BAG}"
