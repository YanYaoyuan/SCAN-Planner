#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: play_outputs_with_rviz.sh /path/to/output_bag [ros2 bag play args...]

Start RViz, publish the PCD map, then play a validation output bag.

Environment overrides:
  OUTPUT_BAG            alternative to first positional argument
  PLAY_LOOP             default: true
  PLAY_RATE             default: 1.0
  GRID_FRAME_ID         default: lio_odom
  PUBLISH_WORLD_ALIAS   default: true
  PUBLISH_PCD_MAP       default: true
  ROS_DOMAIN_ID         default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

if [[ $# -gt 0 && "${1}" != --* ]]; then
  OUTPUT_BAG="$1"
  shift
else
  OUTPUT_BAG="${OUTPUT_BAG:-}"
fi

if [[ -z "${OUTPUT_BAG}" ]]; then
  echo "Usage: $0 /path/to/output_bag [ros2 bag play args...]" >&2
  echo "Or set OUTPUT_BAG=/path/to/output_bag" >&2
  exit 1
fi
require_path "${OUTPUT_BAG}" "output bag"

GRID_FRAME_ID="${GRID_FRAME_ID:-lio_odom}"
PUBLISH_WORLD_ALIAS="${PUBLISH_WORLD_ALIAS:-true}"
PUBLISH_PCD_MAP="${PUBLISH_PCD_MAP:-true}"
PLAY_LOOP="${PLAY_LOOP:-true}"
PLAY_RATE="${PLAY_RATE:-1.0}"

pids=()
cleanup() {
  trap - EXIT INT TERM
  if ((${#pids[@]} > 0)); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if is_true "${PUBLISH_WORLD_ALIAS}"; then
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world "${GRID_FRAME_ID}" &
  pids+=("$!")
fi

if is_true "${PUBLISH_PCD_MAP}"; then
  "${SCRIPT_DIR}/publish_pcd_map.sh" &
  pids+=("$!")
fi

ros2 launch scan_planner rviz.launch.py &
pids+=("$!")
sleep 2

play_args=(--rate "${PLAY_RATE}")
if is_true "${PLAY_LOOP}"; then
  play_args+=(--loop)
fi

echo "Playing output bag: ${OUTPUT_BAG}"
ros2 bag play "${OUTPUT_BAG}" "${play_args[@]}" "$@"
