#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: start_rviz_live.sh

Start RViz plus optional helpers for live offline validation.

Environment overrides:
  GRID_FRAME_ID         default: lio_odom
  PUBLISH_WORLD_ALIAS   default: true, publishes world -> GRID_FRAME_ID
  PUBLISH_PCD_MAP       default: true, publishes scans.pcd for RViz
  PCD_MAP_FILE          default: repo/scans.pcd
  ROS_DOMAIN_ID         default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

GRID_FRAME_ID="${GRID_FRAME_ID:-lio_odom}"
PUBLISH_WORLD_ALIAS="${PUBLISH_WORLD_ALIAS:-true}"
PUBLISH_PCD_MAP="${PUBLISH_PCD_MAP:-true}"

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

echo "Starting RViz. Use Fixed Frame 'world' or '${GRID_FRAME_ID}'."
ros2 launch scan_planner rviz.launch.py
