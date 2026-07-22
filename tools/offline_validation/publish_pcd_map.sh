#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: publish_pcd_map.sh

Publish a PCD map as /map_generator/global_cloud for RViz reference.

Environment overrides:
  PCD_MAP_FILE        default: repo/scans.pcd
  GRID_FRAME_ID       default: lio_odom
  PCD_DOWNSAMPLE_RES  default: 0.1
  PCD_PUBLISH_RATE    default: 0.2 Hz
  ROS_DOMAIN_ID       default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

PCD_MAP_FILE="${PCD_MAP_FILE:-${DEFAULT_PCD_MAP}}"
GRID_FRAME_ID="${GRID_FRAME_ID:-lio_odom}"
PCD_DOWNSAMPLE_RES="${PCD_DOWNSAMPLE_RES:-0.1}"
PCD_PUBLISH_RATE="${PCD_PUBLISH_RATE:-0.2}"

require_path "${PCD_MAP_FILE}" "PCD map"

echo "Publishing PCD map as /map_generator/global_cloud:"
echo "  file=${PCD_MAP_FILE}"
echo "  frame=${GRID_FRAME_ID}"

exec ros2 run map_generator map_pub \
  --ros-args \
  -r __ns:=/map_generator \
  -p file_name:="${PCD_MAP_FILE}" \
  -p frame_id:="${GRID_FRAME_ID}" \
  -p publish_rate:="${PCD_PUBLISH_RATE}" \
  -p downsample_res:="${PCD_DOWNSAMPLE_RES}"
