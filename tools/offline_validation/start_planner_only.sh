#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: start_planner_only.sh

Only start SCAN-Planner in offline real-input mode. This script does not play
any bag, does not record outputs, and does not start RViz.

Environment overrides are the same as run_planner_with_bag_inputs.sh:
  PCD_MAP_FILE       default: repo/scans.pcd
  GRID_FRAME_ID      default: lio_odom
  LIDAR_ODOM_TOPIC   default: /state_estimation
  CLOUD_TOPIC        default: /cloud_registered
  BODY_ODOM_TOPIC    default: /body_state_estimation
  CMD_VEL_TOPIC      default: /scan_planner/cmd_vel
  ROS_DOMAIN_ID      default: 73
EOF
  exit 0
fi

exec "${SCRIPT_DIR}/run_planner_with_bag_inputs.sh" "$@"
