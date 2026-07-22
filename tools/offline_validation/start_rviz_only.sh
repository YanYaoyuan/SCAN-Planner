#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: start_rviz_only.sh

Only start RViz with the SCAN-Planner config. This script does not publish PCD
map, does not publish helper TF, and does not play any bag.

Environment overrides:
  ROS_DOMAIN_ID default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

exec ros2 launch scan_planner rviz.launch.py "$@"
