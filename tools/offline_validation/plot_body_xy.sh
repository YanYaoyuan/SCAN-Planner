#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: plot_body_xy.sh [/path/to/bag] [plot_body_xy.py args...]

Plot /body_state_estimation.pose.pose.position.x and y from a ROS 2 bag.

Examples:
  tools/offline_validation/plot_body_xy.sh
  tools/offline_validation/plot_body_xy.sh /path/to/bag -o /tmp/body_xy.png
  tools/offline_validation/plot_body_xy.sh --csv /tmp/body_xy.csv

Environment overrides:
  BAG_PATH      default: /home/user/robot/data/gangbeng/rosbag2_1970_01_01-09_53_08
  ROS_DOMAIN_ID default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

if [[ $# -gt 0 && "${1}" != -* ]]; then
  BAG_PATH="$1"
  shift
else
  BAG_PATH="${BAG_PATH:-${DEFAULT_OFFLINE_BAG}}"
fi

require_path "${BAG_PATH}" "bag"

exec python3 "${SCRIPT_DIR}/plot_body_xy.py" "${BAG_PATH}" "$@"
