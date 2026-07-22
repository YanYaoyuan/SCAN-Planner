#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export SCAN_PLANNER_REPO_ROOT="${SCAN_PLANNER_REPO_ROOT:-${REPO_ROOT}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/scanplanner_ros_logs}"

DEFAULT_OFFLINE_BAG="/home/user/robot/data/gangbeng/rosbag2_1970_01_01-09_53_08"
DEFAULT_PCD_MAP="${SCAN_PLANNER_REPO_ROOT}/scans.pcd"

mkdir -p "${ROS_LOG_DIR}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash

if [[ ! -f "${SCAN_PLANNER_REPO_ROOT}/install/setup.bash" ]]; then
  echo "ERROR: ${SCAN_PLANNER_REPO_ROOT}/install/setup.bash not found." >&2
  echo "Build first, for example: colcon build --symlink-install --packages-up-to scan_planner" >&2
  exit 1
fi
source "${SCAN_PLANNER_REPO_ROOT}/install/setup.bash"

require_path() {
  local path="$1"
  local description="$2"
  if [[ ! -e "${path}" ]]; then
    echo "ERROR: ${description} does not exist: ${path}" >&2
    exit 1
  fi
}

is_true() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}
