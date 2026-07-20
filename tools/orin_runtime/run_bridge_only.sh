#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"

exec ros2 launch zsibot_cmd_bridge zsibot_cmd_bridge.launch.py \
  cmd_vel_topic:="${CMD_VEL_TOPIC}"
