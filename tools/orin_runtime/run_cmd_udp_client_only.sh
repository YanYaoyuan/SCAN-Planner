#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${ENABLE_DEPRECATED_ZSIBOT_TRANSPORT:-0}" != "1" ]]; then
  echo "ERROR: zsibot_cmd_udp_client is deprecated and disabled by default." >&2
  echo "       Prefer the unified robot bridge. For an isolated migration test, run:" >&2
  echo "       ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 $0" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"

exec ros2 launch zsibot_cmd_bridge zsibot_cmd_udp_client.launch.py \
  enable_deprecated_zsibot_transport:=true \
  cmd_vel_topic:="${CMD_VEL_TOPIC}"
