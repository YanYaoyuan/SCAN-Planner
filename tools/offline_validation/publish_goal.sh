#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: publish_goal.sh

Publish one PoseStamped goal to /move_base_simple/goal.

Environment overrides:
  GOAL_FRAME   default: lio_odom
  GOAL_X       default: 2.381195423852814
  GOAL_Y       default: 4.589078052270621
  GOAL_Z       default: 0.2
  GOAL_YAW     default: 0.0 radians
  GOAL_TOPIC   default: /move_base_simple/goal
  ROS_DOMAIN_ID default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

GOAL_FRAME="${GOAL_FRAME:-lio_odom}"
GOAL_X="${GOAL_X:-2.381195423852814}"
GOAL_Y="${GOAL_Y:-4.589078052270621}"
GOAL_Z="${GOAL_Z:-0.2}"
GOAL_YAW="${GOAL_YAW:-0.0}"
GOAL_TOPIC="${GOAL_TOPIC:-/move_base_simple/goal}"

read -r QZ QW < <(python3 - "${GOAL_YAW}" <<'PY'
import math
import sys

yaw = float(sys.argv[1])
print(math.sin(yaw * 0.5), math.cos(yaw * 0.5))
PY
)

echo "Publishing goal on ${GOAL_TOPIC}: frame=${GOAL_FRAME}, xyz=(${GOAL_X}, ${GOAL_Y}, ${GOAL_Z}), yaw=${GOAL_YAW}"

exec ros2 topic pub --once "${GOAL_TOPIC}" geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: ${GOAL_FRAME}}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}, orientation: {z: ${QZ}, w: ${QW}}}}"
