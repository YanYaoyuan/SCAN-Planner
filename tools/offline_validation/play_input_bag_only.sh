#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: play_input_bag_only.sh [/path/to/input_bag] [replay_input_only.py args...]

Only replay Planner input topics from a bag. This script does not start
Planner, does not record outputs, and does not start RViz.

Default topics:
  /state_estimation
  /cloud_registered

Environment overrides:
  BAG_PATH         default: /home/user/robot/data/gangbeng/rosbag2_1970_01_01-09_53_08
  REPLAY_RATE      default: 1.0
  REPLAY_LOOP      default: true
  REPLAY_DURATION  default: unset
  ROS topic pause: tools/offline_validation/pause_input_replay.sh
  ROS topic resume: tools/offline_validation/resume_input_replay.sh
  ROS_DOMAIN_ID    default: 73
EOF
  exit 0
fi

exec "${SCRIPT_DIR}/replay_bag_inputs.sh" "$@"
