#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: resume_input_replay.sh

Resume play_input_bag_only.sh / replay_input_only.py after pausing.

Environment overrides:
  INPUT_REPLAY_PAUSE_TOPIC default: /scanplanner/input_replay_pause
  ROS_DOMAIN_ID            default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

INPUT_REPLAY_PAUSE_TOPIC="${INPUT_REPLAY_PAUSE_TOPIC:-/scanplanner/input_replay_pause}"

exec ros2 topic pub --once "${INPUT_REPLAY_PAUSE_TOPIC}" std_msgs/msg/Bool "{data: false}"
