#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: play_output_bag_only.sh /path/to/output_bag [ros2 bag play args...]

Only play a validation output bag. This script does not start Planner and does
not start RViz.

Environment overrides:
  OUTPUT_BAG     alternative to first positional argument
  PLAY_LOOP      default: true
  PLAY_RATE      default: 1.0
  ROS_DOMAIN_ID  default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

if [[ $# -gt 0 && "${1}" != --* ]]; then
  OUTPUT_BAG="$1"
  shift
else
  OUTPUT_BAG="${OUTPUT_BAG:-}"
fi

if [[ -z "${OUTPUT_BAG}" ]]; then
  echo "Usage: $0 /path/to/output_bag [ros2 bag play args...]" >&2
  echo "Or set OUTPUT_BAG=/path/to/output_bag" >&2
  exit 1
fi
require_path "${OUTPUT_BAG}" "output bag"

PLAY_LOOP="${PLAY_LOOP:-true}"
PLAY_RATE="${PLAY_RATE:-1.0}"

play_args=(--rate "${PLAY_RATE}")
if is_true "${PLAY_LOOP}"; then
  play_args+=(--loop)
fi

echo "Playing output bag only: ${OUTPUT_BAG}"
exec ros2 bag play "${OUTPUT_BAG}" "${play_args[@]}" "$@"
