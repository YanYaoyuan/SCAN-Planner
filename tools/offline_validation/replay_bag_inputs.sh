#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

if [[ $# -gt 0 && "${1}" != --* ]]; then
  BAG_PATH="$1"
  shift
else
  BAG_PATH="${BAG_PATH:-${DEFAULT_OFFLINE_BAG}}"
fi

require_path "${BAG_PATH}" "input bag"

REPLAY_RATE="${REPLAY_RATE:-1.0}"
REPLAY_LOOP="${REPLAY_LOOP:-true}"
REPLAY_QOS="${REPLAY_QOS:-best_effort}"

loop_args=()
if is_true "${REPLAY_LOOP}"; then
  loop_args+=(--loop)
fi

duration_args=()
if [[ -n "${REPLAY_DURATION:-}" ]]; then
  duration_args+=(--duration "${REPLAY_DURATION}")
fi

exec python3 "${SCRIPT_DIR}/replay_input_only.py" \
  --bag "${BAG_PATH}" \
  --rate "${REPLAY_RATE}" \
  --qos "${REPLAY_QOS}" \
  "${loop_args[@]}" \
  "${duration_args[@]}" \
  "$@"
