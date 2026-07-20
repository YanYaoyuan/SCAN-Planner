#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"

exec "${SCRIPT_DIR}/zsibot_sdk_proxy" \
  --listen-ip 0.0.0.0 \
  --listen-port 44000 \
  --sdk-local-ip 127.0.0.1 \
  --sdk-local-port 43988 \
  --sdk-dog-ip 127.0.0.1 \
  --max-vx 0.3 \
  --max-vy 0.15 \
  --max-yaw-rate 0.5 \
  "$@"
