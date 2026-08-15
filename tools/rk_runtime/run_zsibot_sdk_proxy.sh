#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${ENABLE_DEPRECATED_ZSIBOT_TRANSPORT:-0}" != "1" ]]; then
  echo "ERROR: zsibot_sdk_proxy is deprecated and disabled by default." >&2
  echo "       Prefer the unified robot bridge. For an isolated migration test, run:" >&2
  echo "       ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 $0" >&2
  exit 2
fi

export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"

exec "${SCRIPT_DIR}/zsibot_sdk_proxy" \
  --enable-deprecated-transport \
  --listen-ip 0.0.0.0 \
  --listen-port 44000 \
  --sdk-local-ip 127.0.0.1 \
  --sdk-local-port 43988 \
  --sdk-dog-ip 127.0.0.1 \
  --max-vx 0.3 \
  --max-vy 0.15 \
  --max-yaw-rate 0.5 \
  "$@"
