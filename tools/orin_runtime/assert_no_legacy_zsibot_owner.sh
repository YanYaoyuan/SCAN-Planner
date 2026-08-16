#!/usr/bin/env bash
set -euo pipefail

# Product deployments have exactly one vendor-SDK owner: the unified Gateway,
# and no legacy command forwarder may remain active. Stopping generation of old
# CMake targets does not remove binaries already in a reused colcon install
# tree, so fail closed instead of silently shipping one.

CHECK_PROCESSES=1
if [[ "${1:-}" == "--artifacts-only" ]]; then
  CHECK_PROCESSES=0
  shift
fi

if [[ "${CHECK_PROCESSES}" == "1" ]]; then
  if ! command -v pgrep >/dev/null 2>&1; then
    echo "ERROR: pgrep is unavailable; cannot prove that no legacy SDK owner is running." >&2
    exit 5
  fi

  set +e
  process_check_output="$({ pgrep -f \
    '(^|/)(zsibot_cmd_bridge|zsibot_cmd_udp_client|zsibot_sdk_proxy)([[:space:]]|$)'; } 2>&1)"
  process_check_status=$?
  set -e
  case "${process_check_status}" in
    0)
      echo "ERROR: a deprecated ZsiBot control process is still running." >&2
      echo "       PID(s): ${process_check_output}" >&2
      echo "       Stop zsibot_cmd_bridge/zsibot_cmd_udp_client/zsibot_sdk_proxy" >&2
      echo "       before product startup." >&2
      exit 4
      ;;
    1)
      ;;
    *)
      echo "ERROR: process enumeration failed; refusing product startup." >&2
      echo "       pgrep: ${process_check_output}" >&2
      exit 5
      ;;
  esac
fi

install_prefixes=("$@")
if [[ ${#install_prefixes[@]} -eq 0 ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  install_prefixes+=("${SCRIPT_DIR}/install" "${SCRIPT_DIR}/../../install")
  if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
    IFS=':' read -r -a ament_prefixes <<< "${AMENT_PREFIX_PATH}"
    install_prefixes+=("${ament_prefixes[@]}")
  fi
fi

for install_prefix in "${install_prefixes[@]}"; do
  [[ -n "${install_prefix}" ]] || continue
  for stale_target in \
    "${install_prefix}/lib/zsibot_cmd_bridge/zsibot_cmd_bridge" \
    "${install_prefix}/lib/zsibot_cmd_bridge/zsibot_sdk_proxy"
  do
    if [[ -e "${stale_target}" || -L "${stale_target}" ]]; then
      echo "ERROR: stale deprecated SDK-owner binary remains in an install tree:" >&2
      echo "       ${stale_target}" >&2
      echo "       Build/deploy into a clean product install prefix before startup." >&2
      exit 3
    fi
  done
done
