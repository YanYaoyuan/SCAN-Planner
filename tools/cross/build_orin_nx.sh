#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SYSROOT="${ORIN_NX_SYSROOT:-/home/user/jetson/orin-nx/sysroot}"

if [[ ! -d "${SYSROOT}" ]]; then
  echo "sysroot not found: ${SYSROOT}" >&2
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

cd "${WORKSPACE}"
tools/cross/patch_orin_sysroot_cmake.sh "${SYSROOT}"

colcon --log-base log-orin-sysroot build \
  --build-base build-orin-sysroot \
  --install-base install-orin-sysroot \
  --merge-install \
  --packages-up-to scan_planner zsibot_cmd_bridge \
  --cmake-clean-cache \
  --cmake-args \
    "-DCMAKE_TOOLCHAIN_FILE=${WORKSPACE}/tools/cross/orin_nx_toolchain.cmake" \
    "-DORIN_NX_SYSROOT=${SYSROOT}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
