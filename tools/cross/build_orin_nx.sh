#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SYSROOT="${ORIN_NX_SYSROOT:-/home/user/jetson/orin-nx/sysroot}"
BUILD_LEGACY_ZSIBOT_UDP_CLIENT="${BUILD_LEGACY_ZSIBOT_UDP_CLIENT:-0}"
BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS="${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS:-0}"

if [[ -n "${BUILD_DEPRECATED_ZSIBOT_BRIDGE:-}" ]]; then
  echo "ERROR: BUILD_DEPRECATED_ZSIBOT_BRIDGE was split into two explicit flags:" >&2
  echo "       BUILD_LEGACY_ZSIBOT_UDP_CLIENT and BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS." >&2
  exit 2
fi

for flag_value in \
  "BUILD_LEGACY_ZSIBOT_UDP_CLIENT=${BUILD_LEGACY_ZSIBOT_UDP_CLIENT}" \
  "BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS=${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS}"
do
  if [[ "${flag_value#*=}" != "0" && "${flag_value#*=}" != "1" ]]; then
    echo "${flag_value%%=*} must be 0 or 1" >&2
    exit 2
  fi
done

PACKAGES_UP_TO=(scan_planner)
if [[ "${BUILD_LEGACY_ZSIBOT_UDP_CLIENT}" == "1" ||
      "${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS}" == "1" ]]; then
  PACKAGES_UP_TO+=(zsibot_cmd_bridge)
fi
if [[ "${BUILD_LEGACY_ZSIBOT_UDP_CLIENT}" == "1" ]]; then
  echo "WARNING: including the deprecated UDP compatibility client." >&2
fi
if [[ "${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS}" == "1" ]]; then
  echo "WARNING: building deprecated vendor SDK owners for an isolated bench only." >&2
fi

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
  --packages-up-to "${PACKAGES_UP_TO[@]}" \
  --cmake-clean-cache \
  --cmake-args \
    "-DCMAKE_TOOLCHAIN_FILE=${WORKSPACE}/tools/cross/orin_nx_toolchain.cmake" \
    "-DORIN_NX_SYSROOT=${SYSROOT}" \
    "-DZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS=${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF

if [[ "${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS}" == "0" ]]; then
  "${WORKSPACE}/tools/orin_runtime/assert_no_legacy_zsibot_owner.sh" \
    --artifacts-only \
    "${WORKSPACE}/install-orin-sysroot"
fi
