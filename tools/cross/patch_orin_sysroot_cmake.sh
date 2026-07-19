#!/usr/bin/env bash
set -euo pipefail

SYSROOT="${1:-/home/user/jetson/orin-nx/sysroot}"
TARGET="/usr/lib/aarch64-linux-gnu/libpython3.10.so"
REPLACEMENT="python3.10"

if [[ ! -d "${SYSROOT}" ]]; then
  echo "sysroot not found: ${SYSROOT}" >&2
  exit 1
fi

if [[ ! -e "${SYSROOT}${TARGET}" ]]; then
  echo "target Python library not found in sysroot: ${SYSROOT}${TARGET}" >&2
  exit 1
fi

changed=0
while IFS= read -r -d '' file; do
  if grep -q "${TARGET}" "${file}" || grep -q "${SYSROOT}${TARGET}" "${file}"; then
    cp -n "${file}" "${file}.orin-cross-bak"
    perl -0pi \
      -e "s#\Q${TARGET}\E#${REPLACEMENT}#g" \
      -e "s#\Q${SYSROOT}${TARGET}\E#${REPLACEMENT}#g" \
      "${file}"
    echo "patched ${file}"
    changed=$((changed + 1))
  fi
done < <(find "${SYSROOT}/opt/ros/humble" "${SYSROOT}/opt/robot" -type f -name '*.cmake' -print0 2>/dev/null)

echo "patched ${changed} CMake export file(s)"
