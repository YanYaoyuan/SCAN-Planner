#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <tool> [args...]" >&2
  exit 2
fi

SYSROOT="${ORIN_NX_SYSROOT:-/home/user/jetson/orin-nx/sysroot}"
TOOL="$1"
shift

export QEMU_LD_PREFIX="${SYSROOT}"
export PATH="${SYSROOT}/usr/bin:${PATH}"

exec /usr/bin/qemu-aarch64-static -L "${SYSROOT}" "${SYSROOT}/usr/bin/${TOOL}" "$@"
