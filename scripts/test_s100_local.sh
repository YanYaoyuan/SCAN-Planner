#!/usr/bin/env bash

# Local entry point for the same S100 cross build used by GitHub Actions.
# A dedicated rootless Docker daemon stores all image data under /data and
# does not touch the system daemon, its images, or its running containers.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
INTERFACES_SOURCE=${OMNI_ROBOT_INTERFACES_SOURCE:-${ROOT_DIR}/../omni_robot_interfaces}
LOCAL_BASE=${S100_LOCAL_BASE:-/data/scan-planner-s100-local}
DOCKER_BASE=${S100_DOCKER_BASE:-/data/docker-s100}
S100_IMAGE=${S100_IMAGE:-pc_tros_ubuntu22.04:v1.0.0}
S100_IMAGE_URL=${S100_IMAGE_URL:-http://archive.d-robotics.cc/TogetheROS/cross_compile_docker/pc_tros_ubuntu22.04_v1.0.0.tar.gz}
S100_IMAGE_SHA256=${S100_IMAGE_SHA256:-ec065c268782373311421f0bb7f2212bfec6584a7070a9737468055a6bce2c88}
CHECK_ONLY=0

if [[ "${1:-}" == "--check-only" ]]; then
  CHECK_ONLY=1
elif [[ $# -gt 0 ]]; then
  printf 'Usage: %s [--check-only]\n' "$0" >&2
  exit 2
fi

log() {
  printf '[s100-local] %s\n' "$*"
}

die() {
  printf '[s100-local] ERROR: %s\n' "$*" >&2
  exit 1
}

[[ -f "${INTERFACES_SOURCE}/package.xml" ]] || \
  die "invalid OMNI_ROBOT_INTERFACES_SOURCE: ${INTERFACES_SOURCE}"

[[ -d /data ]] || die '/data does not exist'
[[ -w /data ]] || die '/data is not writable by the current user'

for command_name in curl docker dockerd-rootless.sh flock rootlesskit sha256sum slirp4netns; do
  command -v "${command_name}" >/dev/null 2>&1 || die "missing command: ${command_name}"
done

available_kb=$(df -Pk /data | awk 'NR == 2 {print $4}')
minimum_kb=$((50 * 1024 * 1024))
if (( available_kb < minimum_kb )); then
  die '/data must have at least 50 GiB available for a clean local build'
fi

mkdir -p "${LOCAL_BASE}"
exec 9> "${LOCAL_BASE}/.build.lock"
if ! flock -n 9; then
  die "another S100 local build is already using ${LOCAL_BASE}; wait for it to finish instead of starting a second copy"
fi

DATA_ROOT="${DOCKER_BASE}/data"
EXEC_ROOT="${DOCKER_BASE}/exec"
RUNTIME_ROOT="${DOCKER_BASE}/run"
CONFIG_ROOT="${DOCKER_BASE}/config"
DAEMON_LOG="${DOCKER_BASE}/dockerd.log"
DAEMON_PID="${DOCKER_BASE}/dockerd.pid"
DOCKER_SOCKET="${RUNTIME_ROOT}/docker.sock"

mkdir -p \
  "${DATA_ROOT}" \
  "${EXEC_ROOT}" \
  "${RUNTIME_ROOT}" \
  "${CONFIG_ROOT}"
chmod 700 "${RUNTIME_ROOT}"
chmod 700 "${CONFIG_ROOT}"

# Do not inherit ~/.docker/config.json. It may use the `pass` credential
# helper, which fails when the host password store has not been initialized.
# This dedicated config is private to the S100 rootless daemon.
export DOCKER_CONFIG="${CONFIG_ROOT}"
if [[ ! -f "${DOCKER_CONFIG}/config.json" ]]; then
  printf '{}\n' > "${DOCKER_CONFIG}/config.json"
  chmod 600 "${DOCKER_CONFIG}/config.json"
fi

export XDG_RUNTIME_DIR="${RUNTIME_ROOT}"
export DOCKERD_ROOTLESS_ROOTLESSKIT_STATE_DIR="${RUNTIME_ROOT}/rootlesskit"
# Allow the slirp4netns host gateway to reach services bound to host loopback.
export DOCKERD_ROOTLESS_ROOTLESSKIT_NET="${S100_ROOTLESS_NET:-slirp4netns}"
export DOCKERD_ROOTLESS_ROOTLESSKIT_DISABLE_HOST_LOOPBACK=false
export DOCKER_HOST="unix://${DOCKER_SOCKET}"

# A loopback proxy on the host is reachable as 10.0.2.2 from slirp4netns.
# Keep the host proxy unchanged for git, while using translated values for the
# rootless daemon and toolchain container.
daemon_http_proxy=${HTTP_PROXY:-}
daemon_https_proxy=${HTTPS_PROXY:-}
daemon_http_proxy=${daemon_http_proxy//127.0.0.1/10.0.2.2}
daemon_https_proxy=${daemon_https_proxy//127.0.0.1/10.0.2.2}
export S100_CONTAINER_HTTP_PROXY="${daemon_http_proxy}"
export S100_CONTAINER_HTTPS_PROXY="${daemon_https_proxy}"

if ! docker info >/dev/null 2>&1; then
  log "starting isolated rootless Docker daemon at ${DOCKER_SOCKET}"

  nohup env \
    HTTP_PROXY="${daemon_http_proxy}" \
    HTTPS_PROXY="${daemon_https_proxy}" \
    dockerd-rootless.sh \
    --data-root "${DATA_ROOT}" \
    --exec-root "${EXEC_ROOT}" \
    --host "unix://${DOCKER_SOCKET}" \
    --pidfile "${DAEMON_PID}" \
    > "${DAEMON_LOG}" 2>&1 &

  launcher_pid=$!

  for _ in $(seq 1 60); do
    if docker info >/dev/null 2>&1; then
      break
    fi

    if ! kill -0 "${launcher_pid}" 2>/dev/null; then
      tail -n 80 "${DAEMON_LOG}" >&2 || true
      die 'rootless Docker daemon exited during startup'
    fi

    sleep 1
  done
fi

docker info >/dev/null 2>&1 || {
  tail -n 80 "${DAEMON_LOG}" >&2 || true
  die 'rootless Docker daemon did not become ready within 60 seconds'
}

log "Docker data root: $(docker info --format '{{.DockerRootDir}}')"
log "Docker client config: ${DOCKER_CONFIG}"
log "workspace: ${LOCAL_BASE}"
log "available /data space: $(df -h /data | awk 'NR == 2 {print $4}')"

if (( CHECK_ONLY == 1 )); then
  log 'local prerequisites and isolated Docker daemon are ready'
  exit 0
fi

if ! docker image inspect "${S100_IMAGE}" >/dev/null 2>&1; then
  image_dir="${DOCKER_BASE}/images"
  image_archive="${image_dir}/pc_tros_ubuntu22.04_v1.0.0.tar.gz"
  mkdir -p "${image_dir}"
  log "downloading official TROS cross-compilation image"
  curl --fail --location \
    --retry 5 --retry-all-errors --continue-at - \
    --output "${image_archive}" \
    "${S100_IMAGE_URL}"
  echo "${S100_IMAGE_SHA256}  ${image_archive}" | sha256sum --check
  docker load --input "${image_archive}"
fi

S100_SOURCE_ROOT="${ROOT_DIR}" \
OMNI_ROBOT_INTERFACES_SOURCE="${INTERFACES_SOURCE}" \
S100_WORK_ROOT="${LOCAL_BASE}/workspace" \
S100_IMAGE="${S100_IMAGE}" \
S100_CLEAN="${S100_CLEAN:-1}" \
  "${ROOT_DIR}/scripts/build_s100_cross.sh"

log "local S100 build succeeded"
log "install overlay: ${LOCAL_BASE}/workspace/cc_ws/tros_ws/install"
