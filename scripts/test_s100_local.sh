#!/usr/bin/env bash

# Local entry point for the same S100 cross build used by GitHub Actions.
# A dedicated rootless Docker daemon stores all image data under /data and
# does not touch the system daemon, its images, or its running containers.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
INTERFACES_SOURCE=${OMNI_ROBOT_INTERFACES_SOURCE:-${ROOT_DIR}/../omni_robot_interfaces}
LOCAL_BASE=${S100_LOCAL_BASE:-/data/scan-planner-s100-local}
DOCKER_BASE=${S100_DOCKER_BASE:-/data/docker-s100}
IMAGE_CACHE_DIR=${S100_IMAGE_CACHE_DIR:-/data/omni-s100-cache/images}
S100_IMAGE=${S100_IMAGE:-pc_tros_ubuntu22.04:v1.0.0}
S100_IMAGE_URL=${S100_IMAGE_URL:-http://archive.d-robotics.cc/TogetheROS/cross_compile_docker/pc_tros_ubuntu22.04_v1.0.0.tar.gz}
S100_IMAGE_SHA256=${S100_IMAGE_SHA256:-ec065c268782373311421f0bb7f2212bfec6584a7070a9737468055a6bce2c88}
S100_IMAGE_ARCHIVE=${S100_IMAGE_ARCHIVE:-${IMAGE_CACHE_DIR}/pc_tros_ubuntu22.04_v1.0.0.tar.gz}
CHECK_ONLY=0
launcher_pid=''

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

mkdir -p \
  "${LOCAL_BASE}" \
  "${DOCKER_BASE}" \
  "${IMAGE_CACHE_DIR}" \
  "$(dirname "${S100_IMAGE_ARCHIVE}")"
exec 9> "${LOCAL_BASE}/.build.lock"
if ! flock -n 9; then
  die "another S100 local build is already using ${LOCAL_BASE}; wait for it to finish instead of starting a second copy"
fi

# SCAN-Planner and omni_slam share the same dedicated daemon. Serialize access
# to it as well as to this repository's workspace.
exec 8> "${DOCKER_BASE}/.build.lock"
if ! flock -n 8; then
  die "another S100 local build is already using Docker state at ${DOCKER_BASE}"
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

stop_docker_daemon() {
  local daemon_pid=''
  local daemon_cmdline=''
  local -a container_ids=()

  if docker info >/dev/null 2>&1; then
    mapfile -t container_ids < <(docker ps -q)
    if (( ${#container_ids[@]} > 0 )); then
      log "stopping ${#container_ids[@]} S100 build container(s)"
      docker stop --time 10 "${container_ids[@]}" >/dev/null || true
    fi
  fi

  if [[ -s "${DAEMON_PID}" ]]; then
    read -r daemon_pid < "${DAEMON_PID}" || true
  fi
  if [[ "${daemon_pid}" =~ ^[0-9]+$ ]] && [[ -r "/proc/${daemon_pid}/cmdline" ]]; then
    daemon_cmdline=$(tr '\0' ' ' < "/proc/${daemon_pid}/cmdline")
    if [[ "${daemon_cmdline}" == *"${DOCKER_SOCKET}"* ]]; then
      log "stopping isolated rootless Docker daemon (pid ${daemon_pid})"
      kill -TERM "${daemon_pid}" 2>/dev/null || true
      for _ in $(seq 1 150); do
        kill -0 "${daemon_pid}" 2>/dev/null || break
        sleep 0.1
      done
    fi
  fi

  if [[ -n "${launcher_pid}" ]] && kill -0 "${launcher_pid}" 2>/dev/null; then
    kill -TERM "${launcher_pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local exit_status=$?
  trap - EXIT
  set +e
  stop_docker_daemon
  if docker info >/dev/null 2>&1; then
    log "ERROR: isolated Docker daemon is still running at ${DOCKER_SOCKET}"
    (( exit_status == 0 )) && exit_status=1
  fi
  flock -u 8
  flock -u 9
  exec 8>&-
  exec 9>&-
  exit "${exit_status}"
}

trap cleanup EXIT

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

  env \
    HTTP_PROXY="${daemon_http_proxy}" \
    HTTPS_PROXY="${daemon_https_proxy}" \
    dockerd-rootless.sh \
    --data-root "${DATA_ROOT}" \
    --exec-root "${EXEC_ROOT}" \
    --host "unix://${DOCKER_SOCKET}" \
    --pidfile "${DAEMON_PID}" \
    > "${DAEMON_LOG}" 2>&1 8>&- 9>&- &

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
log "image cache: ${S100_IMAGE_ARCHIVE}"
log "available /data space: $(df -h /data | awk 'NR == 2 {print $4}')"

if (( CHECK_ONLY == 1 )); then
  log 'local prerequisites and isolated Docker daemon are ready'
  exit 0
fi

if ! docker image inspect "${S100_IMAGE}" >/dev/null 2>&1; then
  if [[ ! -f "${S100_IMAGE_ARCHIVE}" ]]; then
    image_partial="${S100_IMAGE_ARCHIVE}.part"
    log "downloading official TROS cross-compilation image to ${S100_IMAGE_ARCHIVE}"
    curl --fail --location \
      --retry 5 --retry-all-errors --continue-at - \
      --output "${image_partial}" \
      "${S100_IMAGE_URL}"
    echo "${S100_IMAGE_SHA256}  ${image_partial}" | sha256sum --check
    mv "${image_partial}" "${S100_IMAGE_ARCHIVE}"
  fi
  echo "${S100_IMAGE_SHA256}  ${S100_IMAGE_ARCHIVE}" | sha256sum --check
  docker load --input "${S100_IMAGE_ARCHIVE}"
fi

S100_SOURCE_ROOT="${ROOT_DIR}" \
OMNI_ROBOT_INTERFACES_SOURCE="${INTERFACES_SOURCE}" \
S100_WORK_ROOT="${LOCAL_BASE}/workspace" \
S100_IMAGE="${S100_IMAGE}" \
S100_CLEAN="${S100_CLEAN:-1}" \
  "${ROOT_DIR}/scripts/build_s100_cross.sh"

log "local S100 build succeeded"
log "install overlay: ${LOCAL_BASE}/workspace/cc_ws/tros_ws/install"
log "deployable runtime: ${LOCAL_BASE}/workspace/cc_ws/tros_ws/runtime"
