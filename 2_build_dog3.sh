#!/usr/bin/env bash
set -euo pipefail

readonly SCAN_PLANNER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DOG3_DOCKER_IMAGE="scan-planner-dog3:humble-arm64"
readonly DOG3_DOCKERFILE="$SCAN_PLANNER_DIR/deploy/dog3/Dockerfile"

if [[ ! -f /app/opt/ros/humble/setup.bash ]]; then
  printf '%s\n' "错误：本脚本只用于 dog3；未找到 /app/opt/ros/humble/setup.bash。" >&2
  exit 2
fi
if [[ ! -f /app/idl_msgs/local_setup.bash ]]; then
  printf '%s\n' "错误：缺少 dog3 自定义消息环境 /app/idl_msgs/local_setup.bash。" >&2
  exit 2
fi
readonly DOG3_ARCH="$(uname -m)"
case "$DOG3_ARCH" in
  aarch64|arm64) ;;
  *)
    printf '错误：dog3 构建必须在 ARM64 主机执行，当前架构为 %s。\n' \
      "$DOG3_ARCH" >&2
    exit 2
    ;;
esac
if ! command -v docker >/dev/null 2>&1; then
  printf '%s\n' "错误：dog3 上未找到 docker。" >&2
  exit 2
fi
if [[ ! -f "$DOG3_DOCKERFILE" ]]; then
  printf '错误：缺少 dog3 Dockerfile：%s。\n' "$DOG3_DOCKERFILE" >&2
  exit 2
fi

for package_file in \
  "$SCAN_PLANNER_DIR/src/dog3_vendor/pcl_msgs/package.xml" \
  "$SCAN_PLANNER_DIR/src/dog3_vendor/perception_pcl/pcl_conversions/package.xml" \
  "$SCAN_PLANNER_DIR/src/dog3_vendor/vision_opencv/cv_bridge/package.xml"; do
  if [[ ! -f "$package_file" ]]; then
    printf '错误：缺少 dog3 vendor 源码 %s。\n' "$package_file" >&2
    exit 2
  fi
done

printf '构建 dog3 ARM64 依赖镜像：%s\n' "$DOG3_DOCKER_IMAGE"
docker build \
  --network host \
  --tag "$DOG3_DOCKER_IMAGE" \
  --file "$DOG3_DOCKERFILE" \
  "$SCAN_PLANNER_DIR/deploy/dog3"

printf '%s\n' "在 dog3 容器中构建 vendor + Omni planner 九包闭包"
docker run --rm \
  --network host \
  --ipc host \
  -e SCAN_PLANNER_IN_DOCKER=1 \
  -e WHEELDOG_PLATFORM=dog \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -v /app:/app:ro \
  -v "$SCAN_PLANNER_DIR:$SCAN_PLANNER_DIR" \
  -w "$SCAN_PLANNER_DIR" \
  "$DOG3_DOCKER_IMAGE" \
  bash -lc '
    set -euo pipefail
    set +u
    source /app/opt/ros/humble/setup.bash
    source /app/idl_msgs/local_setup.bash
    set -u
    python3 -m pip check
    colcon --log-base log-dog3 build \
      --build-base build-dog3 \
      --install-base install-dog3 \
      --merge-install \
      --parallel-workers 2 \
      --base-paths \
        src/planner \
        src/dog3_vendor/pcl_msgs \
        src/dog3_vendor/perception_pcl/pcl_conversions \
        src/dog3_vendor/vision_opencv/cv_bridge \
      --packages-select \
        pcl_msgs \
        pcl_conversions \
        cv_bridge \
        scan_planner_msgs \
        plan_env \
        path_searching \
        bspline_opt \
        traj_utils \
        scan_planner \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  '

printf '%s\n' \
  "dog3 构建完成：$SCAN_PLANNER_DIR/install-dog3" \
  "检查：./1_run_scanplanner.sh --check" \
  "启动：./1_run_scanplanner.sh"
