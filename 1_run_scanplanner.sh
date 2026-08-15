#!/usr/bin/env bash
set -eo pipefail

readonly SCAN_PLANNER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DOG3_INSTALL="$SCAN_PLANNER_DIR/install-dog3"
readonly DOG3_LAUNCH="dog3_mode3.launch.py"
readonly DOG3_DOCKER_IMAGE="scan-planner-dog3:humble-arm64"

usage() {
  printf '%s\n' \
    "用法：" \
    "  ./1_run_scanplanner.sh" \
    "  ./1_run_scanplanner.sh --check" \
    "  ./1_run_scanplanner.sh --help" \
    "" \
    "固定启动 Omni SCAN-Planner dog3 模式3，只发布局部路径，不启动控制器。"
}

case "${1:-}" in
  "") CHECK_ONLY=0 ;;
  --check) CHECK_ONLY=1 ;;
  -h|--help) usage; exit 0 ;;
  *)
    printf '错误：不支持参数 %s；请修改 dog3.yaml。\n' "$1" >&2
    usage >&2
    exit 2
    ;;
esac

if (($# > 1)); then
  printf '%s\n' "错误：不接受额外启动参数；请修改 dog3.yaml。" >&2
  exit 2
fi

# dog3 主机根分区较小，PCL/OpenCV 依赖由镜像承载。直接挂载当前仓库，
# 不再要求它必须位于 /userdata/1_heng，也不依赖兄弟目录 0_source_env。
if [[ "${SCAN_PLANNER_IN_DOCKER:-0}" != 1 \
  && -f /app/opt/ros/humble/setup.bash ]]; then
  if ! command -v docker >/dev/null 2>&1; then
    printf '%s\n' "错误：检测到 dog3 环境，但未找到 docker。" >&2
    exit 2
  fi
  if [[ ! -S /var/run/docker.sock ]]; then
    printf '%s\n' "错误：检测到 dog3 环境，但 Docker daemon 不可用。" >&2
    exit 2
  fi
  if [[ ! -d /app_param ]]; then
    printf '%s\n' "错误：缺少 /app_param；dog3 Zenoh namespace 无法加载。" >&2
    exit 2
  fi
  if ! docker image inspect "$DOG3_DOCKER_IMAGE" >/dev/null 2>&1; then
    printf '错误：尚未构建镜像 %s，请先运行 ./2_build_dog3.sh。\n' \
      "$DOG3_DOCKER_IMAGE" >&2
    exit 2
  fi

  exec docker run --rm \
    --network host \
    --ipc host \
    --cap-add SYS_NICE \
    --ulimit memlock=-1:-1 \
    --ulimit rtprio=99:99 \
    --init \
    -e SCAN_PLANNER_IN_DOCKER=1 \
    -e WHEELDOG_PLATFORM=dog \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    -v /app:/app:ro \
    -v /app_param:/app_param:ro \
    -v "$SCAN_PLANNER_DIR:$SCAN_PLANNER_DIR" \
    -v /etc/localtime:/etc/localtime:ro \
    -w "$SCAN_PLANNER_DIR" \
    "$DOG3_DOCKER_IMAGE" \
    bash "$SCAN_PLANNER_DIR/1_run_scanplanner.sh" "$@"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "${SCAN_PLANNER_IN_DOCKER:-0}" == 1 || "${WHEELDOG_PLATFORM:-}" == dog ]]; then
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
fi

set +u
if [[ -f /app/opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /app/opt/ros/humble/setup.bash
  if [[ -f /app/idl_msgs/local_setup.bash ]]; then
    # shellcheck disable=SC1091
    source /app/idl_msgs/local_setup.bash
  fi
  if [[ -f /app/script/env.sh ]]; then
    # dog3 厂家脚本提供 Zenoh namespace/config 等运行时环境。
    # shellcheck disable=SC1091
    source /app/script/env.sh
  fi
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
elif ! command -v ros2 >/dev/null 2>&1; then
  printf '%s\n' "错误：未找到 ROS 2 Humble 环境。" >&2
  exit 2
fi

if [[ ! -f "$DOG3_INSTALL/setup.bash" ]]; then
  printf '错误：未找到 dog3 安装空间 %s，请先运行 ./2_build_dog3.sh。\n' \
    "$DOG3_INSTALL" >&2
  exit 2
fi
# shellcheck disable=SC1090
source "$DOG3_INSTALL/setup.bash"
set +u

if ! command -v ros2 >/dev/null 2>&1; then
  printf '%s\n' "错误：加载环境后仍未找到 ros2。" >&2
  exit 2
fi
if [[ "${SCAN_PLANNER_IN_DOCKER:-0}" == 1 \
  || "${WHEELDOG_PLATFORM:-}" == dog ]]; then
  if ! ros2 pkg prefix "${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" >/dev/null 2>&1; then
    printf '错误：dog3 ROS 环境缺少 RMW 包 %s。\n' \
      "${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" >&2
    exit 2
  fi
fi

PACKAGE_PREFIX="$(ros2 pkg prefix scan_planner)"
PACKAGE_SHARE="$(ros2 pkg prefix --share scan_planner)"
INSTALLED_LAUNCH="$PACKAGE_SHARE/launch/$DOG3_LAUNCH"
PARAMS_FILE="$PACKAGE_SHARE/config/dog3.yaml"

case "$PACKAGE_PREFIX" in
  "$DOG3_INSTALL"|"$DOG3_INSTALL"/*) ;;
  *)
    printf '错误：当前 scan_planner 不是本仓库 dog3 构建：%s\n' \
      "$PACKAGE_PREFIX" >&2
    exit 2
    ;;
esac

for required_file in "$INSTALLED_LAUNCH" "$PARAMS_FILE"; do
  if [[ ! -f "$required_file" ]]; then
    printf '错误：安装空间缺少 %s，请重新构建。\n' "$required_file" >&2
    exit 2
  fi
done

if [[ "$CHECK_ONLY" -eq 1 ]]; then
  printf '%s\n' \
    "dog3 运行环境检查通过" \
    "工程目录：$SCAN_PLANNER_DIR" \
    "安装目录：$PACKAGE_PREFIX" \
    "启动文件：$INSTALLED_LAUNCH" \
    "配置文件：$PARAMS_FILE" \
    "ROS_DOMAIN_ID：$ROS_DOMAIN_ID" \
    "RMW：${RMW_IMPLEMENTATION:-系统默认}" \
    "输入：/relocalizing/map_frame/odometry、/lidar_points、/map_frame/global_path" \
    "输出：/map_frame/local_path" \
    "下游要求：支持空 Path 撤销和 steady-clock 接收超时"
  exit 0
fi

printf '%s\n' \
  "启动 Omni SCAN-Planner dog3 模式3（planner-only）" \
  "输出：/map_frame/local_path；不会启动 controller 或发布 cmd_vel" \
  "安全要求：下游必须支持空 Path 撤销和路径接收超时；禁止直接使用旧 7_moveforward" \
  "按 Ctrl+C 停止"
exec ros2 launch scan_planner "$DOG3_LAUNCH"
