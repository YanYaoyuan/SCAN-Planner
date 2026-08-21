#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MATRIX_ROOT="${MATRIX_ROOT:-/data/robot/sim/matrix}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"
BODY_ODOM_TOPIC="${BODY_ODOM_TOPIC:-/omni/tf_manager/body_odom_global}"
TF_READY_TOPIC="${TF_READY_TOPIC:-/omni/tf_manager/ready}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered_global}"
GRID_FRAME_ID="${GRID_FRAME_ID:-omni_map}"
BODY_FRAME_ID="${BODY_FRAME_ID:-omni_base_link}"
SENSOR_FRAME_ID="${SENSOR_FRAME_ID:-omni_lidar_link}"
GOAL_TOPIC="${GOAL_TOPIC:-/move_base_simple/goal}"
NAVI_MODE="${NAVI_MODE:-3}"
CONTROL_MODE="${CONTROL_MODE:-none}"
START_ZENOH="${START_ZENOH:-1}"
RVIZ="${RVIZ:-0}"

CONTROLLER_DRIVE_MODE="${CONTROLLER_DRIVE_MODE:-pure_pursuit}"
CONTROLLER_TRACKING_MODE="${CONTROLLER_TRACKING_MODE:-path_follow}"
CONTROLLER_MAX_VX="${CONTROLLER_MAX_VX:-0.25}"
CONTROLLER_MAX_VY="${CONTROLLER_MAX_VY:-0.10}"
CONTROLLER_MAX_VYAW="${CONTROLLER_MAX_VYAW:-0.5}"
CONTROLLER_LOOKAHEAD_DIST="${CONTROLLER_LOOKAHEAD_DIST:-0.45}"
CONTROLLER_YAW_LOOKAHEAD_DIST="${CONTROLLER_YAW_LOOKAHEAD_DIST:-0.55}"
CONTROLLER_PURE_PURSUIT_SPEED="${CONTROLLER_PURE_PURSUIT_SPEED:-0.20}"

MATRIX_SDK_CONFIG="${MATRIX_SDK_CONFIG:-$MATRIX_ROOT/src/robot_mc/build/export/config/sdk_config.yaml}"
BRIDGE_CONFIG_TEMPLATE="${BRIDGE_CONFIG_TEMPLATE:-$SCRIPT_DIR/src/zsibot_cmd_bridge/config/matrix_sim_bridge.yaml}"
BRIDGE_CONFIG_FILE="${BRIDGE_CONFIG_FILE:-}"

usage() {
  cat <<EOF
Usage:
  ./run_matrix_planner.sh [options] [extra ros2 launch args]

Options:
  --no-control            Start planner only. This is the default.
  --control               Explicitly opt in to the deprecated ZsiBot SDK bridge.
  --navi-mode MODE        1 for direct goal, 2 for waypoints, 3 for generated global path. Default: ${NAVI_MODE}
  --goal-topic TOPIC      Goal topic. Default: ${GOAL_TOPIC}
  --cloud-topic TOPIC     World cloud topic from SLAM. Default: ${CLOUD_TOPIC}
  --odom-topic TOPIC      Manager body odom topic. Default: ${BODY_ODOM_TOPIC}
  --cmd-topic TOPIC       Planner cmd_vel topic. Default: ${CMD_VEL_TOPIC}
  --bridge-config PATH    Use an explicit zsibot_cmd_bridge YAML.
  --rviz                  Also start SCAN-Planner RViz.
  --no-zenoh              Do not auto-start rmw_zenohd.
  -h, --help              Show this help.

Environment:
  ROS_DOMAIN_ID defaults to 89.
  RMW_IMPLEMENTATION defaults to rmw_zenoh_cpp.
  MATRIX_ROOT defaults to ${MATRIX_ROOT}.
  --control also requires building zsibot_cmd_bridge with
  -DZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS=ON.
EOF
}

EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-control)
      CONTROL_MODE="none"
      shift
      ;;
    --control)
      CONTROL_MODE="bridge"
      shift
      ;;
    --navi-mode)
      NAVI_MODE="${2:?missing navigation mode}"
      shift 2
      ;;
    --goal-topic)
      GOAL_TOPIC="${2:?missing goal topic}"
      shift 2
      ;;
    --cloud-topic)
      CLOUD_TOPIC="${2:?missing cloud topic}"
      shift 2
      ;;
    --odom-topic)
      BODY_ODOM_TOPIC="${2:?missing odom topic}"
      shift 2
      ;;
    --cmd-topic)
      CMD_VEL_TOPIC="${2:?missing cmd topic}"
      shift 2
      ;;
    --bridge-config)
      BRIDGE_CONFIG_FILE="${2:?missing bridge config path}"
      shift 2
      ;;
    --rviz)
      RVIZ=1
      shift
      ;;
    --no-zenoh)
      START_ZENOH=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "$CONTROL_MODE" != "bridge" && "$CONTROL_MODE" != "none" ]]; then
  echo "[ERROR] CONTROL_MODE must be 'bridge' or 'none'" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog}"
mkdir -p "$ROS_LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

cleanup_pids=()
RUNTIME_BRIDGE_CONFIG=""
cleanup() {
  for pid in "${cleanup_pids[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  if [[ -n "$RUNTIME_BRIDGE_CONFIG" && -f "$RUNTIME_BRIDGE_CONFIG" ]]; then
    rm -f "$RUNTIME_BRIDGE_CONFIG"
  fi
}
trap cleanup EXIT INT TERM

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-30}"
  local start
  start=$(date +%s)
  while ! topic_exists "$topic"; do
    if (( $(date +%s) - start >= timeout )); then
      echo "[ERROR] timed out waiting for topic: $topic" >&2
      echo "        Start MATRiX and omni_slam relocalization first, with ROS_DOMAIN_ID=$ROS_DOMAIN_ID and RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION." >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_message() {
  local topic="$1"
  local timeout="${2:-30}"
  local start
  start=$(date +%s)
  while ! timeout 2 ros2 topic echo "$topic" --once >/dev/null 2>&1; do
    if (( $(date +%s) - start >= timeout )); then
      echo "[ERROR] timed out waiting for data on topic: $topic" >&2
      echo "        The topic exists, but no messages were received. Check the upstream publisher and QoS." >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_ready() {
  local timeout_seconds="${1:-45}"
  local start
  start=$(date +%s)
  while ! timeout 3 ros2 topic echo "$TF_READY_TOPIC" std_msgs/msg/Bool \
      --qos-durability transient_local --once 2>/dev/null | grep -Fq "data: true"; do
    if (( $(date +%s) - start >= timeout_seconds )); then
      echo "[ERROR] timed out waiting for $TF_READY_TOPIC=data:true" >&2
      echo "        Check omni_tf_manager diagnostics and SLAM localization state." >&2
      return 1
    fi
    sleep 1
  done
}

read_yaml_value() {
  local key="$1"
  local file="$2"
  awk -F: -v key="$key" '$1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
    value=$2
    sub(/^[[:space:]]*/, "", value)
    sub(/[[:space:]]*#.*/, "", value)
    gsub(/"/, "", value)
    print value
    exit
  }' "$file"
}

write_bridge_config() {
  if [[ -n "$BRIDGE_CONFIG_FILE" ]]; then
    printf '%s\n' "$BRIDGE_CONFIG_FILE"
    return
  fi
  if [[ ! -f "$BRIDGE_CONFIG_TEMPLATE" ]]; then
    echo "[ERROR] bridge config template not found: $BRIDGE_CONFIG_TEMPLATE" >&2
    exit 1
  fi

  local local_ip="127.0.0.1"
  local dog_ip="127.0.0.1"
  local local_port="43988"
  if [[ -f "$MATRIX_SDK_CONFIG" ]]; then
    local_ip="$(read_yaml_value target_ip "$MATRIX_SDK_CONFIG" || true)"
    local_port="$(read_yaml_value target_port "$MATRIX_SDK_CONFIG" || true)"
    local_ip="${local_ip:-127.0.0.1}"
    local_port="${local_port:-43988}"
  else
    echo "[WARN] MATRiX sdk_config.yaml not found; using bridge local_port=${local_port}" >&2
  fi

  RUNTIME_BRIDGE_CONFIG="$(mktemp /tmp/matrix_zsibot_bridge_XXXXXX.yaml)"
  python3 "$SCRIPT_DIR/tools/matrix_configure_zsibot_bridge.py" \
    --input "$BRIDGE_CONFIG_TEMPLATE" \
    --output "$RUNTIME_BRIDGE_CONFIG" \
    --local-ip "$local_ip" \
    --dog-ip "$dog_ip" \
    --local-port "$local_port"
  printf '%s\n' "$RUNTIME_BRIDGE_CONFIG"
}

if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" && "$START_ZENOH" == "1" ]]; then
  if ! pgrep -f "rmw_zenoh_cpp.*rmw_zenohd" >/dev/null 2>&1; then
    echo "[INFO] starting rmw_zenohd"
    ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/matrix_scan_planner_zenohd.log 2>&1 &
    cleanup_pids+=("$!")
    sleep 2
  fi
fi

echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[INFO] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[INFO] waiting for TF authority: $TF_READY_TOPIC"
wait_for_topic "$TF_READY_TOPIC" 45
wait_for_ready 45
echo "[INFO] waiting for manager body odom: $BODY_ODOM_TOPIC"
wait_for_topic "$BODY_ODOM_TOPIC" 45
echo "[INFO] waiting for first manager body odom sample: $BODY_ODOM_TOPIC"
wait_for_message "$BODY_ODOM_TOPIC" 45
echo "[INFO] waiting for SLAM cloud: $CLOUD_TOPIC"
wait_for_topic "$CLOUD_TOPIC" 45
echo "[INFO] waiting for first SLAM cloud sample: $CLOUD_TOPIC"
wait_for_message "$CLOUD_TOPIC" 45

if [[ "$CONTROL_MODE" == "bridge" ]]; then
  echo "[WARN] deprecated ZsiBot SDK bridge explicitly enabled; stop any unified robot bridge" >&2
  BRIDGE_CONFIG_FILE="$(write_bridge_config)"
  echo "[INFO] control bridge enabled: $BRIDGE_CONFIG_FILE"
else
  echo "[INFO] control bridge disabled; planner will only publish $CMD_VEL_TOPIC"
fi

USE_BRIDGE="false"
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT="false"
ZSIBOT_LAUNCH_ARGS=()
if [[ "$CONTROL_MODE" == "bridge" ]]; then
  USE_BRIDGE="true"
  ENABLE_DEPRECATED_ZSIBOT_TRANSPORT="true"
  ZSIBOT_LAUNCH_ARGS+=("zsibot_config_file:=$BRIDGE_CONFIG_FILE")
fi

USE_GLOBAL_PATH_PUBLISHER="false"
if [[ "$NAVI_MODE" == "3" ]]; then
  USE_GLOBAL_PATH_PUBLISHER="true"
fi

if [[ "$RVIZ" == "1" ]]; then
  ros2 launch scan_planner rviz.launch.py >/tmp/matrix_scan_planner_rviz.log 2>&1 &
  cleanup_pids+=("$!")
fi

exec ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:="$NAVI_MODE" \
  use_global_path_publisher:="$USE_GLOBAL_PATH_PUBLISHER" \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  controller_tracking_mode:="$CONTROLLER_TRACKING_MODE" \
  controller_drive_mode:="$CONTROLLER_DRIVE_MODE" \
  controller_max_vx:="$CONTROLLER_MAX_VX" \
  controller_max_vy:="$CONTROLLER_MAX_VY" \
  controller_max_vyaw:="$CONTROLLER_MAX_VYAW" \
  controller_lookahead_dist:="$CONTROLLER_LOOKAHEAD_DIST" \
  controller_yaw_lookahead_dist:="$CONTROLLER_YAW_LOOKAHEAD_DIST" \
  controller_pure_pursuit_speed:="$CONTROLLER_PURE_PURSUIT_SPEED" \
  use_gpu:=false \
  publish_robot_description:=false \
  enable_deprecated_zsibot_transport:="$ENABLE_DEPRECATED_ZSIBOT_TRANSPORT" \
  use_zsibot_bridge:="$USE_BRIDGE" \
  body_frame_id:="$BODY_FRAME_ID" \
  sensor_frame_id:="$SENSOR_FRAME_ID" \
  require_tf_ready:=true \
  tf_ready_topic:="$TF_READY_TOPIC" \
  real_cmd_vel_topic:="$CMD_VEL_TOPIC" \
  real_body_pose_topic:="$BODY_ODOM_TOPIC" \
  real_cloud_topic:="$CLOUD_TOPIC" \
  real_grid_frame_id:="$GRID_FRAME_ID" \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:="$GRID_FRAME_ID" \
  goal_topic:="$GOAL_TOPIC" \
  "${ZSIBOT_LAUNCH_ARGS[@]}" \
  "${EXTRA_ARGS[@]}"
