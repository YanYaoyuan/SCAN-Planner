#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: run_planner_with_bag_inputs.sh

Start SCAN-Planner in real-input offline mode. It consumes bag-replayed:
  /omni/tf_manager/body_odom_global
  /cloud_registered_global

Environment overrides:
  PCD_MAP_FILE       default: repo/scans.pcd
  GRID_FRAME_ID      default: omni_map
  CLOUD_TOPIC        default: /cloud_registered_global
  BODY_ODOM_TOPIC    default: /omni/tf_manager/body_odom_global
  CMD_VEL_TOPIC      default: /scan_planner/cmd_vel
  CONTROLLER_DRIVE_MODE default: pure_pursuit
  ROS_DOMAIN_ID      default: 73
EOF
  exit 0
fi
source "${SCRIPT_DIR}/common.sh"

PCD_MAP_FILE="${PCD_MAP_FILE:-${DEFAULT_PCD_MAP}}"
GRID_FRAME_ID="${GRID_FRAME_ID:-omni_map}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered_global}"
BODY_ODOM_TOPIC="${BODY_ODOM_TOPIC:-/omni/tf_manager/body_odom_global}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"
BODY_FRAME_ID="${BODY_FRAME_ID:-omni_base_link}"
SENSOR_FRAME_ID="${SENSOR_FRAME_ID:-omni_lidar_link}"
GRID_MAP_VIS_HEIGHT="${GRID_MAP_VIS_HEIGHT:-2.0}"
GOAL_FRAME_ID="${GOAL_FRAME_ID:-${GRID_FRAME_ID}}"

CONTROLLER_DRIVE_MODE="${CONTROLLER_DRIVE_MODE:-pure_pursuit}"
CONTROLLER_TRACKING_MODE="${CONTROLLER_TRACKING_MODE:-path_follow}"
CONTROLLER_MAX_VX="${CONTROLLER_MAX_VX:-0.3}"
CONTROLLER_MAX_VY="${CONTROLLER_MAX_VY:-0.15}"
CONTROLLER_MAX_VYAW="${CONTROLLER_MAX_VYAW:-0.5}"
CONTROLLER_LOOKAHEAD_DIST="${CONTROLLER_LOOKAHEAD_DIST:-0.45}"
CONTROLLER_YAW_LOOKAHEAD_DIST="${CONTROLLER_YAW_LOOKAHEAD_DIST:-0.55}"
CONTROLLER_PURE_PURSUIT_SPEED="${CONTROLLER_PURE_PURSUIT_SPEED:-0.20}"

require_path "${PCD_MAP_FILE}" "PCD map"

echo "Starting SCAN-Planner offline validation:"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  grid frame=${GRID_FRAME_ID}"
echo "  body odom=${BODY_ODOM_TOPIC}"
echo "  cloud=${CLOUD_TOPIC}"
echo "  pcd map=${PCD_MAP_FILE}"
echo "  cmd_vel=${CMD_VEL_TOPIC}"
echo "  controller=${CONTROLLER_TRACKING_MODE}/${CONTROLLER_DRIVE_MODE}, vx=${CONTROLLER_PURE_PURSUIT_SPEED}, lookahead=${CONTROLLER_LOOKAHEAD_DIST}"

exec ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  controller_tracking_mode:="${CONTROLLER_TRACKING_MODE}" \
  controller_drive_mode:="${CONTROLLER_DRIVE_MODE}" \
  controller_max_vx:="${CONTROLLER_MAX_VX}" \
  controller_max_vy:="${CONTROLLER_MAX_VY}" \
  controller_max_vyaw:="${CONTROLLER_MAX_VYAW}" \
  controller_lookahead_dist:="${CONTROLLER_LOOKAHEAD_DIST}" \
  controller_yaw_lookahead_dist:="${CONTROLLER_YAW_LOOKAHEAD_DIST}" \
  controller_pure_pursuit_speed:="${CONTROLLER_PURE_PURSUIT_SPEED}" \
  use_gpu:=false \
  publish_robot_description:=false \
  use_zsibot_bridge:=false \
  use_zsibot_udp_client:=false \
  body_frame_id:="${BODY_FRAME_ID}" \
  sensor_frame_id:="${SENSOR_FRAME_ID}" \
  require_tf_ready:=false \
  real_body_pose_topic:="${BODY_ODOM_TOPIC}" \
  real_cloud_topic:="${CLOUD_TOPIC}" \
  real_cmd_vel_topic:="${CMD_VEL_TOPIC}" \
  real_grid_frame_id:="${GRID_FRAME_ID}" \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:="${GOAL_FRAME_ID}" \
  grid_map_vis_height:="${GRID_MAP_VIS_HEIGHT}" \
  use_pcd_map:=true \
  pcd_map_file:="${PCD_MAP_FILE}"
