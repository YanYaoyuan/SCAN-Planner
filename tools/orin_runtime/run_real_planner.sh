#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"
LIDAR_ODOM_TOPIC="${LIDAR_ODOM_TOPIC:-/state_estimation}"
BODY_ODOM_TOPIC="${BODY_ODOM_TOPIC:-/body_state_estimation}"
GRID_FRAME_ID="${GRID_FRAME_ID:-lio_odom}"
BODY_FRAME_ID="${BODY_FRAME_ID:-base_link}"
SENSOR_FRAME_ID="${SENSOR_FRAME_ID:-livox_frame}"
BODY_TO_SENSOR_X="${BODY_TO_SENSOR_X:-0.0}"
BODY_TO_SENSOR_Y="${BODY_TO_SENSOR_Y:-0.0}"
BODY_TO_SENSOR_Z="${BODY_TO_SENSOR_Z:-0.0}"
BODY_TO_SENSOR_ROLL="${BODY_TO_SENSOR_ROLL:-0.0}"
BODY_TO_SENSOR_PITCH="${BODY_TO_SENSOR_PITCH:-0.0}"
BODY_TO_SENSOR_YAW="${BODY_TO_SENSOR_YAW:-0.0}"

exec ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false \
  publish_robot_description:=false \
  use_zsibot_bridge:=true \
  use_lidar_to_body_odom:=true \
  lidar_odom_topic:="${LIDAR_ODOM_TOPIC}" \
  body_odom_topic:="${BODY_ODOM_TOPIC}" \
  body_odom_frame_id:="${BODY_FRAME_ID}" \
  body_odom_sensor_frame_id:="${SENSOR_FRAME_ID}" \
  body_odom_world_frame_id:="${GRID_FRAME_ID}" \
  body_odom_publish_tf:=false \
  body_to_sensor_x:="${BODY_TO_SENSOR_X}" \
  body_to_sensor_y:="${BODY_TO_SENSOR_Y}" \
  body_to_sensor_z:="${BODY_TO_SENSOR_Z}" \
  body_to_sensor_roll:="${BODY_TO_SENSOR_ROLL}" \
  body_to_sensor_pitch:="${BODY_TO_SENSOR_PITCH}" \
  body_to_sensor_yaw:="${BODY_TO_SENSOR_YAW}" \
  real_cmd_vel_topic:="${CMD_VEL_TOPIC}" \
  real_sensor_pose_topic:="${LIDAR_ODOM_TOPIC}" \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:="${GRID_FRAME_ID}" \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:="${GRID_FRAME_ID}"
