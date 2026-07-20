#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

exec ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false \
  publish_robot_description:=false \
  use_zsibot_bridge:=false \
  use_zsibot_udp_client:=true \
  real_body_pose_topic:=/state_estimation \
  real_sensor_pose_topic:=/state_estimation \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:=odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false
