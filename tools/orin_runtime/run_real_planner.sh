#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

# 脚本既可放在独立 runtime 包根目录，也可直接从源码仓库的
# tools/orin_runtime 目录运行。按这个顺序寻找 overlay，避免源码工作区
# 已经 colcon build 成功却因为脚本相对路径不同而启动失败。
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/install/setup.bash"
elif [[ -f "${SCRIPT_DIR}/../../install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/../../install/setup.bash"
else
  echo "ERROR: 找不到 ROS 2 工作空间 install/setup.bash。请先在 SCAN-Planner 根目录执行 colcon build。" >&2
  exit 1
fi

CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/scan_planner/cmd_vel}"
LIDAR_ODOM_TOPIC="${LIDAR_ODOM_TOPIC:-/state_estimation_global}"
BODY_ODOM_TOPIC="${BODY_ODOM_TOPIC:-/body_state_estimation_global}"
GRID_FRAME_ID="${GRID_FRAME_ID:-lio_map}"
BODY_FRAME_ID="${BODY_FRAME_ID:-scan_base_link}"
SENSOR_FRAME_ID="${SENSOR_FRAME_ID:-livox_frame}"
# Calibrated scan_base_link -> livox_frame translation from
# doc/calibration_results.yaml:
#   lidar_front -> imu_front + imu_front -> base
BODY_TO_SENSOR_X="${BODY_TO_SENSOR_X:-0.13011}"
BODY_TO_SENSOR_Y="${BODY_TO_SENSOR_Y:--0.02329}"
BODY_TO_SENSOR_Z="${BODY_TO_SENSOR_Z:-0.17598}"
BODY_TO_SENSOR_ROLL="${BODY_TO_SENSOR_ROLL:-0.0}"
BODY_TO_SENSOR_PITCH="${BODY_TO_SENSOR_PITCH:-0.0}"
BODY_TO_SENSOR_YAW="${BODY_TO_SENSOR_YAW:-0.0}"
CONTROLLER_DRIVE_MODE="${CONTROLLER_DRIVE_MODE:-pure_pursuit}"
CONTROLLER_TRACKING_MODE="${CONTROLLER_TRACKING_MODE:-path_follow}"
CONTROLLER_MAX_VX="${CONTROLLER_MAX_VX:-0.3}"
CONTROLLER_MAX_VY="${CONTROLLER_MAX_VY:-0.15}"
CONTROLLER_MAX_VYAW="${CONTROLLER_MAX_VYAW:-0.5}"
CONTROLLER_LOOKAHEAD_DIST="${CONTROLLER_LOOKAHEAD_DIST:-0.45}"
CONTROLLER_YAW_LOOKAHEAD_DIST="${CONTROLLER_YAW_LOOKAHEAD_DIST:-0.55}"
CONTROLLER_PURE_PURSUIT_SPEED="${CONTROLLER_PURE_PURSUIT_SPEED:-0.20}"
CONTROLLER_HEADING_ERROR_THRESHOLD="${CONTROLLER_HEADING_ERROR_THRESHOLD:-0.80}"
CONTROLLER_HEADING_ERROR_EXIT_THRESHOLD="${CONTROLLER_HEADING_ERROR_EXIT_THRESHOLD:-0.45}"
CONTROLLER_HEADING_SLOWDOWN_START="${CONTROLLER_HEADING_SLOWDOWN_START:-0.20}"
CONTROLLER_ALIGN_HEADING_GAIN="${CONTROLLER_ALIGN_HEADING_GAIN:-0.80}"
CONTROLLER_PP_HEADING_GAIN="${CONTROLLER_PP_HEADING_GAIN:-0.35}"
CONTROLLER_LATERAL_ERROR_DEADBAND="${CONTROLLER_LATERAL_ERROR_DEADBAND:-0.04}"
CONTROLLER_HEADING_ERROR_DEADBAND="${CONTROLLER_HEADING_ERROR_DEADBAND:-0.035}"
CONTROLLER_CURVATURE_DEADBAND="${CONTROLLER_CURVATURE_DEADBAND:-0.04}"
CONTROLLER_CURVATURE_SPEED_GAIN="${CONTROLLER_CURVATURE_SPEED_GAIN:-0.60}"
CONTROLLER_YAW_RATE_RESERVE="${CONTROLLER_YAW_RATE_RESERVE:-0.03}"
CONTROLLER_MAX_LATERAL_ACCELERATION="${CONTROLLER_MAX_LATERAL_ACCELERATION:-0.20}"
CONTROLLER_MAX_LINEAR_ACCELERATION="${CONTROLLER_MAX_LINEAR_ACCELERATION:-0.35}"
CONTROLLER_MAX_LINEAR_DECELERATION="${CONTROLLER_MAX_LINEAR_DECELERATION:-0.60}"
CONTROLLER_YAW_RATE_DEADBAND="${CONTROLLER_YAW_RATE_DEADBAND:-0.025}"
CONTROLLER_YAW_FILTER_TIME_CONSTANT="${CONTROLLER_YAW_FILTER_TIME_CONSTANT:-0.12}"
CONTROLLER_MAX_YAW_ACCELERATION="${CONTROLLER_MAX_YAW_ACCELERATION:-1.0}"
CONTROLLER_YAW_REVERSAL_THRESHOLD="${CONTROLLER_YAW_REVERSAL_THRESHOLD:-0.12}"
CONTROLLER_YAW_START_THRESHOLD="${CONTROLLER_YAW_START_THRESHOLD:-0.06}"
CONTROLLER_YAW_STOP_THRESHOLD="${CONTROLLER_YAW_STOP_THRESHOLD:-0.03}"
CONTROLLER_MIN_NONZERO_YAW_RATE="${CONTROLLER_MIN_NONZERO_YAW_RATE:-0.10}"
CONTROLLER_MIN_NONZERO_LINEAR_SPEED="${CONTROLLER_MIN_NONZERO_LINEAR_SPEED:-0.05}"
CONTROLLER_YAW_SYNC_THRESHOLD="${CONTROLLER_YAW_SYNC_THRESHOLD:-0.10}"
CONTROLLER_LAUNCH_YAW_READINESS_RATIO="${CONTROLLER_LAUNCH_YAW_READINESS_RATIO:-0.85}"
CONTROLLER_PLANNER_HEARTBEAT_TIMEOUT="${CONTROLLER_PLANNER_HEARTBEAT_TIMEOUT:-0.40}"
CONTROLLER_CONTROL_RATE="${CONTROLLER_CONTROL_RATE:-50.0}"
FSM_EMERGENCY_TIME="${FSM_EMERGENCY_TIME:-1.0}"
FSM_LOCAL_PROGRESS_COLLISION_BACKTRACK="${FSM_LOCAL_PROGRESS_COLLISION_BACKTRACK:-0.10}"
FSM_LOCAL_PROGRESS_MAX_CROSS_TRACK="${FSM_LOCAL_PROGRESS_MAX_CROSS_TRACK:-0.40}"
FSM_LOCAL_RECOVERY_CHECK_MIN_CROSS_TRACK="${FSM_LOCAL_RECOVERY_CHECK_MIN_CROSS_TRACK:-0.05}"
FSM_LOCAL_RECOVERY_LOOKAHEAD="${FSM_LOCAL_RECOVERY_LOOKAHEAD:-${CONTROLLER_LOOKAHEAD_DIST}}"
FSM_LOCAL_RECOVERY_SAMPLE_DISTANCE="${FSM_LOCAL_RECOVERY_SAMPLE_DISTANCE:-0.05}"
FSM_LOCAL_FINISH_SPEED="${FSM_LOCAL_FINISH_SPEED:-0.06}"
PLANNER_MAX_VEL="${PLANNER_MAX_VEL:-0.40}"
PLANNER_MAX_ACC="${PLANNER_MAX_ACC:-0.50}"

exec ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=3 \
  use_global_path_publisher:=true \
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
  controller_heading_error_threshold:="${CONTROLLER_HEADING_ERROR_THRESHOLD}" \
  controller_heading_error_exit_threshold:="${CONTROLLER_HEADING_ERROR_EXIT_THRESHOLD}" \
  controller_heading_slowdown_start:="${CONTROLLER_HEADING_SLOWDOWN_START}" \
  controller_align_heading_gain:="${CONTROLLER_ALIGN_HEADING_GAIN}" \
  controller_pp_heading_gain:="${CONTROLLER_PP_HEADING_GAIN}" \
  controller_lateral_error_deadband:="${CONTROLLER_LATERAL_ERROR_DEADBAND}" \
  controller_heading_error_deadband:="${CONTROLLER_HEADING_ERROR_DEADBAND}" \
  controller_curvature_deadband:="${CONTROLLER_CURVATURE_DEADBAND}" \
  controller_curvature_speed_gain:="${CONTROLLER_CURVATURE_SPEED_GAIN}" \
  controller_yaw_rate_reserve:="${CONTROLLER_YAW_RATE_RESERVE}" \
  controller_max_lateral_acceleration:="${CONTROLLER_MAX_LATERAL_ACCELERATION}" \
  controller_max_linear_acceleration:="${CONTROLLER_MAX_LINEAR_ACCELERATION}" \
  controller_max_linear_deceleration:="${CONTROLLER_MAX_LINEAR_DECELERATION}" \
  controller_yaw_rate_deadband:="${CONTROLLER_YAW_RATE_DEADBAND}" \
  controller_yaw_filter_time_constant:="${CONTROLLER_YAW_FILTER_TIME_CONSTANT}" \
  controller_max_yaw_acceleration:="${CONTROLLER_MAX_YAW_ACCELERATION}" \
  controller_yaw_reversal_threshold:="${CONTROLLER_YAW_REVERSAL_THRESHOLD}" \
  controller_yaw_start_threshold:="${CONTROLLER_YAW_START_THRESHOLD}" \
  controller_yaw_stop_threshold:="${CONTROLLER_YAW_STOP_THRESHOLD}" \
  controller_min_nonzero_yaw_rate:="${CONTROLLER_MIN_NONZERO_YAW_RATE}" \
  controller_min_nonzero_linear_speed:="${CONTROLLER_MIN_NONZERO_LINEAR_SPEED}" \
  controller_yaw_sync_threshold:="${CONTROLLER_YAW_SYNC_THRESHOLD}" \
  controller_launch_yaw_readiness_ratio:="${CONTROLLER_LAUNCH_YAW_READINESS_RATIO}" \
  controller_planner_heartbeat_timeout:="${CONTROLLER_PLANNER_HEARTBEAT_TIMEOUT}" \
  controller_control_rate:="${CONTROLLER_CONTROL_RATE}" \
  fsm_emergency_time:="${FSM_EMERGENCY_TIME}" \
  fsm_local_progress_collision_backtrack:="${FSM_LOCAL_PROGRESS_COLLISION_BACKTRACK}" \
  fsm_local_progress_max_cross_track:="${FSM_LOCAL_PROGRESS_MAX_CROSS_TRACK}" \
  fsm_local_recovery_check_min_cross_track:="${FSM_LOCAL_RECOVERY_CHECK_MIN_CROSS_TRACK}" \
  fsm_local_recovery_lookahead:="${FSM_LOCAL_RECOVERY_LOOKAHEAD}" \
  fsm_local_recovery_sample_distance:="${FSM_LOCAL_RECOVERY_SAMPLE_DISTANCE}" \
  fsm_local_finish_speed:="${FSM_LOCAL_FINISH_SPEED}" \
  manager_max_vel:="${PLANNER_MAX_VEL}" \
  manager_max_acc:="${PLANNER_MAX_ACC}" \
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
  body_odom_transform_twist:=true \
  body_to_sensor_x:="${BODY_TO_SENSOR_X}" \
  body_to_sensor_y:="${BODY_TO_SENSOR_Y}" \
  body_to_sensor_z:="${BODY_TO_SENSOR_Z}" \
  body_to_sensor_roll:="${BODY_TO_SENSOR_ROLL}" \
  body_to_sensor_pitch:="${BODY_TO_SENSOR_PITCH}" \
  body_to_sensor_yaw:="${BODY_TO_SENSOR_YAW}" \
  real_cmd_vel_topic:="${CMD_VEL_TOPIC}" \
  real_sensor_pose_topic:="${LIDAR_ODOM_TOPIC}" \
  real_cloud_topic:=/cloud_registered_global \
  real_grid_frame_id:="${GRID_FRAME_ID}" \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:="${GRID_FRAME_ID}"
