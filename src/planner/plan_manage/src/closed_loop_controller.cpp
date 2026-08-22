// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file closed_loop_controller.cpp @brief Closed-loop B-spline tracking controller. */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/planner_heartbeat.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/bspline_message_validator.h"
#include "plan_manage/local_trajectory_tracker.h"
#include "plan_manage/path_command_governor.h"
#include "plan_manage/path_tracking_controller.h"
#include "plan_manage/planner_heartbeat_gate.h"
#include "plan_manage/slew_rate_limiter.h"
#include "plan_manage/yaw_command_filter.h"

namespace scan_planner
{
/** @brief Tracks local B-splines using odometry feedback and publishes body velocity commands. */
class ClosedLoopController : public rclcpp::Node
{
public:
  /** @brief Loads controller parameters and creates ROS interfaces. */
  ClosedLoopController() : Node("omni_closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    heading_error_exit_threshold_ =
        declare_parameter<double>("heading_error_exit_threshold", 0.45);
    heading_slowdown_start_ =
        declare_parameter<double>("heading_slowdown_start", 0.20);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    align_heading_gain_ = declare_parameter<double>("align_heading_gain", 0.80);
    pp_heading_gain_ = declare_parameter<double>("pp_heading_gain", 0.35);
    max_vx_ = std::max(0.01, declare_parameter<double>("max_vx", 0.75));
    max_vy_ = std::max(0.0, declare_parameter<double>("max_vy", 0.35));
    max_vyaw_ = std::clamp(
        declare_parameter<double>("max_vyaw", 1.0), 0.01, kMaxVYawLimit);
    finish_dist_ = std::max(0.0, declare_parameter<double>("finish_dist", 0.15));
    tracking_mode_ = declare_parameter<std::string>("tracking_mode", "path_follow");
    drive_mode_ = declare_parameter<std::string>("drive_mode", "pure_pursuit");
    path_sample_dt_ = std::max(0.02, declare_parameter<double>("path_sample_dt", 0.05));
    lookahead_dist_ = std::max(0.05, declare_parameter<double>("lookahead_dist", 0.35));
    yaw_lookahead_dist_ = std::max(0.05, declare_parameter<double>("yaw_lookahead_dist", 0.45));
    min_path_speed_ = std::max(0.0, declare_parameter<double>("min_path_speed", 0.08));
    pure_pursuit_speed_ = std::max(0.0, declare_parameter<double>("pure_pursuit_speed", 0.20));
    lateral_error_deadband_ =
        std::max(0.0, declare_parameter<double>("lateral_error_deadband", 0.04));
    heading_error_deadband_ =
        std::max(0.0, declare_parameter<double>("heading_error_deadband", 0.035));
    curvature_deadband_ =
        std::max(0.0, declare_parameter<double>("curvature_deadband", 0.04));
    curvature_speed_gain_ =
        std::max(0.0, declare_parameter<double>("curvature_speed_gain", 0.60));
    yaw_rate_reserve_ =
        std::max(0.0, declare_parameter<double>("yaw_rate_reserve", 0.03));
    max_lateral_acceleration_ =
        std::max(0.01, declare_parameter<double>("max_lateral_acceleration", 0.20));
    max_linear_acceleration_ =
        std::max(0.01, declare_parameter<double>("max_linear_acceleration", 0.35));
    max_linear_deceleration_ =
        std::max(0.01, declare_parameter<double>("max_linear_deceleration", 0.60));
    yaw_rate_deadband_ = std::clamp(
        declare_parameter<double>("yaw_rate_deadband", 0.025),
        0.0,
        max_vyaw_);
    yaw_filter_time_constant_ =
        std::max(0.0, declare_parameter<double>("yaw_filter_time_constant", 0.12));
    max_yaw_acceleration_ =
        std::max(0.01, declare_parameter<double>("max_yaw_acceleration", 1.0));
    yaw_reversal_threshold_ = std::clamp(
        declare_parameter<double>("yaw_reversal_threshold", 0.12),
        yaw_rate_deadband_,
        max_vyaw_);
    yaw_start_threshold_ = std::clamp(
        declare_parameter<double>("yaw_start_threshold", 0.06),
        yaw_rate_deadband_,
        max_vyaw_);
    yaw_stop_threshold_ =
        std::clamp(
            declare_parameter<double>("yaw_stop_threshold", 0.03),
            0.0,
            yaw_start_threshold_);
    min_nonzero_yaw_rate_ =
        std::clamp(
            declare_parameter<double>("min_nonzero_yaw_rate", 0.10),
            0.0,
            max_vyaw_);
    yaw_command_filter_.setConfig(
        YawCommandFilter::Config{
            yaw_rate_deadband_,
            yaw_filter_time_constant_,
            max_yaw_acceleration_,
            yaw_reversal_threshold_,
            yaw_start_threshold_,
            yaw_stop_threshold_,
            min_nonzero_yaw_rate_});
    min_nonzero_linear_speed_ = std::clamp(
        declare_parameter<double>("min_nonzero_linear_speed", 0.05),
        0.001,
        max_vx_);
    yaw_sync_threshold_ = std::clamp(
        declare_parameter<double>("yaw_sync_threshold", 0.10),
        yaw_rate_deadband_,
        max_vyaw_);
    launch_yaw_readiness_ratio_ = std::clamp(
        declare_parameter<double>("launch_yaw_readiness_ratio", 0.85),
        0.10,
        0.99);
    odom_timeout_ = std::max(0.05, declare_parameter<double>("odom_timeout", 0.30));
    planner_heartbeat_timeout_ =
        std::max(0.10, declare_parameter<double>("planner_heartbeat_timeout", 0.40));
    max_bspline_duration_ =
        std::max(1.0, declare_parameter<double>("max_bspline_duration", 60.0));
    // ROS 2 的 PARAMETER_INTEGER 统一按 int64_t 保存；显式保持两侧类型一致，
    // 避免在 aarch64/Humble 上因 int 与 long 类型不同导致 std::max 编译失败。
    const std::int64_t configured_max_control_points =
        declare_parameter<std::int64_t>("max_bspline_control_points", std::int64_t{1000});
    max_bspline_control_points_ = static_cast<std::size_t>(std::max<std::int64_t>(
        std::int64_t{4}, configured_max_control_points));
    control_rate_ = std::clamp(
        declare_parameter<double>("control_rate", 50.0), 10.0, 200.0);
    local_progress_max_advance_ = std::max(
        0.10, declare_parameter<double>("local_progress_max_advance", 0.75));
    local_progress_movement_scale_ = std::max(
        1.0, declare_parameter<double>("local_progress_movement_scale", 1.5));
    local_progress_movement_deadband_ = std::max(
        0.0, declare_parameter<double>("local_progress_movement_deadband", 0.01));
    local_progress_initial_credit_ = std::max(
        0.0, declare_parameter<double>("local_progress_initial_credit", 0.15));
    expected_frame_id_ = declare_parameter<std::string>("expected_frame_id", "");

    local_trajectory_tracker_.setConfig(
        LocalTrajectoryTracker::Config{
            local_progress_max_advance_,
            local_progress_movement_scale_,
            local_progress_movement_deadband_,
            local_progress_initial_credit_});
    path_tracking_controller_.setConfig(
        PathTrackingController::Config{
            pure_pursuit_speed_,
            max_vx_,
            max_vyaw_,
            align_heading_gain_,
            pp_heading_gain_,
            heading_error_threshold_,
            heading_error_exit_threshold_,
            heading_slowdown_start_,
            curvature_speed_gain_,
            yaw_rate_reserve_,
            max_lateral_acceleration_,
            max_linear_deceleration_,
            finish_dist_,
            lateral_error_deadband_,
            curvature_deadband_,
            0.05});
    linear_x_limiter_.setConfig(
        SlewRateLimiter::Config{
            max_linear_acceleration_, max_linear_deceleration_});
    linear_y_limiter_.setConfig(
        SlewRateLimiter::Config{
            max_linear_acceleration_, max_linear_deceleration_});
    PathCommandGovernor::Config governor_config;
    governor_config.max_forward_speed = max_vx_;
    governor_config.max_yaw_rate = max_vyaw_;
    governor_config.min_nonzero_linear_speed = min_nonzero_linear_speed_;
    governor_config.yaw_sync_threshold = yaw_sync_threshold_;
    governor_config.yaw_direction_deadband = yaw_rate_deadband_;
    governor_config.launch_yaw_readiness_ratio =
        launch_yaw_readiness_ratio_;
    governor_config.linear_limiter = SlewRateLimiter::Config{
        max_linear_acceleration_, max_linear_deceleration_};
    governor_config.yaw_filter = YawCommandFilter::Config{
        yaw_rate_deadband_,
        yaw_filter_time_constant_,
        max_yaw_acceleration_,
        yaw_reversal_threshold_,
        yaw_start_threshold_,
        yaw_stop_threshold_,
        min_nonzero_yaw_rate_};
    path_command_governor_.setConfig(governor_config);
    if (tracking_mode_ != "path_follow" && tracking_mode_ != "time")
    {
      RCLCPP_WARN(get_logger(), "Unknown tracking_mode '%s'; falling back to path_follow",
                  tracking_mode_.c_str());
      tracking_mode_ = "path_follow";
    }
    if (drive_mode_ != "pure_pursuit" && drive_mode_ != "omni")
    {
      RCLCPP_WARN(get_logger(), "Unknown drive_mode '%s'; falling back to pure_pursuit",
                  drive_mode_.c_str());
      drive_mode_ = "pure_pursuit";
    }
    if (tracking_mode_ == "time")
    {
      RCLCPP_WARN(
          get_logger(),
          "tracking_mode=time is a legacy mode: planner heartbeat only authorizes "
          "motion; its spatial progress is intentionally not applied to exec_time");
    }

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    planner_heartbeat_sub_ =
        create_subscription<scan_planner_msgs::msg::PlannerHeartbeat>(
            "planning/planner_heartbeat",
            rclcpp::QoS(1).reliable().durability_volatile(),
            std::bind(
                &ClosedLoopController::plannerHeartbeatCallback,
                this,
                std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    controller_bspline_pub_ = create_publisher<nav_msgs::msg::Path>(
        "planning/controller_bspline", rclcpp::QoS(10).reliable().transient_local());
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "planning/local_path", rclcpp::QoS(10).reliable().transient_local());
    controller_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "planning/controller_target", 20);
    cmd_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready, tracking_mode=%s, drive_mode=%s",
                tracking_mode_.c_str(), drive_mode_.c_str());
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  /** @brief Wraps an angle to [-pi, pi]. @param angle Input angle. @return Wrapped angle. */
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  /** @brief Limits a vector magnitude. @param value Input vector. @param max_norm Maximum norm. @return Limited vector. */
  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  /** @brief Safely normalizes a vector. @param value Input vector. @return Unit vector or zero. */
  static Eigen::Vector2d normalizedOrZero(const Eigen::Vector2d &value)
  {
    const double norm = value.norm();
    if (norm < 1e-6)
      return Eigen::Vector2d::Zero();
    return value / norm;
  }

  /** @brief Estimates desired heading from the trajectory tangent. @param t_cur Current trajectory time. @param pos_des Desired position. @return Desired yaw. */
  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  /**
   * @brief Filters and publishes a velocity command.
   * @param command Raw controller command.
   * @param dt Controller period in seconds.
   */
  void publishFilteredCommand(
      geometry_msgs::msg::Twist command,
      double dt)
  {
    command.linear.x = std::clamp(
        linear_x_limiter_.update(command.linear.x, dt), -max_vx_, max_vx_);
    command.linear.y = std::clamp(
        linear_y_limiter_.update(command.linear.y, dt), -max_vy_, max_vy_);
    const double raw_yaw_rate =
        std::clamp(command.angular.z, -max_vyaw_, max_vyaw_);
    command.angular.z = std::clamp(
        yaw_command_filter_.update(raw_yaw_rate, dt),
        -max_vyaw_,
        max_vyaw_);
    cmd_vel_pub_->publish(command);
  }

  /** @brief 用无 ROS 的联合治理器生成并发布真机 Pure Pursuit 命令。 */
  PathCommandGovernor::Command publishPathTrackingCommand(
      const PathTrackingController::Command &target,
      double dt)
  {
    const PathCommandGovernor::Target governor_target{
        target.linear_x,
        target.hard_speed_limit,
        target.curvature,
        target.heading_yaw,
        target.rotate_in_place,
        target.allow_minimum_speed_creep};
    const PathCommandGovernor::Command governed =
        path_command_governor_.update(governor_target, dt);
    geometry_msgs::msg::Twist command;
    if (governed.valid)
    {
      command.linear.x = governed.linear_x;
      command.angular.z = governed.angular_z;
    }
    cmd_vel_pub_->publish(command);
    return governed;
  }

  /**
   * @brief Publishes an immediate zero command and clears filter memory.
   *
   * Safety stops deliberately bypass smoothing so stale angular velocity can
   * never delay a stop.
   */
  void publishImmediateStop()
  {
    yaw_command_filter_.reset();
    linear_x_limiter_.reset();
    linear_y_limiter_.reset();
    path_command_governor_.reset();
    path_tracking_controller_.reset();
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  /**
   * @brief Publishes a smoothed in-place rotation command.
   * @param yaw_rate Raw desired yaw rate in rad/s.
   * @param dt Controller period in seconds.
   */
  void publishYawOnly(double yaw_rate, double dt)
  {
    geometry_msgs::msg::Twist command;
    command.angular.z = yaw_rate;
    publishFilteredCommand(command, dt);
  }

  /** @brief Reports whether trajectory time is frozen. @param frozen Freeze state. */
  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  /** @brief 接收规划器心跳；超时计龄使用本机 steady clock，不依赖 ROS 时间。 */
  void plannerHeartbeatCallback(
      const scan_planner_msgs::msg::PlannerHeartbeat::ConstSharedPtr msg)
  {
    if (!msg)
      return;
    heartbeat_progress_valid_ =
        msg->motion_allowed && msg->traj_id > 0 &&
        std::isfinite(msg->progress_time) && msg->progress_time >= 0.0 &&
        std::isfinite(msg->progress_arc_length) &&
        msg->progress_arc_length >= 0.0;
    // 新协议把空间进度作为运动授权的一部分；字段非法时即使
    // motion_allowed=true 也拒绝执行，避免重启后从错误位置恢复。
    planner_heartbeat_gate_.update(
        heartbeat_progress_valid_, msg->traj_id, msg->trajectory_start_time);
    if (heartbeat_progress_valid_)
    {
      heartbeat_progress_traj_id_ = msg->traj_id;
      heartbeat_progress_start_time_ = msg->trajectory_start_time;
      heartbeat_progress_time_ = msg->progress_time;
      heartbeat_progress_arc_length_ = msg->progress_arc_length;
    }
    applyHeartbeatProgress();
  }

  /** @brief 心跳进度是否与当前 B-spline 的双身份和取值范围一致。 */
  bool heartbeatProgressMatchesTrajectory() const
  {
    if (!heartbeat_progress_valid_ || !receive_traj_ ||
        heartbeat_progress_traj_id_ != traj_id_ ||
        !PlannerHeartbeatGate::sameTimestamp(
            heartbeat_progress_start_time_, traj_start_time_) ||
        heartbeat_progress_time_ > traj_duration_ + 0.10)
    {
      return false;
    }
    return !local_trajectory_tracker_.active() ||
        heartbeat_progress_arc_length_ <=
            local_trajectory_tracker_.totalLength() + 0.10;
  }

  /** @brief path_follow 模式下把匹配双身份的 FSM 空间进度应用到 tracker。 */
  void applyHeartbeatProgress()
  {
    // time 模式的 exec_time_ 只能由本地控制周期推进；若再写入 FSM 的空间
    // 投影时间，两套推进方式会每个心跳周期互相拉扯。time 模式仍严格校验
    // 心跳身份和新鲜度，但 progress_time/progress_arc_length 只用于授权校验。
    if (tracking_mode_ != "path_follow" ||
        !heartbeatProgressMatchesTrajectory() ||
        !local_trajectory_tracker_.active())
      return;

    local_trajectory_tracker_.advanceTo(heartbeat_progress_arc_length_);
    path_progress_s_ = local_trajectory_tracker_.progress();
    exec_time_ = local_trajectory_tracker_.progressTime();
  }

  /** @brief 当前心跳是否新鲜且准确授权了正在缓存的 B-spline。 */
  bool plannerHeartbeatReady() const
  {
    if (!heartbeatProgressMatchesTrajectory())
      return false;
    return planner_heartbeat_gate_.authorized(
        traj_id_, traj_start_time_, planner_heartbeat_timeout_);
  }

  /** @brief Validates all odometry numeric fields. @param msg Odometry. @return True when finite. */
  static bool finiteOdometry(const nav_msgs::msg::Odometry &msg)
  {
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    const double q_norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q_norm2) && q_norm2 > 1.0e-12;
  }

  /** @brief Clears active trajectory and stops motion. @param reason Diagnostic reason. @param safety_stop Whether to latch safety-stop behavior. */
  void clearTrajectory(const std::string &reason, bool safety_stop = false)
  {
    if (receive_traj_)
      RCLCPP_INFO(get_logger(), "Clearing active trajectory: %s", reason.c_str());
    receive_traj_ = false;
    traj_.clear();
    sampled_path_.clear();
    sampled_times_.clear();
    sampled_arc_lengths_.clear();
    local_trajectory_tracker_.clear();
    traj_duration_ = 0.0;
    traj_id_ = -1;
    traj_start_time_ = builtin_interfaces::msg::Time();
    traj_frame_id_.clear();
    heartbeat_progress_valid_ = false;
    heartbeat_progress_traj_id_ = -1;
    heartbeat_progress_time_ = 0.0;
    heartbeat_progress_arc_length_ = 0.0;
    planner_heartbeat_gate_.invalidate();
    exec_time_ = 0.0;
    closest_path_idx_ = 0;
    path_progress_s_ = 0.0;
    omni_aligning_ = false;
    time_aligning_ = false;
    safety_stop_active_ = safety_stop;
    publishExecutionFrozen(safety_stop_active_);
    publishImmediateStop();
  }

  /** @brief Receives a new local B-spline. @param msg B-spline message. */
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (!msg)
      return;

    const std::string required_frame =
        expected_frame_id_.empty() ? odom_frame_id_ : expected_frame_id_;
    const BsplineValidationResult validation = validateBsplineMessage(
        *msg,
        required_frame,
        max_bspline_control_points_,
        max_bspline_duration_);
    if (validation.clear_command)
    {
      clearTrajectory("planner clear command", false);
      return;
    }
    if (!validation.valid)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Rejecting B-spline trajectory %lld: %s",
          static_cast<long long>(msg->traj_id),
          validation.reason.c_str());
      clearTrajectory("malformed B-spline", true);
      return;
    }

    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    if (!std::isfinite(traj_duration_) ||
        std::abs(traj_duration_ - validation.duration) > 1.0e-6)
    {
      clearTrajectory("B-spline duration changed during construction", true);
      return;
    }
    traj_id_ = msg->traj_id;
    traj_start_time_ = msg->start_time;
    traj_frame_id_ = msg->header.frame_id;
    exec_time_ = 0.0;
    closest_path_idx_ = 0;
    path_progress_s_ = 0.0;
    last_update_time_ = now();
    if (!sampleControllerPath())
    {
      clearTrajectory("B-spline sampling produced invalid data", true);
      return;
    }
    receive_traj_ = true;
    // 只有一条完整通过校验、采样和进度初始化的轨迹，才能解除轨迹故障锁存。
    safety_stop_active_ = false;
    // heartbeat 可能先于 transient B-spline 到达；轨迹安装完成后再补应用一次。
    applyHeartbeatProgress();
    const auto &first = msg->pos_pts.front();
    const auto &last = msg->pos_pts.back();
    RCLCPP_INFO(get_logger(),
                "Received trajectory %lld, frame='%s', duration %.3fs, "
                "ctrl_pts=%zu, first=[%.3f %.3f %.3f], last=[%.3f %.3f %.3f]",
                static_cast<long long>(traj_id_), traj_frame_id_.c_str(),
                traj_duration_, msg->pos_pts.size(),
                first.x, first.y, first.z, last.x, last.y, last.z);
    publishControllerBspline();
  }

  /** @brief Receives body odometry. @param msg Odometry message. */
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (msg->header.frame_id.empty() || !finiteOdometry(*msg))
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting body_pose with empty frame_id, non-finite pose, or invalid quaternion");
      clearTrajectory("invalid body_pose", true);
      have_odom_ = false;
      return;
    }
    if (!expected_frame_id_.empty() && msg->header.frame_id != expected_frame_id_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting body_pose in frame '%s'; expected '%s'",
          msg->header.frame_id.c_str(), expected_frame_id_.c_str());
      clearTrajectory("body_pose frame mismatch", true);
      have_odom_ = false;
      return;
    }
    if (!traj_frame_id_.empty() && msg->header.frame_id != traj_frame_id_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting body_pose in frame '%s'; active B-spline uses '%s'",
          msg->header.frame_id.c_str(), traj_frame_id_.c_str());
      clearTrajectory("body_pose and B-spline frame mismatch", true);
      have_odom_ = false;
      return;
    }
    if (odom_frame_id_ != msg->header.frame_id)
    {
      odom_frame_id_ = msg->header.frame_id;
      RCLCPP_INFO(get_logger(), "Controller odom frame is '%s', child_frame_id='%s'",
                  odom_frame_id_.c_str(), msg->child_frame_id.c_str());
    }
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    tf2::Quaternion odom_orientation(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    odom_orientation.normalize();
    odom_yaw_ = tf2::getYaw(odom_orientation);
    have_odom_ = true;
    last_odom_time_ = now();
    // 若上一条轨迹因坏消息/坐标系错误被清除，单独恢复 odom 不能让旧轨迹
    // 复活；必须等规划器重新发布一条完整合法的 B-spline。
    if (receive_traj_)
      safety_stop_active_ = false;
  }

  /** @brief Publishes the active B-spline as a sampled path. */
  void publishControllerBspline()
  {
    if (sampled_path_.empty() || traj_frame_id_.empty())
      return;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = traj_frame_id_;
    path.poses.reserve(sampled_path_.size());
    for (const Eigen::Vector3d &pos : sampled_path_)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.position.z = pos.z();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    controller_bspline_pub_->publish(path);
    local_path_pub_->publish(path);
  }

  /** @brief Publishes the current tracking target marker. @param pos_des Desired position. @param vel_des Desired velocity. */
  void publishControllerTarget(const Eigen::Vector3d &pos_des,
                               const rclcpp::Time &stamp)
  {
    if (traj_frame_id_.empty())
      return;

    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = stamp;
    target.header.frame_id = traj_frame_id_;
    target.pose.position.x = pos_des.x();
    target.pose.position.y = pos_des.y();
    target.pose.position.z = pos_des.z();
    target.pose.orientation.w = 1.0;
    controller_target_pub_->publish(target);
  }

  /** @brief 采样并验证活动 B-spline，同时初始化真实空间进度跟踪器。 */
  bool sampleControllerPath()
  {
    sampled_path_.clear();
    sampled_times_.clear();
    sampled_arc_lengths_.clear();

    if (traj_.empty() || traj_duration_ < 1e-6)
      return false;

    const int sample_num =
        std::max(2, static_cast<int>(std::ceil(traj_duration_ / path_sample_dt_)) + 1);
    if (sample_num > 10000)
      return false;
    sampled_path_.reserve(sample_num);
    sampled_times_.reserve(sample_num);
    sampled_arc_lengths_.reserve(sample_num);

    double arc_length = 0.0;
    for (int i = 0; i < sample_num; ++i)
    {
      const double t = traj_duration_ * static_cast<double>(i) /
          static_cast<double>(sample_num - 1);
      const Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(t);
      const Eigen::Vector3d vel = traj_[1].evaluateDeBoorT(t);
      const Eigen::Vector3d acc = traj_[2].evaluateDeBoorT(t);
      if (!pos.allFinite() || !vel.allFinite() || !acc.allFinite())
        return false;
      if (!sampled_path_.empty())
      {
        arc_length += (pos.head<2>() - sampled_path_.back().head<2>()).norm();
      }
      sampled_path_.push_back(pos);
      sampled_times_.push_back(t);
      sampled_arc_lengths_.push_back(arc_length);
    }

    local_trajectory_tracker_.clear();
    if (arc_length > 1.0e-8)
    {
      try
      {
        local_trajectory_tracker_.reset(
            sampled_path_,
            sampled_times_,
            have_odom_ ? odom_pos_ : sampled_path_.front());
        // tracker 会删除平面重复点；控制目标与进度必须使用同一份折线。
        sampled_path_ = local_trajectory_tracker_.points();
        sampled_times_ = local_trajectory_tracker_.times();
        sampled_arc_lengths_ = local_trajectory_tracker_.arcLengths();
      }
      catch (const std::invalid_argument &error)
      {
        RCLCPP_ERROR(
            get_logger(), "Unable to initialize controller path progress: %s",
            error.what());
        return false;
      }
    }
    return sampled_path_.size() >= 2;
  }

  /** @brief Finds the nearest forward path sample. @return Closest sample index. */
  size_t findClosestPathIndex()
  {
    if (sampled_path_.empty())
      return 0;

    if (local_trajectory_tracker_.active())
    {
      path_progress_s_ = local_trajectory_tracker_.update(odom_pos_);
      exec_time_ = local_trajectory_tracker_.progressTime();
      const auto upper = std::lower_bound(
          sampled_arc_lengths_.begin(),
          sampled_arc_lengths_.end(),
          path_progress_s_);
      closest_path_idx_ = upper == sampled_arc_lengths_.end()
          ? sampled_arc_lengths_.size() - 1
          : static_cast<std::size_t>(
                std::distance(sampled_arc_lengths_.begin(), upper));
      return closest_path_idx_;
    }

    // 合法急停轨迹可能没有平面长度；把它视为已到轨迹末端。
    exec_time_ = traj_duration_;
    closest_path_idx_ = 0;
    path_progress_s_ = 0.0;
    return closest_path_idx_;
  }

  /**
   * @brief Interpolates a continuous point at an arc length on the sampled path.
   * @param arc_length Requested path arc length in metres.
   * @return Interpolated path position.
   */
  Eigen::Vector3d pointAtArcLength(double arc_length) const
  {
    if (sampled_path_.empty())
      return Eigen::Vector3d::Zero();
    if (arc_length <= 0.0 || sampled_path_.size() == 1)
      return sampled_path_.front();
    if (arc_length >= sampled_arc_lengths_.back())
      return sampled_path_.back();

    const auto upper = std::lower_bound(
        sampled_arc_lengths_.begin(), sampled_arc_lengths_.end(), arc_length);
    const size_t upper_idx =
        static_cast<size_t>(std::distance(sampled_arc_lengths_.begin(), upper));
    const size_t lower_idx = upper_idx - 1;
    const double segment_length =
        sampled_arc_lengths_[upper_idx] - sampled_arc_lengths_[lower_idx];
    if (segment_length < 1.0e-9)
      return sampled_path_[upper_idx];
    const double ratio =
        (arc_length - sampled_arc_lengths_[lower_idx]) / segment_length;
    return sampled_path_[lower_idx] +
        ratio * (sampled_path_[upper_idx] - sampled_path_[lower_idx]);
  }

  /** @brief Returns reference speed at a sampled path index. @param idx Sample index. @return Speed. */
  double pathSpeedAt(size_t idx) const
  {
    if (traj_.size() < 2 || sampled_times_.empty())
      return min_path_speed_;
    const double t = sampled_times_[std::min(idx, sampled_times_.size() - 1)];
    const double speed = traj_[1].evaluateDeBoorT(t).head<2>().norm();
    return std::min(max_vx_, std::max(min_path_speed_, speed));
  }

  /** @brief Converts world velocity to body coordinates and publishes it. @param vel_world World-frame planar velocity. @param yaw_rate Angular velocity. */
  void publishBodyCommand(const Eigen::Vector2d &vel_world,
                          double yaw_command,
                          geometry_msgs::msg::Twist &command) const
  {
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
  }

  /** @brief 把世界系前视目标转换到机体系并生成受约束的 Pure Pursuit 目标。 */
  PathTrackingController::Command computePurePursuitCommand(
      const Eigen::Vector2d &to_target_world,
      double heading_error,
      bool target_is_end,
      double dist_to_end)
  {
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    const double local_x = c * to_target_world.x() + s * to_target_world.y();
    const double local_y = -s * to_target_world.x() + c * to_target_world.y();
    return path_tracking_controller_.compute(
        local_x, local_y, heading_error, target_is_end, dist_to_end);
  }

  /** @brief Executes one spatial path-following update. @param current_time Current ROS time. @param dt Controller period. */
  void trackPathFollowing(const rclcpp::Time &current_time, double dt)
  {
    if (sampled_path_.empty())
    {
      trackTimeFollowing(current_time, dt);
      return;
    }

    const size_t closest_idx = findClosestPathIndex();
    const double target_s = std::min(
        sampled_arc_lengths_.back(), path_progress_s_ + lookahead_dist_);
    const double yaw_target_s = std::min(
        sampled_arc_lengths_.back(), path_progress_s_ + yaw_lookahead_dist_);
    const Eigen::Vector3d pos_des = pointAtArcLength(target_s);
    publishControllerTarget(pos_des, current_time);

    Eigen::Vector2d to_target = pos_des.head<2>() - odom_pos_.head<2>();
    const double tangent_back_s = std::max(0.0, path_progress_s_ - 0.15);
    Eigen::Vector2d path_dir =
        pointAtArcLength(yaw_target_s).head<2>() -
        pointAtArcLength(tangent_back_s).head<2>();
    if (path_dir.squaredNorm() < 1e-4)
      path_dir = to_target;
    if (path_dir.squaredNorm() < 1e-4 && traj_.size() > 1)
      path_dir = traj_[1].evaluateDeBoorT(sampled_times_[closest_idx]).head<2>();

    const double desired_yaw = path_dir.squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(path_dir.y(), path_dir.x());
    const double raw_yaw_error = normalizeAngle(desired_yaw - odom_yaw_);
    const double yaw_error =
        std::abs(raw_yaw_error) < heading_error_deadband_ ? 0.0 : raw_yaw_error;
    const double yaw_command =
        std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    const Eigen::Vector2d to_end = sampled_path_.back().head<2>() - odom_pos_.head<2>();
    const bool target_is_end =
        target_s >= sampled_arc_lengths_.back() - 1.0e-6;
    const bool inside_finish_tolerance =
        target_is_end && to_end.norm() <= finish_dist_;
    last_update_time_ = current_time;

    if (inside_finish_tolerance && drive_mode_ != "pure_pursuit")
    {
      // 正常抵达终点不是安全急停：先给全向控制器零目标，让线加减速器和
      // yaw 滤波器自然收敛，状态确实归零后再做最终 reset。
      publishExecutionFrozen(false);
      publishFilteredCommand(geometry_msgs::msg::Twist(), dt);
      if (std::abs(linear_x_limiter_.output()) <= 1.0e-9 &&
          std::abs(linear_y_limiter_.output()) <= 1.0e-9 &&
          std::abs(yaw_command_filter_.output()) <= 1.0e-9)
      {
        publishImmediateStop();
      }
      return;
    }

    if (drive_mode_ == "pure_pursuit")
    {
      PathTrackingController::Command target;
      if (inside_finish_tolerance)
      {
        // 已进入终点容差后不再看位于机体侧后方的 endpoint，也不做 ALIGN。
        // 直接给普通零目标，并保留非零 hard limit，让 governor 按正常减速度
        // 收敛；失联、坏轨迹和障碍急停仍走 publishImmediateStop()。
        target.valid = true;
        target.hard_speed_limit = max_vx_;
      }
      else
      {
        target = computePurePursuitCommand(
            to_target, yaw_error, target_is_end, to_end.norm());
      }
      if (!target.valid)
      {
        clearTrajectory("non-finite Pure Pursuit target", true);
        return;
      }
      publishExecutionFrozen(target.rotate_in_place);
      const PathCommandGovernor::Command governed =
          publishPathTrackingCommand(target, dt);
      if (inside_finish_tolerance && governed.valid && governed.settled)
      {
        // 普通终点停车已经完成；此时 reset 不会造成额外速度阶跃。
        publishImmediateStop();
      }
      return;
    }

    const double abs_yaw_error = std::abs(yaw_error);
    if (omni_aligning_)
    {
      if (abs_yaw_error <= heading_error_exit_threshold_)
        omni_aligning_ = false;
    }
    else if (abs_yaw_error >= heading_error_threshold_)
    {
      omni_aligning_ = true;
    }
    if (omni_aligning_)
    {
      publishExecutionFrozen(true);
      publishYawOnly(yaw_command, dt);
      return;
    }

    publishExecutionFrozen(false);
    geometry_msgs::msg::Twist command;
    const Eigen::Vector2d vel_ff =
        normalizedOrZero(path_dir) * pathSpeedAt(closest_idx);
    const Eigen::Vector2d vel_world =
        clampNorm(vel_ff + kp_pos_ * to_target, std::max(max_vx_, max_vy_));
    publishBodyCommand(vel_world, yaw_command, command);
    publishFilteredCommand(command, dt);
  }

  /** @brief Executes one time-parameterized tracking update. @param current_time Current ROS time. @param dt Controller period. */
  void trackTimeFollowing(const rclcpp::Time &current_time, double dt)
  {
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    publishControllerTarget(pos_des, current_time);
    const double raw_yaw_error =
        normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_error =
        std::abs(raw_yaw_error) < heading_error_deadband_ ? 0.0 : raw_yaw_error;
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    const double abs_yaw_error = std::abs(yaw_error);
    if (time_aligning_)
    {
      if (abs_yaw_error <= heading_error_exit_threshold_)
        time_aligning_ = false;
    }
    else if (abs_yaw_error >= heading_error_threshold_)
    {
      time_aligning_ = true;
    }
    if (time_aligning_)
    {
      publishExecutionFrozen(true);
      publishYawOnly(yaw_command, dt);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    publishControllerTarget(pos_des, current_time);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));

    geometry_msgs::msg::Twist command;
    publishBodyCommand(vel_world, yaw_command, command);
    if (exec_time_ >= traj_duration_ &&
        pos_error.norm() < finish_dist_)
    {
      publishImmediateStop();
      return;
    }
    publishFilteredCommand(command, dt);
  }

  /** @brief Periodic controller update and command publication callback. */
  void cmdCallback()
  {
    const auto current_time = now();
    if (have_odom_ && (current_time - last_odom_time_).seconds() > odom_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "body_pose timed out (limit %.2fs); clearing trajectory and stopping",
          odom_timeout_);
      have_odom_ = false;
      clearTrajectory("body_pose timeout", true);
    }
    if (safety_stop_active_)
    {
      publishExecutionFrozen(true);
      publishImmediateStop();
      return;
    }
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishImmediateStop();
      return;
    }
    if (!plannerHeartbeatReady())
    {
      if (!planner_gate_blocked_)
      {
        RCLCPP_ERROR(
            get_logger(),
            "Planner heartbeat lost, disallowed, or does not match trajectory %lld; stopping",
            static_cast<long long>(traj_id_));
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting for a fresh matching planner heartbeat for trajectory %lld",
            static_cast<long long>(traj_id_));
      }
      planner_gate_blocked_ = true;
      publishExecutionFrozen(true);
      publishImmediateStop();
      return;
    }
    if (planner_gate_blocked_)
    {
      RCLCPP_INFO(
          get_logger(),
          "Planner heartbeat matched trajectory %lld; motion enabled",
          static_cast<long long>(traj_id_));
    }
    planner_gate_blocked_ = false;
    double dt = (current_time - last_update_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
    {
      // 调度长时间停顿或 ROS 时间回跳后，旧 limiter/filter 状态已经不能
      // 代表连续控制过程。先安全归零并重置，下一正常周期再从最小速度起步。
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Controller time step %.6fs is invalid; resetting command state", dt);
      last_update_time_ = current_time;
      publishExecutionFrozen(true);
      publishImmediateStop();
      return;
    }
    if (tracking_mode_ == "path_follow")
      trackPathFollowing(current_time, dt);
    else
      trackTimeFollowing(current_time, dt);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr controller_bspline_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr controller_target_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::PlannerHeartbeat>::SharedPtr
      planner_heartbeat_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool safety_stop_active_{false};
  bool planner_gate_blocked_{true};
  bool omni_aligning_{false};
  bool time_aligning_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{-1};
  builtin_interfaces::msg::Time traj_start_time_;
  PlannerHeartbeatGate planner_heartbeat_gate_;
  bool heartbeat_progress_valid_{false};
  std::int64_t heartbeat_progress_traj_id_{-1};
  builtin_interfaces::msg::Time heartbeat_progress_start_time_;
  double heartbeat_progress_time_{0.0};
  double heartbeat_progress_arc_length_{0.0};
  std::string traj_frame_id_;
  std::string odom_frame_id_;
  std::string tracking_mode_{"path_follow"};
  std::string drive_mode_{"pure_pursuit"};
  std::vector<Eigen::Vector3d> sampled_path_;
  std::vector<double> sampled_times_;
  std::vector<double> sampled_arc_lengths_;
  LocalTrajectoryTracker local_trajectory_tracker_;
  PathTrackingController path_tracking_controller_;
  PathCommandGovernor path_command_governor_;
  SlewRateLimiter linear_x_limiter_;
  SlewRateLimiter linear_y_limiter_;
  size_t closest_path_idx_{0};
  double path_progress_s_{0.0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, heading_error_exit_threshold_;
  double heading_slowdown_start_, kp_pos_, kp_yaw_;
  double align_heading_gain_, pp_heading_gain_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double path_sample_dt_, lookahead_dist_, yaw_lookahead_dist_, min_path_speed_;
  double pure_pursuit_speed_{0.20};
  double lateral_error_deadband_{0.04};
  double heading_error_deadband_{0.035};
  double curvature_deadband_{0.04};
  double curvature_speed_gain_{0.60};
  double yaw_rate_reserve_{0.03};
  double max_lateral_acceleration_{0.20};
  double max_linear_acceleration_{0.35};
  double max_linear_deceleration_{0.60};
  double yaw_rate_deadband_{0.025};
  double yaw_filter_time_constant_{0.12};
  double max_yaw_acceleration_{1.0};
  double yaw_reversal_threshold_{0.12};
  double yaw_start_threshold_{0.06};
  double yaw_stop_threshold_{0.03};
  double min_nonzero_yaw_rate_{0.10};
  double min_nonzero_linear_speed_{0.05};
  double yaw_sync_threshold_{0.10};
  double launch_yaw_readiness_ratio_{0.85};
  YawCommandFilter yaw_command_filter_;
  double odom_timeout_{0.30};
  double planner_heartbeat_timeout_{0.40};
  double max_bspline_duration_{60.0};
  std::size_t max_bspline_control_points_{1000};
  double control_rate_{50.0};
  double local_progress_max_advance_{0.75};
  double local_progress_movement_scale_{1.5};
  double local_progress_movement_deadband_{0.01};
  double local_progress_initial_credit_{0.15};
  std::string expected_frame_id_;
};
}  // namespace scan_planner

/** @brief Runs the closed-loop controller. @param argc Argument count. @param argv Argument vector. @return Process exit status. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
