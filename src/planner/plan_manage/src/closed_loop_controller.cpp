// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file closed_loop_controller.cpp @brief Closed-loop B-spline tracking controller. */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/yaw_command_filter.h"

namespace scan_planner
{
/** @brief Tracks local B-splines using odometry feedback and publishes body velocity commands. */
class ClosedLoopController : public rclcpp::Node
{
public:
  /** @brief Loads controller parameters and creates ROS interfaces. */
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
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
    yaw_rate_deadband_ =
        std::max(0.0, declare_parameter<double>("yaw_rate_deadband", 0.03));
    yaw_filter_time_constant_ =
        std::max(0.0, declare_parameter<double>("yaw_filter_time_constant", 0.20));
    max_yaw_acceleration_ =
        std::max(0.01, declare_parameter<double>("max_yaw_acceleration", 1.0));
    yaw_reversal_threshold_ =
        std::max(
            yaw_rate_deadband_,
            declare_parameter<double>("yaw_reversal_threshold", 0.15));
    yaw_start_threshold_ =
        std::max(
            yaw_rate_deadband_,
            declare_parameter<double>("yaw_start_threshold", 0.12));
    yaw_stop_threshold_ =
        std::clamp(
            declare_parameter<double>("yaw_stop_threshold", 0.06),
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
    odom_timeout_ = std::max(0.05, declare_parameter<double>("odom_timeout", 0.30));
    expected_frame_id_ = declare_parameter<std::string>("expected_frame_id", "");
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

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
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
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
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
    const double raw_yaw_rate =
        std::clamp(command.angular.z, -max_vyaw_, max_vyaw_);
    command.angular.z = std::clamp(
        yaw_command_filter_.update(raw_yaw_rate, dt),
        -max_vyaw_,
        max_vyaw_);
    cmd_vel_pub_->publish(command);
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
    traj_duration_ = 0.0;
    exec_time_ = 0.0;
    closest_path_idx_ = 0;
    path_progress_s_ = 0.0;
    safety_stop_active_ = safety_stop;
    publishExecutionFrozen(safety_stop_active_);
    publishImmediateStop();
  }

  /** @brief Receives a new local B-spline. @param msg B-spline message. */
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      clearTrajectory("empty/invalid B-spline", safety_stop_active_);
      return;
    }
    if (msg->header.frame_id.empty())
    {
      clearTrajectory("B-spline has an empty frame_id", true);
      return;
    }
    const std::string required_frame =
        expected_frame_id_.empty() ? odom_frame_id_ : expected_frame_id_;
    if (!required_frame.empty() && msg->header.frame_id != required_frame)
    {
      RCLCPP_ERROR(
          get_logger(), "Rejecting B-spline in frame '%s'; expected '%s'",
          msg->header.frame_id.c_str(), required_frame.c_str());
      clearTrajectory("B-spline frame mismatch", true);
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
    traj_id_ = msg->traj_id;
    traj_frame_id_ = msg->header.frame_id;
    exec_time_ = 0.0;
    closest_path_idx_ = 0;
    path_progress_s_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    sampleControllerPath();
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
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
    last_odom_time_ = now();
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

  /** @brief Samples the active B-spline for path-following control. */
  void sampleControllerPath()
  {
    sampled_path_.clear();
    sampled_times_.clear();
    sampled_arc_lengths_.clear();

    if (traj_.empty() || traj_duration_ < 1e-6)
      return;

    const int sample_num =
        std::max(2, static_cast<int>(std::ceil(traj_duration_ / path_sample_dt_)) + 1);
    sampled_path_.reserve(sample_num);
    sampled_times_.reserve(sample_num);
    sampled_arc_lengths_.reserve(sample_num);

    double arc_length = 0.0;
    for (int i = 0; i < sample_num; ++i)
    {
      const double t = traj_duration_ * static_cast<double>(i) /
          static_cast<double>(sample_num - 1);
      const Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(t);
      if (!sampled_path_.empty())
      {
        arc_length += (pos.head<2>() - sampled_path_.back().head<2>()).norm();
      }
      sampled_path_.push_back(pos);
      sampled_times_.push_back(t);
      sampled_arc_lengths_.push_back(arc_length);
    }
  }

  /** @brief Finds the nearest forward path sample. @return Closest sample index. */
  size_t findClosestPathIndex()
  {
    if (sampled_path_.empty())
      return 0;

    const size_t search_back = 2;
    const size_t start_idx = closest_path_idx_ > search_back ? closest_path_idx_ - search_back : 0;
    const double max_forward_search = std::max(1.0, 3.0 * lookahead_dist_);
    double best_dist = std::numeric_limits<double>::max();
    size_t best_idx = closest_path_idx_;
    for (size_t i = start_idx; i < sampled_path_.size(); ++i)
    {
      if (i > closest_path_idx_ &&
          sampled_arc_lengths_[i] - sampled_arc_lengths_[closest_path_idx_] >
              max_forward_search)
      {
        break;
      }
      const double dist = (sampled_path_[i].head<2>() - odom_pos_.head<2>()).squaredNorm();
      if (dist < best_dist)
      {
        best_dist = dist;
        best_idx = i;
      }
    }
    closest_path_idx_ = std::max(closest_path_idx_, best_idx);
    path_progress_s_ = std::max(path_progress_s_, sampled_arc_lengths_[closest_path_idx_]);
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

  /** @brief Advances along sampled path by arc length. @param start_idx Starting sample index. @param distance Desired forward distance. @return Target sample index. */
  size_t indexAtArcDistance(size_t start_idx, double distance) const
  {
    if (sampled_arc_lengths_.empty())
      return 0;
    const double target_s = sampled_arc_lengths_[std::min(start_idx, sampled_arc_lengths_.size() - 1)] +
        std::max(0.0, distance);
    for (size_t i = start_idx; i < sampled_arc_lengths_.size(); ++i)
    {
      if (sampled_arc_lengths_[i] >= target_s)
        return i;
    }
    return sampled_arc_lengths_.size() - 1;
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

  /** @brief Computes a pure-pursuit command. @param to_target_world Vector to lookahead target. @param target_is_end Whether the target is the final sample. @param dist_to_end Distance to the local trajectory end. @param[out] command Raw body command. */
  void publishPurePursuitCommand(const Eigen::Vector2d &to_target_world,
                                 bool target_is_end,
                                 double dist_to_end,
                                 geometry_msgs::msg::Twist &command) const
  {
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    const double local_x = c * to_target_world.x() + s * to_target_world.y();
    const double local_y = -s * to_target_world.x() + c * to_target_world.y();
    const double lookahead = std::max(0.05, std::hypot(local_x, local_y));
    double vx = std::min(max_vx_, pure_pursuit_speed_);
    double yaw_rate = 0.0;

    if (target_is_end)
      vx = std::min(vx, std::max(min_path_speed_, dist_to_end));

    if (local_x < 0.02)
    {
      vx = 0.0;
      yaw_rate = kp_yaw_ * std::atan2(local_y, local_x);
    }
    else
    {
      const double filtered_local_y =
          std::abs(local_y) < lateral_error_deadband_ ? 0.0 : local_y;
      double curvature =
          2.0 * filtered_local_y / (lookahead * lookahead);
      if (std::abs(curvature) < curvature_deadband_)
        curvature = 0.0;
      yaw_rate = vx * curvature;
    }

    command.linear.x = std::clamp(vx, 0.0, max_vx_);
    command.linear.y = 0.0;
    command.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
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
    const size_t target_idx = indexAtArcDistance(closest_idx, lookahead_dist_);
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
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishYawOnly(yaw_command, dt);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    geometry_msgs::msg::Twist command;
    const Eigen::Vector2d to_end = sampled_path_.back().head<2>() - odom_pos_.head<2>();
    const bool target_is_end = target_idx + 1 >= sampled_path_.size();
    if (drive_mode_ == "pure_pursuit")
    {
      publishPurePursuitCommand(to_target, target_is_end, to_end.norm(), command);
    }
    else
    {
      const Eigen::Vector2d vel_ff = normalizedOrZero(path_dir) * pathSpeedAt(closest_idx);
      const Eigen::Vector2d vel_world =
          clampNorm(vel_ff + kp_pos_ * to_target, std::max(max_vx_, max_vy_));
      publishBodyCommand(vel_world, yaw_command, command);
    }

    exec_time_ = sampled_times_[closest_idx];
    last_update_time_ = current_time;
    if (target_idx + 1 >= sampled_path_.size() &&
        to_end.norm() < finish_dist_)
    {
      publishImmediateStop();
      return;
    }
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
    if (std::abs(yaw_error) > heading_error_threshold_)
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
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool safety_stop_active_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  std::string traj_frame_id_;
  std::string odom_frame_id_;
  std::string tracking_mode_{"path_follow"};
  std::string drive_mode_{"pure_pursuit"};
  std::vector<Eigen::Vector3d> sampled_path_;
  std::vector<double> sampled_times_;
  std::vector<double> sampled_arc_lengths_;
  size_t closest_path_idx_{0};
  double path_progress_s_{0.0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double path_sample_dt_, lookahead_dist_, yaw_lookahead_dist_, min_path_speed_;
  double pure_pursuit_speed_{0.20};
  double lateral_error_deadband_{0.04};
  double heading_error_deadband_{0.035};
  double curvature_deadband_{0.04};
  double yaw_rate_deadband_{0.03};
  double yaw_filter_time_constant_{0.20};
  double max_yaw_acceleration_{1.0};
  double yaw_reversal_threshold_{0.15};
  double yaw_start_threshold_{0.12};
  double yaw_stop_threshold_{0.06};
  double min_nonzero_yaw_rate_{0.10};
  YawCommandFilter yaw_command_filter_;
  double odom_timeout_{0.30};
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
