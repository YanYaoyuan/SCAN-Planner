#include <algorithm>
#include <cmath>
#include <cstdint>
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

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
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

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  static Eigen::Vector2d normalizedOrZero(const Eigen::Vector2d &value)
  {
    const double norm = value.norm();
    if (norm < 1e-6)
      return Eigen::Vector2d::Zero();
    return value / norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void clearTrajectory(const std::string &reason)
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
    publishExecutionFrozen(false);
    publishStop();
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      clearTrajectory("empty/invalid B-spline");
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

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (odom_frame_id_ != msg->header.frame_id)
    {
      odom_frame_id_ = msg->header.frame_id;
      RCLCPP_INFO(get_logger(), "Controller odom frame is '%s', child_frame_id='%s'",
                  odom_frame_id_.c_str(), msg->child_frame_id.c_str());
    }
    warnFrameMismatchIfNeeded();
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void warnFrameMismatchIfNeeded()
  {
    if (traj_frame_id_.empty() || odom_frame_id_.empty() || traj_frame_id_ == odom_frame_id_)
      return;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "B-spline frame '%s' differs from odom frame '%s'. Check real_grid_frame_id, "
        "body_odom_world_frame_id, and goal_frame_id; controller still compares numeric "
        "coordinates directly.",
        traj_frame_id_.c_str(), odom_frame_id_.c_str());
  }

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
  }

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

  size_t findClosestPathIndex()
  {
    if (sampled_path_.empty())
      return 0;

    const size_t search_back = 5;
    const size_t start_idx = closest_path_idx_ > search_back ? closest_path_idx_ - search_back : 0;
    double best_dist = std::numeric_limits<double>::max();
    size_t best_idx = closest_path_idx_;
    for (size_t i = start_idx; i < sampled_path_.size(); ++i)
    {
      const double dist = (sampled_path_[i].head<2>() - odom_pos_.head<2>()).squaredNorm();
      if (dist < best_dist)
      {
        best_dist = dist;
        best_idx = i;
      }
    }
    closest_path_idx_ = best_idx;
    return closest_path_idx_;
  }

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

  double pathSpeedAt(size_t idx) const
  {
    if (traj_.size() < 2 || sampled_times_.empty())
      return min_path_speed_;
    const double t = sampled_times_[std::min(idx, sampled_times_.size() - 1)];
    const double speed = traj_[1].evaluateDeBoorT(t).head<2>().norm();
    return std::min(max_vx_, std::max(min_path_speed_, speed));
  }

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
      yaw_rate = vx * 2.0 * local_y / (lookahead * lookahead);
    }

    command.linear.x = std::clamp(vx, 0.0, max_vx_);
    command.linear.y = 0.0;
    command.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
  }

  void trackPathFollowing(const rclcpp::Time &current_time, double dt)
  {
    if (sampled_path_.empty())
    {
      trackTimeFollowing(current_time, dt);
      return;
    }

    const size_t closest_idx = findClosestPathIndex();
    const size_t target_idx = indexAtArcDistance(closest_idx, lookahead_dist_);
    const size_t yaw_idx = indexAtArcDistance(closest_idx, yaw_lookahead_dist_);
    const Eigen::Vector3d pos_des = sampled_path_[target_idx];
    publishControllerTarget(pos_des, current_time);

    Eigen::Vector2d to_target = pos_des.head<2>() - odom_pos_.head<2>();
    Eigen::Vector2d path_dir = sampled_path_[yaw_idx].head<2>() - sampled_path_[closest_idx].head<2>();
    if (path_dir.squaredNorm() < 1e-4)
      path_dir = to_target;
    if (path_dir.squaredNorm() < 1e-4 && traj_.size() > 1)
      path_dir = traj_[1].evaluateDeBoorT(sampled_times_[closest_idx]).head<2>();

    const double desired_yaw = path_dir.squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(path_dir.y(), path_dir.x());
    const double yaw_error = normalizeAngle(desired_yaw - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
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

    if (target_idx + 1 >= sampled_path_.size() && to_end.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();

    exec_time_ = sampled_times_[closest_idx];
    last_update_time_ = current_time;
    cmd_vel_pub_->publish(command);
  }

  void trackTimeFollowing(const rclcpp::Time &current_time, double dt)
  {
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    publishControllerTarget(pos_des, current_time);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
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
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr controller_target_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
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
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double path_sample_dt_, lookahead_dist_, yaw_lookahead_dist_, min_path_speed_;
  double pure_pursuit_speed_{0.20};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
