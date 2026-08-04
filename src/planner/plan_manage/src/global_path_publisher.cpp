// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file global_path_publisher.cpp @brief Global reference-path publisher. */

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace scan_planner
{
/** @brief Builds a global-frame reference route from RViz goals or configured waypoints. */
class GlobalPathPublisher : public rclcpp::Node
{
public:
  /** @brief Loads path parameters and creates ROS interfaces. */
  GlobalPathPublisher() : Node("global_path_publisher")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "lio_map");
    body_frame_id_ =
        declare_parameter<std::string>("body_frame_id", "scan_base_link");
    path_spacing_ = declare_parameter<double>("path_spacing", 0.25);
    goal_transform_timeout_ =
        declare_parameter<double>("goal_transform_timeout", 0.2);
    rcl_interfaces::msg::ParameterDescriptor waypoints_descriptor;
    waypoints_descriptor.type =
        rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    configured_waypoints_ =
        declare_parameter<std::vector<double>>(
            "waypoints", std::vector<double>{}, waypoints_descriptor);

    if (frame_id_.empty() || body_frame_id_.empty())
      throw std::invalid_argument("frame_id and body_frame_id must not be empty");
    if (!std::isfinite(path_spacing_) || path_spacing_ <= 0.0)
      throw std::invalid_argument("path_spacing must be finite and positive");
    if (!std::isfinite(goal_transform_timeout_) ||
        goal_transform_timeout_ < 0.0)
      throw std::invalid_argument(
          "goal_transform_timeout must be finite and non-negative");
    if (configured_waypoints_.size() % 3 != 0)
      throw std::invalid_argument("waypoints must contain x, y, z triples");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose",
        rclcpp::SensorDataQoS(),
        std::bind(
            &GlobalPathPublisher::odometryCallback,
            this,
            std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal",
        rclcpp::QoS(10).reliable(),
        std::bind(
            &GlobalPathPublisher::goalCallback,
            this,
            std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "global_path",
        rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
        get_logger(),
        "Global path publisher ready in frame '%s'",
        frame_id_.c_str());
  }

private:
  /** @brief Validates a 3D point. @param position Point to test. @return True when finite. */
  static bool finitePosition(const geometry_msgs::msg::Point &position)
  {
    return std::isfinite(position.x) &&
           std::isfinite(position.y) &&
           std::isfinite(position.z);
  }

  /** @brief Updates the current global body pose. @param message Body odometry. */
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    if (message->header.frame_id != frame_id_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Rejecting body_pose in frame '%s'; expected '%s'",
          message->header.frame_id.c_str(),
          frame_id_.c_str());
      return;
    }
    if (message->child_frame_id != body_frame_id_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Rejecting body_pose child '%s'; expected '%s'",
          message->child_frame_id.c_str(),
          body_frame_id_.c_str());
      return;
    }
    if (!finitePosition(message->pose.pose.position))
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Rejecting body_pose with non-finite position");
      return;
    }

    current_pose_.header = message->header;
    current_pose_.pose = message->pose.pose;
    have_pose_ = true;

    if (!configured_path_published_ && !configured_waypoints_.empty())
    {
      configured_path_published_ = publishConfiguredPath();
    }
    if (have_pending_goal_)
    {
      have_pending_goal_ = false;
      goalCallback(
          std::make_shared<geometry_msgs::msg::PoseStamped>(pending_goal_));
    }
  }

  /** @brief Transforms a goal into the configured global frame. @param input Input goal. @param[out] output Transformed goal. @return True on success. */
  bool transformGoal(
      const geometry_msgs::msg::PoseStamped &input,
      geometry_msgs::msg::PoseStamped &output)
  {
    if (input.header.frame_id.empty())
    {
      RCLCPP_ERROR(get_logger(), "Rejecting goal with an empty frame_id");
      return false;
    }
    if (!finitePosition(input.pose.position))
    {
      RCLCPP_ERROR(get_logger(), "Rejecting goal with non-finite position");
      return false;
    }
    if (input.header.frame_id == frame_id_)
    {
      output = input;
      return true;
    }

    try
    {
      output = tf_buffer_->transform(
          input,
          frame_id_,
          tf2::durationFromSec(goal_transform_timeout_));
      return true;
    }
    catch (const tf2::TransformException &exception)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Cannot transform goal from '%s' to '%s': %s",
          input.header.frame_id.c_str(),
          frame_id_.c_str(),
          exception.what());
      return false;
    }
  }

  /** @brief Receives an RViz goal and publishes an interpolated path. @param message Goal pose. */
  void goalCallback(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
  {
    if (!have_pose_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Caching goal until the first valid global body pose arrives");
      pending_goal_ = *message;
      have_pending_goal_ = true;
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    if (!transformGoal(*message, goal))
      return;

    // RViz 2D Goal commonly publishes z=0. The route represents the body
    // center, so keep the current body height for every interpolated point.
    goal.pose.position.z = current_pose_.pose.position.z;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    appendSegment(current_pose_.pose.position, goal.pose.position, path);
    publishPath(path, "goal");
  }

  /** @brief Publishes the parameter-defined waypoint route. @return True when valid and published. */
  bool publishConfiguredPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;

    geometry_msgs::msg::Point previous = current_pose_.pose.position;
    for (std::size_t i = 0; i < configured_waypoints_.size(); i += 3)
    {
      geometry_msgs::msg::Point waypoint;
      waypoint.x = configured_waypoints_[i];
      waypoint.y = configured_waypoints_[i + 1];
      waypoint.z = configured_waypoints_[i + 2];
      if (!finitePosition(waypoint))
      {
        RCLCPP_ERROR(
            get_logger(),
            "Configured waypoint %zu contains a non-finite value",
            i / 3);
        return false;
      }
      appendSegment(previous, waypoint, path);
      previous = waypoint;
    }

    publishPath(path, "configured waypoints");
    return true;
  }

  /** @brief Appends evenly spaced samples for one route segment. @param start Segment start. @param end Segment end. @param[in,out] path Destination path. */
  void appendSegment(
      const geometry_msgs::msg::Point &start,
      const geometry_msgs::msg::Point &end,
      nav_msgs::msg::Path &path) const
  {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const std::size_t sample_count = std::max<std::size_t>(
        1,
        static_cast<std::size_t>(std::ceil(distance / path_spacing_)));
    const double yaw =
        std::hypot(dx, dy) > 1.0e-9 ? std::atan2(dy, dx) : 0.0;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    orientation.normalize();

    const std::size_t first_sample = path.poses.empty() ? 0 : 1;
    for (std::size_t i = first_sample; i <= sample_count; ++i)
    {
      const double ratio =
          static_cast<double>(i) / static_cast<double>(sample_count);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = start.x + ratio * dx;
      pose.pose.position.y = start.y + ratio * dy;
      pose.pose.position.z = start.z + ratio * dz;
      pose.pose.orientation = tf2::toMsg(orientation);
      path.poses.push_back(pose);
    }
  }

  /** @brief Validates and publishes a completed route. @param[in,out] path Path message. @param source Diagnostic source label. */
  void publishPath(nav_msgs::msg::Path &path, const char *source)
  {
    if (path.poses.size() < 2)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Not publishing %s path because it has fewer than two poses",
          source);
      return;
    }
    for (auto &pose : path.poses)
      pose.header = path.header;
    path_pub_->publish(path);
    RCLCPP_INFO(
        get_logger(),
        "Published %s global path with %zu poses in '%s'",
        source,
        path.poses.size(),
        frame_id_.c_str());
  }

  std::string frame_id_;
  std::string body_frame_id_;
  double path_spacing_{0.25};
  double goal_transform_timeout_{0.2};
  std::vector<double> configured_waypoints_;
  bool have_pose_{false};
  bool have_pending_goal_{false};
  bool configured_path_published_{false};
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped pending_goal_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};
}  // namespace scan_planner

/** @brief Runs the global path publisher. @param argc Argument count. @param argv Argument vector. @return Process exit status. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<scan_planner::GlobalPathPublisher>());
  }
  catch (const std::exception &exception)
  {
    RCLCPP_FATAL(
        rclcpp::get_logger("global_path_publisher"),
        "%s",
        exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
