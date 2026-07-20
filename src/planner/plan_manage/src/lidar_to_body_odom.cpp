#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace scan_planner
{
class LidarToBodyOdom : public rclcpp::Node
{
public:
  LidarToBodyOdom() : Node("lidar_to_body_odom")
  {
    body_frame_id_ = declare_parameter<std::string>("body_frame_id", "base_link");
    sensor_frame_id_ = declare_parameter<std::string>("sensor_frame_id", "livox_frame");
    world_frame_id_ = declare_parameter<std::string>("world_frame_id", "");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    transform_twist_ = declare_parameter<bool>("transform_twist", false);
    status_log_period_ = declare_parameter<double>("status_log_period", 2.0);

    const double x = declare_parameter<double>("body_to_sensor.x", 0.0);
    const double y = declare_parameter<double>("body_to_sensor.y", 0.0);
    const double z = declare_parameter<double>("body_to_sensor.z", 0.0);
    const double roll = declare_parameter<double>("body_to_sensor.roll", 0.0);
    const double pitch = declare_parameter<double>("body_to_sensor.pitch", 0.0);
    const double yaw = declare_parameter<double>("body_to_sensor.yaw", 0.0);
    body_to_sensor_t_ = Eigen::Vector3d(x, y, z);
    body_to_sensor_r_ = rpyToRotation(roll, pitch, yaw);

    body_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_odom", rclcpp::SensorDataQoS());
    sensor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "sensor_odom", rclcpp::SensorDataQoS(),
        std::bind(&LidarToBodyOdom::odomCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    last_status_log_time_ = now();

    RCLCPP_INFO(
        get_logger(),
        "lidar_to_body_odom ready: sensor frame '%s' -> body frame '%s', publish_tf=%s",
        sensor_frame_id_.c_str(), body_frame_id_.c_str(), publish_tf_ ? "true" : "false");
  }

private:
  static Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw)
  {
    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  }

  static bool validQuaternion(const geometry_msgs::msg::Quaternion& q)
  {
    const double norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return std::isfinite(norm2) && norm2 > 1e-12;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!msg)
      return;
    if (!validQuaternion(msg->pose.pose.orientation))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Ignoring sensor odom with invalid orientation");
      return;
    }
    if (!sensor_frame_id_.empty() && !msg->child_frame_id.empty() &&
        msg->child_frame_id != sensor_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sensor_odom child_frame_id is '%s', expected '%s'",
          msg->child_frame_id.c_str(), sensor_frame_id_.c_str());
    }

    Eigen::Quaterniond q_ws(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    q_ws.normalize();
    const Eigen::Matrix3d sensor_to_world_r = q_ws.toRotationMatrix();
    const Eigen::Vector3d sensor_pos(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    const Eigen::Matrix3d body_to_world_r = sensor_to_world_r * body_to_sensor_r_.transpose();
    const Eigen::Vector3d body_pos = sensor_pos - body_to_world_r * body_to_sensor_t_;
    Eigen::Quaterniond q_wb(body_to_world_r);
    q_wb.normalize();

    nav_msgs::msg::Odometry body_odom = *msg;
    body_odom.header.frame_id = world_frame_id_.empty() ? msg->header.frame_id : world_frame_id_;
    body_odom.child_frame_id = body_frame_id_;
    body_odom.pose.pose.position.x = body_pos.x();
    body_odom.pose.pose.position.y = body_pos.y();
    body_odom.pose.pose.position.z = body_pos.z();
    body_odom.pose.pose.orientation.x = q_wb.x();
    body_odom.pose.pose.orientation.y = q_wb.y();
    body_odom.pose.pose.orientation.z = q_wb.z();
    body_odom.pose.pose.orientation.w = q_wb.w();

    if (transform_twist_)
    {
      const Eigen::Vector3d linear_sensor(
          msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
      const Eigen::Vector3d angular_sensor(
          msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
      const Eigen::Vector3d linear_body = body_to_sensor_r_ * linear_sensor;
      const Eigen::Vector3d angular_body = body_to_sensor_r_ * angular_sensor;
      body_odom.twist.twist.linear.x = linear_body.x();
      body_odom.twist.twist.linear.y = linear_body.y();
      body_odom.twist.twist.linear.z = linear_body.z();
      body_odom.twist.twist.angular.x = angular_body.x();
      body_odom.twist.twist.angular.y = angular_body.y();
      body_odom.twist.twist.angular.z = angular_body.z();
    }

    body_pub_->publish(body_odom);
    if (publish_tf_)
      publishTransform(body_odom);
    maybeLogStatus(body_odom);
  }

  void publishTransform(const nav_msgs::msg::Odometry& body_odom)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = body_odom.header;
    transform.child_frame_id = body_odom.child_frame_id;
    transform.transform.translation.x = body_odom.pose.pose.position.x;
    transform.transform.translation.y = body_odom.pose.pose.position.y;
    transform.transform.translation.z = body_odom.pose.pose.position.z;
    transform.transform.rotation = body_odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void maybeLogStatus(const nav_msgs::msg::Odometry& body_odom)
  {
    if (status_log_period_ <= 0.0)
      return;
    const auto current_time = now();
    if ((current_time - last_status_log_time_).seconds() < status_log_period_)
      return;
    last_status_log_time_ = current_time;
    RCLCPP_INFO(
        get_logger(),
        "body_odom: %s -> %s pos=[%.3f %.3f %.3f]",
        body_odom.header.frame_id.c_str(), body_odom.child_frame_id.c_str(),
        body_odom.pose.pose.position.x, body_odom.pose.pose.position.y,
        body_odom.pose.pose.position.z);
  }

  std::string body_frame_id_;
  std::string sensor_frame_id_;
  std::string world_frame_id_;
  bool publish_tf_{false};
  bool transform_twist_{false};
  double status_log_period_{2.0};
  Eigen::Vector3d body_to_sensor_t_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d body_to_sensor_r_{Eigen::Matrix3d::Identity()};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace scan_planner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::LidarToBodyOdom>());
  rclcpp::shutdown();
  return 0;
}
