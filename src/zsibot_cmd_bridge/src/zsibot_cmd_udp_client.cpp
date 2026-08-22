// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file zsibot_cmd_udp_client.cpp @brief UDP client for forwarding robot commands. */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
double clampFinite(double value, double limit)
{
  if (!std::isfinite(value))
    return 0.0;
  const double abs_limit = std::max(0.0, std::abs(limit));
  return std::clamp(value, -abs_limit, abs_limit);
}

double applyDeadband(double value, double deadband)
{
  return std::abs(value) < std::abs(deadband) ? 0.0 : value;
}
}  // namespace

namespace zsibot_cmd_bridge
{
/** @brief Sends guarded ROS velocity commands to the on-robot SDK proxy over UDP. */
class ZsiBotCmdUdpClient : public rclcpp::Node
{
public:
  /** @brief Loads UDP and safety parameters, opens the socket, and creates ROS interfaces. */
  ZsiBotCmdUdpClient() : Node("omni_zsibot_cmd_udp_client")
  {
    const bool enable_deprecated_transport =
        declare_parameter<bool>("enable_deprecated_transport", false);
    if (!enable_deprecated_transport)
    {
      throw std::runtime_error(
          "Deprecated ZsiBot UDP transport is disabled. Use the unified robot bridge, "
          "or explicitly set enable_deprecated_transport:=true for an isolated test.");
    }

    proxy_ip_ = declare_parameter<std::string>("proxy_ip", "192.168.234.1");
    proxy_port_ = declare_parameter<int>("proxy_port", 44000);
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    max_vx_ = declare_parameter<double>("max_vx", 0.3);
    max_vy_ = declare_parameter<double>("max_vy", 0.15);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.5);
    deadband_vx_ = declare_parameter<double>("deadband_vx", 0.0);
    deadband_vy_ = declare_parameter<double>("deadband_vy", 0.0);
    deadband_yaw_rate_ = declare_parameter<double>("deadband_yaw_rate", 0.0);
    status_log_period_ = declare_parameter<double>("status_log_period", 2.0);

    if (proxy_port_ <= 0 || proxy_port_ > 65535)
      throw std::runtime_error("proxy_port must be in 1..65535");
    if (publish_rate_ <= 0.0)
      throw std::runtime_error("publish_rate must be positive");
    if (cmd_timeout_ <= 0.0)
      throw std::runtime_error("cmd_timeout must be positive");

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
      throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));

    std::memset(&proxy_addr_, 0, sizeof(proxy_addr_));
    proxy_addr_.sin_family = AF_INET;
    proxy_addr_.sin_port = htons(static_cast<uint16_t>(proxy_port_));
    if (::inet_pton(AF_INET, proxy_ip_.c_str(), &proxy_addr_.sin_addr) != 1)
      throw std::runtime_error("Invalid proxy_ip: " + proxy_ip_);

    last_cmd_time_ = now();
    last_status_log_time_ = now();
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&ZsiBotCmdUdpClient::cmdCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        std::bind(&ZsiBotCmdUdpClient::timerCallback, this));

    RCLCPP_INFO(get_logger(), "ZsiBot UDP client ready: cmd_vel -> %s:%d",
                proxy_ip_.c_str(), proxy_port_);
  }

  /** @brief Sends a final stop packet and closes the UDP socket. */
  ~ZsiBotCmdUdpClient() override
  {
    try
    {
      sendPacket(0.0, 0.0, 0.0);
    }
    catch (...)
    {
    }
    if (socket_fd_ >= 0)
      ::close(socket_fd_);
  }

private:
  /** @brief Stores the latest planner command. @param msg Twist command. */
  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    vx_ = applyDeadband(clampFinite(msg->linear.x, max_vx_), deadband_vx_);
    vy_ = applyDeadband(clampFinite(msg->linear.y, max_vy_), deadband_vy_);
    yaw_rate_ = applyDeadband(clampFinite(msg->angular.z, max_yaw_rate_), deadband_yaw_rate_);
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  /** @brief Applies watchdog behavior and sends one UDP packet. */
  void timerCallback()
  {
    const bool timed_out = !have_cmd_ || (now() - last_cmd_time_).seconds() > cmd_timeout_;
    if (timed_out)
    {
      if (!last_sent_zero_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "cmd_vel timeout; sending zero UDP command");
      sendPacket(0.0, 0.0, 0.0);
      last_sent_zero_ = true;
      maybeLogStatus(true);
      return;
    }

    sendPacket(vx_, vy_, yaw_rate_);
    last_sent_zero_ = vx_ == 0.0 && vy_ == 0.0 && yaw_rate_ == 0.0;
    maybeLogStatus(false);
  }

  /** @brief Serializes and transmits one command. @param vx Forward velocity. @param vy Lateral velocity. @param yaw_rate Yaw rate. */
  void sendPacket(double vx, double vy, double yaw_rate)
  {
    ++seq_;
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(6);
    stream << "ZSIBOT_CMD_V1 " << seq_ << ' ' << vx << ' ' << vy << ' ' << yaw_rate << '\n';
    const std::string packet = stream.str();
    const ssize_t sent = ::sendto(socket_fd_, packet.data(), packet.size(), 0,
                                  reinterpret_cast<const sockaddr*>(&proxy_addr_),
                                  sizeof(proxy_addr_));
    if (sent < 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "sendto(%s:%d) failed: %s", proxy_ip_.c_str(), proxy_port_,
                           std::strerror(errno));
    }
  }

  /** @brief Emits throttled UDP-client diagnostics. @param timed_out Whether the command watchdog expired. */
  void maybeLogStatus(bool timed_out)
  {
    if (status_log_period_ <= 0.0)
      return;
    const auto current_time = now();
    if ((current_time - last_status_log_time_).seconds() < status_log_period_)
      return;
    last_status_log_time_ = current_time;
    RCLCPP_INFO(get_logger(), "UDP cmd=%s seq=%" PRIu64 " vx=%.3f vy=%.3f yaw=%.3f -> %s:%d",
                timed_out ? "timeout/zero" : "active", seq_,
                timed_out ? 0.0 : vx_, timed_out ? 0.0 : vy_, timed_out ? 0.0 : yaw_rate_,
                proxy_ip_.c_str(), proxy_port_);
  }

  int socket_fd_{-1};
  sockaddr_in proxy_addr_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string proxy_ip_;
  int proxy_port_{44000};
  double publish_rate_{50.0};
  double cmd_timeout_{0.3};
  double max_vx_{0.3};
  double max_vy_{0.15};
  double max_yaw_rate_{0.5};
  double deadband_vx_{0.0};
  double deadband_vy_{0.0};
  double deadband_yaw_rate_{0.0};
  double status_log_period_{2.0};
  bool have_cmd_{false};
  bool last_sent_zero_{false};
  uint64_t seq_{0};
  double vx_{0.0};
  double vy_{0.0};
  double yaw_rate_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace zsibot_cmd_bridge

/** @brief Runs the ZsiBot UDP command client. @param argc Argument count. @param argv Argument vector. @return Process exit status. */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<zsibot_cmd_bridge::ZsiBotCmdUdpClient>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("zsibot_cmd_udp_client"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
