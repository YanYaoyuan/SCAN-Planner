// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file zsibot_cmd_bridge.cpp @brief ROS 2 to ZsiBot SDK command bridge. */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "zsibot_cmd_bridge/sdk_owner_lock.hpp"

#if defined(ZSIBOT_MODEL_ZSL_1)
#include "zsl-1/highlevel.h"
namespace zsibot_model = mc_sdk::zsl_1;
#elif defined(ZSIBOT_MODEL_ZSL_1W)
#include "zsl-1w/highlevel.h"
namespace zsibot_model = mc_sdk::zsl_1w;
#else
#error "A ZsiBot model compile definition is required"
#endif

namespace zsibot_cmd_bridge
{
/** @brief Converts ROS body-twist commands into guarded ZsiBot SDK commands. */
class ZsiBotCmdBridge : public rclcpp::Node
{
public:
  /** @brief Loads network, safety, and command-limit parameters and initializes the SDK. */
  ZsiBotCmdBridge() : Node("zsibot_cmd_bridge")
  {
    const bool enable_deprecated_transport =
        declare_parameter<bool>("enable_deprecated_transport", false);
    if (!enable_deprecated_transport)
    {
      throw std::runtime_error(
          "Deprecated vendor SDK bridge is disabled. Use the unified robot bridge, "
          "or explicitly set enable_deprecated_transport:=true for an isolated test.");
    }
    sdk_owner_lock_ = std::make_unique<SdkOwnerLock>("deprecated_zsibot_cmd_bridge");

    local_ip_ = declare_parameter<std::string>("local_ip", "192.168.234.234");
    dog_ip_ = declare_parameter<std::string>("dog_ip", "192.168.234.1");
    local_port_ = declare_parameter<int>("local_port", 43988);
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 1.0);
    deadband_vx_ = declare_parameter<double>("deadband_vx", 0.0);
    deadband_vy_ = declare_parameter<double>("deadband_vy", 0.0);
    deadband_yaw_rate_ = declare_parameter<double>("deadband_yaw_rate", 0.0);
    auto_stand_ = declare_parameter<bool>("auto_stand", true);
    require_standing_before_move_ = declare_parameter<bool>("require_standing_before_move", true);
    standup_check_period_ = declare_parameter<double>("standup_check_period", 0.2);
    standup_retry_period_ = declare_parameter<double>("standup_retry_period", 1.0);
    standup_wait_timeout_ = declare_parameter<double>("standup_wait_timeout", 10.0);
    send_zero_on_start_ = declare_parameter<bool>("send_zero_on_start", true);
    require_connection_ = declare_parameter<bool>("require_connection", false);
    status_log_period_ = declare_parameter<double>("status_log_period", 2.0);
    log_sdk_status_ = declare_parameter<bool>("log_sdk_status", true);

    if (publish_rate_ <= 0.0)
      throw std::runtime_error("publish_rate must be positive");
    if (cmd_timeout_ <= 0.0)
      throw std::runtime_error("cmd_timeout must be positive");
    if (standup_check_period_ <= 0.0)
      throw std::runtime_error("standup_check_period must be positive");
    if (standup_retry_period_ <= 0.0)
      throw std::runtime_error("standup_retry_period must be positive");
    if (standup_wait_timeout_ <= 0.0)
      throw std::runtime_error("standup_wait_timeout must be positive");

    RCLCPP_INFO(get_logger(), "Connecting ZsiBot SDK: local %s:%d -> dog %s",
                local_ip_.c_str(), local_port_, dog_ip_.c_str());
    highlevel_.initRobot(local_ip_, local_port_, dog_ip_);

    if (require_connection_ && !highlevel_.checkConnect())
      throw std::runtime_error("ZsiBot SDK checkConnect() failed");

    if (auto_stand_)
    {
      const uint32_t ret = highlevel_.standUp();
      RCLCPP_INFO(get_logger(), "standUp() returned %u", ret);
      if (require_standing_before_move_)
      {
        waiting_for_standup_ = true;
        standup_start_time_ = now();
        last_standup_request_time_ = now();
        last_standup_check_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        RCLCPP_INFO(get_logger(), "Waiting for standUp state before sending move commands");
      }
    }
    if (send_zero_on_start_ && !waiting_for_standup_)
      sendMove(0.0, 0.0, 0.0);

    last_cmd_time_ = now();
    last_status_log_time_ = now();
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&ZsiBotCmdBridge::cmdCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        std::bind(&ZsiBotCmdBridge::timerCallback, this));

    RCLCPP_INFO(get_logger(), "ZsiBot cmd bridge ready");
  }

  /** @brief Sends a final stop command and releases SDK resources. */
  ~ZsiBotCmdBridge() override
  {
    try
    {
      if (!waiting_for_standup_)
        sendMove(0.0, 0.0, 0.0);
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Failed to send zero command during shutdown: %s", e.what());
    }
  }

private:
  /** @brief Sanitizes and limits one command component. @param value Requested value. @param limit Absolute limit. @return Safe limited value. */
  static double clampFinite(double value, double limit)
  {
    if (!std::isfinite(value))
      return 0.0;
    const double abs_limit = std::max(0.0, std::abs(limit));
    return std::clamp(value, -abs_limit, abs_limit);
  }

  /** @brief Zeros a small command component. @param value Requested value. @param deadband Deadband threshold. @return Filtered value. */
  static double applyDeadband(double value, double deadband)
  {
    return std::abs(value) < std::abs(deadband) ? 0.0 : value;
  }

  /** @brief Receives the latest planner velocity command. @param msg Twist command. */
  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    vx_ = applyDeadband(clampFinite(msg->linear.x, max_vx_), deadband_vx_);
    vy_ = applyDeadband(clampFinite(msg->linear.y, max_vy_), deadband_vy_);
    yaw_rate_ = applyDeadband(clampFinite(msg->angular.z, max_yaw_rate_), deadband_yaw_rate_);
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  /** @brief Enforces timeout/stand-up state and sends the current command. */
  void timerCallback()
  {
    if (waiting_for_standup_)
    {
      handleStandupWait();
      maybeLogStatus(true);
      return;
    }

    const bool timed_out = !have_cmd_ || (now() - last_cmd_time_).seconds() > cmd_timeout_;
    if (timed_out)
    {
      if (!last_sent_zero_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "cmd_vel timeout; sending zero velocity");
      sendMove(0.0, 0.0, 0.0);
      last_sent_zero_ = true;
      maybeLogStatus(true);
      return;
    }

    sendMove(vx_, vy_, yaw_rate_);
    last_sent_zero_ = vx_ == 0.0 && vy_ == 0.0 && yaw_rate_ == 0.0;
    maybeLogStatus(false);
  }

  /** @brief Sends one high-level SDK movement request. @param vx Forward velocity. @param vy Lateral velocity. @param yaw_rate Yaw rate. */
  void sendMove(double vx, double vy, double yaw_rate)
  {
    const uint32_t ret = highlevel_.move(
        static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(yaw_rate));
    if (ret != 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "HighLevel::move returned %u", ret);
    }
  }

  /** @brief Runs the optional stand-up delay state. */
  void handleStandupWait()
  {
    const auto current_time = now();
    if ((current_time - last_standup_check_time_).seconds() < standup_check_period_)
      return;
    last_standup_check_time_ = current_time;

    uint32_t ctrlmode = 0;
    try
    {
      ctrlmode = highlevel_.getCurrentCtrlmode();
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Failed to query ctrlmode while waiting for standUp: %s", e.what());
      return;
    }

    if (ctrlmode == 1 || ctrlmode == 3)
    {
      waiting_for_standup_ = false;
      RCLCPP_INFO(get_logger(), "standUp state confirmed: ctrlmode=%u; move commands enabled",
                  ctrlmode);
      if (send_zero_on_start_)
      {
        sendMove(0.0, 0.0, 0.0);
        last_sent_zero_ = true;
      }
      return;
    }

    const double wait_seconds = (current_time - standup_start_time_).seconds();
    if ((current_time - last_standup_request_time_).seconds() >= standup_retry_period_)
    {
      last_standup_request_time_ = current_time;
      const uint32_t ret = highlevel_.standUp();
      RCLCPP_INFO(get_logger(), "Retrying standUp() while ctrlmode=%u; returned %u", ctrlmode, ret);
    }

    if (wait_seconds > standup_wait_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Still waiting for standUp state after %.1fs: ctrlmode=%u. Move commands remain blocked.",
          wait_seconds, ctrlmode);
    }
  }

  /** @brief Emits throttled bridge diagnostics. @param timed_out Whether the command watchdog expired. */
  void maybeLogStatus(bool timed_out)
  {
    if (!log_sdk_status_ || status_log_period_ <= 0.0)
      return;
    const auto current_time = now();
    if ((current_time - last_status_log_time_).seconds() < status_log_period_)
      return;
    last_status_log_time_ = current_time;

    bool connected = false;
    uint32_t battery = 0;
    uint32_t ctrlmode = 0;
    try
    {
      connected = highlevel_.checkConnect();
      battery = highlevel_.getBatteryPower();
      ctrlmode = highlevel_.getCurrentCtrlmode();
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Failed to query SDK status: %s", e.what());
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "SDK status: connected=%s battery=%u%% ctrlmode=%u cmd=%s vx=%.3f vy=%.3f yaw=%.3f",
        connected ? "true" : "false", battery, ctrlmode,
        timed_out ? "timeout/zero" : "active", timed_out ? 0.0 : vx_,
        timed_out ? 0.0 : vy_, timed_out ? 0.0 : yaw_rate_);
  }

  std::unique_ptr<SdkOwnerLock> sdk_owner_lock_;
  zsibot_model::HighLevel highlevel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string local_ip_;
  std::string dog_ip_;
  int local_port_{43988};
  double publish_rate_{50.0};
  double cmd_timeout_{0.3};
  double max_vx_{0.75};
  double max_vy_{0.35};
  double max_yaw_rate_{1.0};
  double deadband_vx_{0.0};
  double deadband_vy_{0.0};
  double deadband_yaw_rate_{0.0};
  bool auto_stand_{true};
  bool require_standing_before_move_{true};
  double standup_check_period_{0.2};
  double standup_retry_period_{1.0};
  double standup_wait_timeout_{10.0};
  bool send_zero_on_start_{true};
  bool require_connection_{false};
  double status_log_period_{2.0};
  bool log_sdk_status_{true};

  bool have_cmd_{false};
  bool waiting_for_standup_{false};
  bool last_sent_zero_{false};
  double vx_{0.0};
  double vy_{0.0};
  double yaw_rate_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time standup_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_standup_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_standup_check_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace zsibot_cmd_bridge

/** @brief Runs the direct ZsiBot SDK bridge. @param argc Argument count. @param argv Argument vector. @return Process exit status. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<zsibot_cmd_bridge::ZsiBotCmdBridge>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("zsibot_cmd_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
