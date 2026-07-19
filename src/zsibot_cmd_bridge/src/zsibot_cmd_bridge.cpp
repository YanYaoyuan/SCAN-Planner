#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

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
class ZsiBotCmdBridge : public rclcpp::Node
{
public:
  ZsiBotCmdBridge() : Node("zsibot_cmd_bridge")
  {
    local_ip_ = declare_parameter<std::string>("local_ip", "192.168.168.10");
    dog_ip_ = declare_parameter<std::string>("dog_ip", "192.168.168.168");
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
    send_zero_on_start_ = declare_parameter<bool>("send_zero_on_start", true);
    require_connection_ = declare_parameter<bool>("require_connection", false);
    status_log_period_ = declare_parameter<double>("status_log_period", 2.0);
    log_sdk_status_ = declare_parameter<bool>("log_sdk_status", true);

    if (publish_rate_ <= 0.0)
      throw std::runtime_error("publish_rate must be positive");
    if (cmd_timeout_ <= 0.0)
      throw std::runtime_error("cmd_timeout must be positive");

    RCLCPP_INFO(get_logger(), "Connecting ZsiBot SDK: local %s:%d -> dog %s",
                local_ip_.c_str(), local_port_, dog_ip_.c_str());
    highlevel_.initRobot(local_ip_, local_port_, dog_ip_);

    if (require_connection_ && !highlevel_.checkConnect())
      throw std::runtime_error("ZsiBot SDK checkConnect() failed");

    if (auto_stand_)
    {
      const uint32_t ret = highlevel_.standUp();
      RCLCPP_INFO(get_logger(), "standUp() returned %u", ret);
    }
    if (send_zero_on_start_)
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

  ~ZsiBotCmdBridge() override
  {
    try
    {
      sendMove(0.0, 0.0, 0.0);
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Failed to send zero command during shutdown: %s", e.what());
    }
  }

private:
  static double clampFinite(double value, double limit)
  {
    if (!std::isfinite(value))
      return 0.0;
    const double abs_limit = std::max(0.0, std::abs(limit));
    return std::clamp(value, -abs_limit, abs_limit);
  }

  static double applyDeadband(double value, double deadband)
  {
    return std::abs(value) < std::abs(deadband) ? 0.0 : value;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    vx_ = applyDeadband(clampFinite(msg->linear.x, max_vx_), deadband_vx_);
    vy_ = applyDeadband(clampFinite(msg->linear.y, max_vy_), deadband_vy_);
    yaw_rate_ = applyDeadband(clampFinite(msg->angular.z, max_yaw_rate_), deadband_yaw_rate_);
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  void timerCallback()
  {
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
  bool send_zero_on_start_{true};
  bool require_connection_{false};
  double status_log_period_{2.0};
  bool log_sdk_status_{true};

  bool have_cmd_{false};
  bool last_sent_zero_{false};
  double vx_{0.0};
  double vy_{0.0};
  double yaw_rate_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace zsibot_cmd_bridge

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
