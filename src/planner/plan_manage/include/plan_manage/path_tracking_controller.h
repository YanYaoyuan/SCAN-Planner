// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file path_tracking_controller.h @brief 四足机器人 Pure Pursuit 目标速度计算。 */

#ifndef PLAN_MANAGE__PATH_TRACKING_CONTROLLER_H_
#define PLAN_MANAGE__PATH_TRACKING_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scan_planner
{

/**
 * @brief 由机体系前视点生成线速度和角速度目标。
 *
 * 这里仅计算目标值；最终线加减速和角速度低通由发布层完成。
 * TRACK/ALIGN 使用不同进入、退出阈值，避免在单一阈值附近抖动。
 */
class PathTrackingController
{
public:
  struct Config
  {
    double cruise_speed{0.20};
    double max_forward_speed{0.30};
    double max_yaw_rate{0.50};
    double align_yaw_gain{0.80};
    double heading_feedback_gain{0.35};
    double align_enter_error{0.80};
    double align_exit_error{0.45};
    double heading_slowdown_start{0.20};
    double curvature_speed_gain{0.60};
    double yaw_rate_reserve{0.03};
    double max_lateral_acceleration{0.20};
    double max_linear_deceleration{0.60};
    double finish_distance{0.15};
    double lateral_error_deadband{0.04};
    double curvature_deadband{0.04};
    double minimum_lookahead{0.05};
  };

  struct Command
  {
    double linear_x{0.0};
    double angular_z{0.0};
    double curvature{0.0};
    double heading_yaw{0.0};
    double hard_speed_limit{0.0};
    bool rotate_in_place{false};
    bool allow_minimum_speed_creep{false};
    bool valid{false};
  };

  PathTrackingController()
  {
    setConfig(Config{});
  }

  explicit PathTrackingController(const Config &config)
  {
    setConfig(config);
  }

  void setConfig(const Config &config)
  {
    const bool valid =
        std::isfinite(config.cruise_speed) && config.cruise_speed >= 0.0 &&
        std::isfinite(config.max_forward_speed) && config.max_forward_speed > 0.0 &&
        std::isfinite(config.max_yaw_rate) && config.max_yaw_rate > 0.0 &&
        std::isfinite(config.align_yaw_gain) && config.align_yaw_gain >= 0.0 &&
        std::isfinite(config.heading_feedback_gain) && config.heading_feedback_gain >= 0.0 &&
        std::isfinite(config.align_enter_error) && config.align_enter_error > 0.0 &&
        std::isfinite(config.align_exit_error) && config.align_exit_error >= 0.0 &&
        config.align_exit_error < config.align_enter_error &&
        std::isfinite(config.heading_slowdown_start) &&
        config.heading_slowdown_start >= 0.0 &&
        config.heading_slowdown_start < config.align_enter_error &&
        std::isfinite(config.curvature_speed_gain) && config.curvature_speed_gain >= 0.0 &&
        std::isfinite(config.yaw_rate_reserve) && config.yaw_rate_reserve >= 0.0 &&
        config.yaw_rate_reserve < config.max_yaw_rate &&
        std::isfinite(config.max_lateral_acceleration) &&
        config.max_lateral_acceleration > 0.0 &&
        std::isfinite(config.max_linear_deceleration) &&
        config.max_linear_deceleration > 0.0 &&
        std::isfinite(config.finish_distance) && config.finish_distance >= 0.0 &&
        std::isfinite(config.lateral_error_deadband) &&
        config.lateral_error_deadband >= 0.0 &&
        std::isfinite(config.curvature_deadband) && config.curvature_deadband >= 0.0 &&
        std::isfinite(config.minimum_lookahead) && config.minimum_lookahead > 0.0;
    if (!valid)
      throw std::invalid_argument("invalid path-tracking controller configuration");
    config_ = config;
  }

  /** @brief 新任务或急停后回到 TRACK，避免继承上一次 ALIGN 状态。 */
  void reset()
  {
    aligning_ = false;
  }

  bool aligning() const { return aligning_; }

  /**
   * @brief 计算一个 Pure Pursuit 控制目标。
   * @param local_x 前视点在机体系的前向坐标。
   * @param local_y 前视点在机体系的左向坐标。
   * @param heading_error 路径切线航向减机器人航向，单位 rad。
   * @param target_is_end 前视点是否已经到达本条局部轨迹末端。
   * @param distance_to_end 机器人到局部轨迹末端的平面距离。
   */
  Command compute(
      double local_x,
      double local_y,
      double heading_error,
      bool target_is_end,
      double distance_to_end)
  {
    Command command;
    if (!std::isfinite(local_x) || !std::isfinite(local_y) ||
        !std::isfinite(heading_error) || !std::isfinite(distance_to_end))
    {
      return command;
    }
    command.valid = true;
    command.hard_speed_limit = config_.max_forward_speed;

    if (target_is_end && distance_to_end <= config_.finish_distance)
    {
      // 进入终点容差后给发布治理器一个“普通零目标”，让线速度按正常
      // 减速度收敛。这里不能把 hard_speed_limit 清零，否则会退化成急停。
      aligning_ = false;
      return command;
    }

    const double abs_heading_error = std::abs(heading_error);
    if (aligning_)
    {
      if (abs_heading_error <= config_.align_exit_error)
        aligning_ = false;
    }
    else if (abs_heading_error >= config_.align_enter_error)
    {
      aligning_ = true;
    }

    if (aligning_)
    {
      command.rotate_in_place = true;
      // ALIGN 的语义是原地转向；硬上限清零，立即结束残余平移。
      command.hard_speed_limit = 0.0;
      // 在退出阈值处把 ALIGN 增益连续收敛到 TRACK 航向增益，避免状态
      // 切换时角速度目标从约 0.36 突降到 0.16rad/s 后继续过度旋转。
      const double gain_ratio = std::clamp(
          (abs_heading_error - config_.align_exit_error) /
              (config_.align_enter_error - config_.align_exit_error),
          0.0,
          1.0);
      const double scheduled_align_gain =
          config_.heading_feedback_gain +
          gain_ratio *
              (config_.align_yaw_gain - config_.heading_feedback_gain);
      command.heading_yaw = std::clamp(
          scheduled_align_gain * heading_error,
          -config_.max_yaw_rate,
          config_.max_yaw_rate);
      command.angular_z = command.heading_yaw;
      return command;
    }

    // 前视点已经落到机体侧后方时，先朝前视点方向转正再继续前进。
    if (local_x < 0.02)
    {
      command.rotate_in_place = true;
      command.hard_speed_limit = 0.0;
      command.heading_yaw = std::clamp(
          config_.align_yaw_gain * std::atan2(local_y, local_x),
          -config_.max_yaw_rate,
          config_.max_yaw_rate);
      command.angular_z = command.heading_yaw;
      return command;
    }

    const double filtered_y =
        std::abs(local_y) < config_.lateral_error_deadband ? 0.0 : local_y;
    const double lookahead_squared = std::max(
        config_.minimum_lookahead * config_.minimum_lookahead,
        local_x * local_x + local_y * local_y);
    double curvature = 2.0 * filtered_y / lookahead_squared;
    if (std::abs(curvature) < config_.curvature_deadband)
      curvature = 0.0;
    command.curvature = curvature;

    double speed = std::min(config_.cruise_speed, config_.max_forward_speed);
    double hard_speed_limit = config_.max_forward_speed;

    // 曲率越大速度越低，避免固定速度在急弯处切弯。
    speed = std::min(
        speed,
        config_.max_forward_speed /
            (1.0 + config_.curvature_speed_gain * std::abs(curvature)));
    if (std::abs(curvature) > 1.0e-6)
    {
      speed = std::min(
          speed,
          std::sqrt(
              config_.max_lateral_acceleration / std::abs(curvature)));
    }

    // 从 slowdown_start 到 ALIGN 进入阈值之间使用余弦连续降速。
    if (abs_heading_error > config_.heading_slowdown_start)
    {
      const double ratio = std::clamp(
          (abs_heading_error - config_.heading_slowdown_start) /
              (config_.align_enter_error - config_.heading_slowdown_start),
          0.0,
          1.0);
      constexpr double kPi = 3.14159265358979323846;
      speed *= 0.5 * (1.0 + std::cos(kPi * ratio));
    }

    const double heading_yaw =
        config_.heading_feedback_gain * heading_error;
    command.heading_yaw = heading_yaw;
    if (std::abs(curvature) > 1.0e-6)
    {
      // 由 |vx*kappa + w_heading| <= yaw_limit 反推线速度上限。
      // 曲率前馈与航向反馈同号时预算相减，反号时二者会互相抵消，预算
      // 应相加。若一律减去 |w_heading|，S 弯中会产生并不存在的低速上限，
      // 甚至让发布治理器误入 0/0 的永久等待状态。
      const double yaw_limit =
          config_.max_yaw_rate - config_.yaw_rate_reserve;
      const double curvature_sign = std::copysign(1.0, curvature);
      const double yaw_budget = std::max(
          0.0, yaw_limit - curvature_sign * heading_yaw);
      hard_speed_limit = std::min(
          hard_speed_limit, yaw_budget / std::abs(curvature));
    }

    if (std::abs(curvature) > 1.0e-6)
    {
      hard_speed_limit = std::min(
          hard_speed_limit,
          std::sqrt(
              config_.max_lateral_acceleration / std::abs(curvature)));
    }

    if (target_is_end)
    {
      // 终点制动属于正常舒适减速目标，不写入 hard_speed_limit；否则发布层
      // 会绕过线减速度限制直接跳到 stopping_speed。急停才允许绕过滤波。
      const double braking_distance =
          std::max(0.0, distance_to_end - config_.finish_distance);
      const double stopping_speed = std::sqrt(
          2.0 * config_.max_linear_deceleration * braking_distance);
      command.allow_minimum_speed_creep =
          stopping_speed > 0.0 && stopping_speed < speed;
      speed = std::min(speed, stopping_speed);
    }

    command.hard_speed_limit =
        std::clamp(hard_speed_limit, 0.0, config_.max_forward_speed);
    speed = std::min(speed, command.hard_speed_limit);
    command.linear_x = std::clamp(speed, 0.0, config_.max_forward_speed);
    command.angular_z = std::clamp(
        command.linear_x * curvature + heading_yaw,
        -config_.max_yaw_rate,
        config_.max_yaw_rate);
    return command;
  }

private:
  Config config_;
  bool aligning_{false};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__PATH_TRACKING_CONTROLLER_H_
