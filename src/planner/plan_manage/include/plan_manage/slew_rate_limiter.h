// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file slew_rate_limiter.h @brief 线速度加减速限制器。 */

#ifndef PLAN_MANAGE__SLEW_RATE_LIMITER_H_
#define PLAN_MANAGE__SLEW_RATE_LIMITER_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scan_planner
{

/** @brief 对标量速度做带独立加速/减速上限的变化率限制。 */
class SlewRateLimiter
{
public:
  struct Config
  {
    double max_acceleration{0.35};  ///< 速度幅值增大时的上限，单位 m/s^2。
    double max_deceleration{0.60};  ///< 速度幅值减小时的上限，单位 m/s^2。
  };

  SlewRateLimiter()
  {
    setConfig(Config{});
  }

  explicit SlewRateLimiter(const Config &config)
  {
    setConfig(config);
  }

  void setConfig(const Config &config)
  {
    if (!std::isfinite(config.max_acceleration) ||
        config.max_acceleration <= 0.0 ||
        !std::isfinite(config.max_deceleration) ||
        config.max_deceleration <= 0.0)
    {
      throw std::invalid_argument("invalid slew-rate limiter configuration");
    }
    config_ = config;
  }

  /**
   * @brief 生成本周期可安全达到的速度。
   *
   * 发生正负换向时先按减速度回到 0，下一周期再向反方向加速，
   * 避免一个控制周期内直接跨过零点。
   */
  double update(double target, double dt)
  {
    if (!std::isfinite(target) || !std::isfinite(dt) || dt <= 0.0)
      return output_;

    if (output_ * target < 0.0)
    {
      const double step = config_.max_deceleration * dt;
      if (std::abs(output_) <= step)
        output_ = 0.0;
      else
        output_ -= std::copysign(step, output_);
      return output_;
    }

    const bool increasing_magnitude = std::abs(target) > std::abs(output_);
    const double limit = increasing_magnitude
        ? config_.max_acceleration
        : config_.max_deceleration;
    const double max_delta = limit * dt;
    output_ += std::clamp(target - output_, -max_delta, max_delta);
    return output_;
  }

  /** @brief 急停后直接清零内部状态，不延迟安全停车。 */
  void reset(double value = 0.0)
  {
    output_ = std::isfinite(value) ? value : 0.0;
  }

  double output() const { return output_; }

private:
  Config config_;
  double output_{0.0};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__SLEW_RATE_LIMITER_H_
