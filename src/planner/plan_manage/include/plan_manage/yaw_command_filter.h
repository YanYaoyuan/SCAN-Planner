// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file yaw_command_filter.h @brief Stateful smoothing for wheel-legged yaw commands. */

#ifndef PLAN_MANAGE__YAW_COMMAND_FILTER_H_
#define PLAN_MANAGE__YAW_COMMAND_FILTER_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scan_planner
{

/**
 * @brief Smooths angular velocity while suppressing small sign reversals.
 *
 * The filter combines an input deadband, first-order low-pass response,
 * angular-acceleration limiting, and reversal hysteresis. Safety stops must
 * bypass this class and call reset() so zero velocity remains immediate.
 */
class YawCommandFilter
{
public:
  /** @brief Tunable yaw-command smoothing limits. */
  struct Config
  {
    double deadband{0.03};             ///< Raw yaw rates below this magnitude become zero.
    double time_constant{0.20};        ///< First-order low-pass time constant in seconds.
    double max_acceleration{1.0};      ///< Maximum yaw-rate change in rad/s^2.
    double reversal_threshold{0.15};   ///< Opposite commands below this magnitude first decay to zero.
    double start_threshold{0.12};      ///< Filtered magnitude required to start turning.
    double stop_threshold{0.06};       ///< Filtered magnitude below which turning stops.
    double min_nonzero_output{0.10};   ///< Smallest non-zero command accepted by the robot.
  };

  /** @brief Constructs a filter with conservative wheel-legged defaults. */
  YawCommandFilter()
  {
    setConfig(Config{});
  }

  /**
   * @brief Constructs a filter with explicit limits.
   * @param config Valid non-negative smoothing limits.
   */
  explicit YawCommandFilter(const Config &config)
  {
    setConfig(config);
  }

  /**
   * @brief Updates smoothing limits without changing current filter state.
   * @param config Valid non-negative smoothing limits.
   * @throws std::invalid_argument If a value is invalid.
   */
  void setConfig(const Config &config)
  {
    if (!std::isfinite(config.deadband) || config.deadband < 0.0 ||
        !std::isfinite(config.time_constant) || config.time_constant < 0.0 ||
        !std::isfinite(config.max_acceleration) ||
        config.max_acceleration <= 0.0 ||
        !std::isfinite(config.reversal_threshold) ||
        config.reversal_threshold < config.deadband ||
        !std::isfinite(config.start_threshold) ||
        config.start_threshold < 0.0 ||
        !std::isfinite(config.stop_threshold) ||
        config.stop_threshold < 0.0 ||
        config.stop_threshold > config.start_threshold ||
        !std::isfinite(config.min_nonzero_output) ||
        config.min_nonzero_output < 0.0)
    {
      throw std::invalid_argument("invalid yaw-command filter configuration");
    }
    config_ = config;
  }

  /**
   * @brief Filters one raw angular-velocity command.
   * @param raw_command Desired yaw rate in rad/s.
   * @param dt Controller period in seconds.
   * @return Smoothed yaw rate in rad/s.
   */
  double update(double raw_command, double dt)
  {
    if (!std::isfinite(raw_command) || !std::isfinite(dt) || dt <= 0.0)
      return command_output_;

    double target =
        std::abs(raw_command) < config_.deadband ? 0.0 : raw_command;
    if (filtered_output_ * target < 0.0 &&
        std::abs(target) < config_.reversal_threshold)
    {
      target = 0.0;
    }

    const double alpha = config_.time_constant <= 0.0
        ? 1.0
        : dt / (config_.time_constant + dt);
    const double low_pass_target =
        filtered_output_ +
        std::clamp(alpha, 0.0, 1.0) * (target - filtered_output_);
    const double max_delta = config_.max_acceleration * dt;
    filtered_output_ += std::clamp(
        low_pass_target - filtered_output_, -max_delta, max_delta);

    const double zero_threshold =
        std::max(1.0e-6, 0.5 * config_.deadband);
    if (target == 0.0 && std::abs(filtered_output_) < zero_threshold)
      filtered_output_ = 0.0;

    const int filtered_sign =
        (filtered_output_ > 0.0) - (filtered_output_ < 0.0);
    bool exited_turning = false;
    if (turning_ &&
        (std::abs(filtered_output_) <= config_.stop_threshold ||
         filtered_sign != turning_sign_))
    {
      turning_ = false;
      turning_sign_ = 0;
      exited_turning = true;
    }
    if (!turning_ && !exited_turning &&
        std::abs(filtered_output_) >= config_.start_threshold &&
        filtered_sign != 0)
    {
      turning_ = true;
      turning_sign_ = filtered_sign;
    }

    if (!turning_)
    {
      command_output_ = 0.0;
      return command_output_;
    }

    command_output_ = std::copysign(
        std::max(std::abs(filtered_output_), config_.min_nonzero_output),
        filtered_output_);
    return command_output_;
  }

  /** @brief Immediately clears all filter memory for a safety stop. */
  void reset()
  {
    filtered_output_ = 0.0;
    command_output_ = 0.0;
    turning_ = false;
    turning_sign_ = 0;
  }

  /** @brief Returns the most recently filtered command. */
  double output() const
  {
    return command_output_;
  }

private:
  Config config_;
  double filtered_output_{0.0};
  double command_output_{0.0};
  bool turning_{false};
  int turning_sign_{0};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__YAW_COMMAND_FILTER_H_
