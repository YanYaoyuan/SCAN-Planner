// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file bspline_message_validator.h @brief 控制器入口 B-spline 消息校验。 */

#ifndef PLAN_MANAGE__BSPLINE_MESSAGE_VALIDATOR_H_
#define PLAN_MANAGE__BSPLINE_MESSAGE_VALIDATOR_H_

#include <scan_planner_msgs/msg/bspline.hpp>

#include <cmath>
#include <cstddef>
#include <string>

namespace scan_planner
{

struct BsplineValidationResult
{
  bool valid{false};
  bool clear_command{false};
  double duration{0.0};
  std::string reason;
};

/**
 * @brief 在构造 UniformBspline 前完成全部边界校验。
 *
 * UniformBspline::setKnot() 只替换向量且不会检查长度；错误 knot 可能触发
 * Eigen 越界断言，无法靠 try/catch 恢复，所以必须在 ROS 消息入口拒绝。
 */
inline BsplineValidationResult validateBsplineMessage(
    const scan_planner_msgs::msg::Bspline &message,
    const std::string &expected_frame,
    std::size_t max_control_points,
    double max_duration)
{
  BsplineValidationResult result;

  const bool clear_shape = message.order == 0 && message.traj_id == -1 &&
      message.pos_pts.empty() && message.knots.empty();
  if (clear_shape)
  {
    result.valid = true;
    result.clear_command = true;
    result.reason = "clear command";
    return result;
  }

  if (message.order != 3)
  {
    result.reason = "only cubic B-splines (order=3) are accepted";
    return result;
  }
  if (message.traj_id <= 0)
  {
    result.reason = "active trajectory requires traj_id > 0";
    return result;
  }
  if (message.start_time.nanosec >= 1000000000u)
  {
    result.reason = "trajectory start_time has an invalid nanosecond field";
    return result;
  }
  if (message.header.frame_id.empty())
  {
    result.reason = "frame_id is empty";
    return result;
  }
  if (!expected_frame.empty() && message.header.frame_id != expected_frame)
  {
    result.reason = "frame_id does not match controller frame";
    return result;
  }

  const std::size_t point_count = message.pos_pts.size();
  const std::size_t degree = static_cast<std::size_t>(message.order);
  if (point_count < degree + 1 || point_count > max_control_points)
  {
    result.reason = "invalid control-point count";
    return result;
  }
  if (message.knots.size() != point_count + degree + 1)
  {
    result.reason = "knot count must equal control_points + order + 1";
    return result;
  }

  for (const auto &point : message.pos_pts)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
    {
      result.reason = "control point contains NaN/Inf";
      return result;
    }
  }

  constexpr double kMinimumKnotStep = 1.0e-9;
  for (std::size_t index = 0; index < message.knots.size(); ++index)
  {
    if (!std::isfinite(message.knots[index]))
    {
      result.reason = "knot contains NaN/Inf";
      return result;
    }
    if (index > 0)
    {
      const double knot_step =
          message.knots[index] - message.knots[index - 1];
      // 两个有限极大值相减仍可能溢出为 Inf，不能只检查 <= epsilon。
      if (!std::isfinite(knot_step) || knot_step <= kMinimumKnotStep)
      {
        result.reason = "knots must be finite and strictly increasing";
        return result;
      }
    }
  }

  // 对 N 个控制点、p 次 B-spline，有效时域是 [u[p], u[N]]。
  result.duration =
      message.knots[point_count] - message.knots[degree];
  if (!std::isfinite(result.duration) || result.duration <= 1.0e-6 ||
      result.duration > max_duration)
  {
    result.reason = "trajectory duration is outside the safe range";
    result.duration = 0.0;
    return result;
  }

  result.valid = true;
  result.reason = "valid";
  return result;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE__BSPLINE_MESSAGE_VALIDATOR_H_
