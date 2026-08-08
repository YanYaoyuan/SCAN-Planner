// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file planner_heartbeat_gate.h @brief 规划器心跳运动授权门。 */

#ifndef PLAN_MANAGE__PLANNER_HEARTBEAT_GATE_H_
#define PLAN_MANAGE__PLANNER_HEARTBEAT_GATE_H_

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cstdint>

namespace scan_planner
{

/** @brief 仅在新鲜心跳精确匹配当前轨迹身份时允许非零运动。 */
class PlannerHeartbeatGate
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void update(
      bool motion_allowed,
      std::int64_t traj_id,
      const builtin_interfaces::msg::Time &trajectory_start_time,
      TimePoint receive_time = Clock::now())
  {
    motion_allowed_ = motion_allowed;
    traj_id_ = traj_id;
    trajectory_start_time_ = trajectory_start_time;
    receive_time_ = receive_time;
    have_heartbeat_ = true;
  }

  bool authorized(
      std::int64_t active_traj_id,
      const builtin_interfaces::msg::Time &active_start_time,
      double timeout,
      TimePoint now = Clock::now()) const
  {
    if (!have_heartbeat_ || !motion_allowed_ || timeout <= 0.0 ||
        active_traj_id <= 0 || traj_id_ != active_traj_id ||
        !sameTimestamp(trajectory_start_time_, active_start_time))
    {
      return false;
    }
    const double age = std::chrono::duration<double>(now - receive_time_).count();
    return age >= 0.0 && age <= timeout;
  }

  void invalidate()
  {
    have_heartbeat_ = false;
    motion_allowed_ = false;
    traj_id_ = -1;
  }

  static bool sameTimestamp(
      const builtin_interfaces::msg::Time &lhs,
      const builtin_interfaces::msg::Time &rhs)
  {
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
  }

private:
  bool have_heartbeat_{false};
  bool motion_allowed_{false};
  std::int64_t traj_id_{-1};
  builtin_interfaces::msg::Time trajectory_start_time_;
  TimePoint receive_time_{};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__PLANNER_HEARTBEAT_GATE_H_
