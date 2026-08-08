// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_planner_heartbeat_gate.cpp @brief 规划器失联与轨迹身份门控测试。 */

#include <gtest/gtest.h>

#include <plan_manage/planner_heartbeat_gate.h>

#include <chrono>

namespace scan_planner
{
namespace
{

builtin_interfaces::msg::Time stamp(std::int32_t sec, std::uint32_t nanosec)
{
  builtin_interfaces::msg::Time value;
  value.sec = sec;
  value.nanosec = nanosec;
  return value;
}

TEST(PlannerHeartbeatGate, RequiresFreshMatchingDoubleIdentity)
{
  PlannerHeartbeatGate gate;
  const auto now = PlannerHeartbeatGate::Clock::now();
  const auto start = stamp(10, 20);
  EXPECT_FALSE(gate.authorized(1, start, 0.4, now));

  gate.update(true, 1, start, now);
  EXPECT_TRUE(gate.authorized(1, start, 0.4, now));
  EXPECT_FALSE(gate.authorized(2, start, 0.4, now));
  EXPECT_FALSE(gate.authorized(1, stamp(11, 20), 0.4, now));
}

TEST(PlannerHeartbeatGate, PlannerRestartCannotReactivateOldTrajectoryId)
{
  PlannerHeartbeatGate gate;
  const auto now = PlannerHeartbeatGate::Clock::now();
  const auto old_start = stamp(10, 0);
  const auto restarted_start = stamp(20, 0);
  gate.update(true, 1, restarted_start, now);
  EXPECT_FALSE(gate.authorized(1, old_start, 0.4, now));
  EXPECT_TRUE(gate.authorized(1, restarted_start, 0.4, now));
}

TEST(PlannerHeartbeatGate, DisallowAndTimeoutStopMotion)
{
  PlannerHeartbeatGate gate;
  const auto now = PlannerHeartbeatGate::Clock::now();
  const auto start = stamp(10, 0);
  gate.update(false, -1, start, now);
  EXPECT_FALSE(gate.authorized(1, start, 0.4, now));

  gate.update(true, 1, start, now);
  EXPECT_FALSE(gate.authorized(
      1, start, 0.4, now + std::chrono::milliseconds(401)));
}

}  // namespace
}  // namespace scan_planner
