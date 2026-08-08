// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_path_command_governor.cpp @brief 真机线角联合发布序列测试。 */

#include <gtest/gtest.h>

#include <plan_manage/path_command_governor.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace scan_planner
{
namespace
{

constexpr double kDt = 0.02;

PathCommandGovernor::Target target(
    double linear_x,
    double curvature,
    double heading_yaw = 0.0,
    double hard_speed_limit = 0.30,
    bool allow_minimum_speed_creep = false)
{
  return PathCommandGovernor::Target{
      linear_x, hard_speed_limit, curvature, heading_yaw, false,
      allow_minimum_speed_creep};
}

TEST(PathCommandGovernor, RejectsSdkYawMinimumAboveGlobalLimit)
{
  auto config = PathCommandGovernor::Config{};
  config.max_yaw_rate = 0.50;
  config.yaw_filter.min_nonzero_output = 0.60;
  EXPECT_THROW(
      {
        PathCommandGovernor governor(config);
        (void)governor;
      },
      std::invalid_argument);
}

TEST(PathCommandGovernor, RejectsYawStartWithoutSynchronizationCorridor)
{
  auto config = PathCommandGovernor::Config{};
  config.yaw_filter.start_threshold = config.yaw_sync_threshold;
  EXPECT_THROW(
      {
        PathCommandGovernor governor(config);
        (void)governor;
      },
      std::invalid_argument);
}

TEST(PathCommandGovernor, StraightLaunchRespectsSdkMinimumLinearSpeed)
{
  PathCommandGovernor governor;
  bool moved = false;
  double previous_speed = 0.0;
  for (int iteration = 0; iteration < 100; ++iteration)
  {
    const auto command = governor.update(target(0.20, 0.0), kDt);
    ASSERT_TRUE(command.valid);
    EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
    if (command.linear_x > 0.0)
    {
      moved = true;
      EXPECT_GE(command.linear_x, 0.05 - 1.0e-12);
      if (previous_speed > 0.0)
      {
        EXPECT_LE(
            command.linear_x - previous_speed,
            0.35 * kDt + 1.0e-12);
      }
    }
    previous_speed = command.linear_x;
  }
  EXPECT_TRUE(moved);
  EXPECT_NEAR(previous_speed, 0.20, 1.0e-12);
}

TEST(PathCommandGovernor, InvalidTimeStepResetsBeforeMotionCanResume)
{
  PathCommandGovernor governor;
  for (int iteration = 0; iteration < 100; ++iteration)
    governor.update(target(0.20, 0.0), kDt);
  ASSERT_NEAR(governor.lastCommand().linear_x, 0.20, 1.0e-12);

  const auto invalid = governor.update(target(0.20, 0.0), 0.0);
  EXPECT_FALSE(invalid.valid);
  EXPECT_DOUBLE_EQ(invalid.linear_x, 0.0);

  // 下一正常周期必须重新预备最小可执行速度，不能从旧 0.20 瞬间恢复。
  const auto resumed = governor.update(target(0.20, 0.0), kDt);
  EXPECT_TRUE(resumed.valid);
  EXPECT_DOUBLE_EQ(resumed.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(resumed.angular_z, 0.0);
}

TEST(PathCommandGovernor, AcuteTurnLaunchesLinearAndYawTogether)
{
  PathCommandGovernor governor;
  const auto acute = target(0.062, 6.4, 0.0, 0.073);
  bool launched = false;
  for (int iteration = 0; iteration < 200; ++iteration)
  {
    const auto command = governor.update(acute, kDt);
    ASSERT_TRUE(command.valid);
    if (command.linear_x == 0.0)
    {
      // preview 只预热内部状态，等待期绝不能把 preview yaw 发给狗。
      EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
      continue;
    }

    launched = true;
    EXPECT_GE(command.linear_x, 0.05 - 1.0e-12);
    EXPECT_GT(command.angular_z, 0.0);
    EXPECT_NEAR(
        command.raw_yaw_rate,
        command.linear_x * 6.4,
        1.0e-12);
    EXPECT_LE(
        std::abs(command.angular_z),
        std::abs(command.raw_yaw_rate) + 1.0e-12);
    break;
  }
  EXPECT_TRUE(launched);
}

TEST(PathCommandGovernor, OpposingTermsPastZeroRampWithoutPeriodicStop)
{
  PathCommandGovernor governor;
  // 这是 PathTrackingController 在 local=(0.30,-0.15)、heading=+0.30
  // 附近给出的反号组合。SDK 最小线速已经越过 raw-yaw 零点，因此启动
  // 与目标 yaw 都向右，但 raw 幅值会从很小逐步增大。
  const auto opposing = target(
      0.107655, -2.6666666666666665, 0.105, 0.215625);
  bool launched = false;
  double previous_speed = 0.0;
  for (int iteration = 0; iteration < 300; ++iteration)
  {
    const auto command = governor.update(opposing, kDt);
    ASSERT_TRUE(command.valid);
    if (command.linear_x == 0.0)
    {
      // 首次同步启动前允许等待；一旦起步，固定目标下不得周期性清零重启。
      EXPECT_FALSE(launched);
      EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
      continue;
    }

    if (launched)
    {
      EXPECT_LE(
          command.linear_x - previous_speed,
          0.35 * kDt + 1.0e-12);
    }
    launched = true;
    previous_speed = command.linear_x;
    EXPECT_GE(command.linear_x, 0.05 - 1.0e-12);
    EXPECT_NEAR(
        command.raw_yaw_rate,
        command.linear_x * opposing.curvature + opposing.heading_yaw,
        1.0e-12);
    EXPECT_LE(command.angular_z, 0.0);
    if (std::abs(command.raw_yaw_rate) >= 0.10)
    {
      EXPECT_LE(
          std::abs(command.angular_z),
          std::abs(command.raw_yaw_rate) + 1.0e-12);
    }
  }
  EXPECT_TRUE(launched);
  EXPECT_NEAR(previous_speed, opposing.linear_x, 1.0e-12);
}

TEST(PathCommandGovernor, OpposingTermsCanCrossYawZeroWithoutOldDirection)
{
  PathCommandGovernor governor;
  // 最小线速时 raw=+0.075，目标线速时 raw=-0.15：加速途中必须完整
  // 穿过零点。换向期间可以暂时发布 yaw=0，但绝不能让旧的正 yaw 与
  // 已经为负的真实 raw 同时出现，也不能在强阈值处周期停车。
  const auto crossing = target(0.20, -1.50, 0.15, 0.30);
  bool launched = false;
  bool crossed_zero = false;
  double previous_speed = 0.0;
  for (int iteration = 0; iteration < 300; ++iteration)
  {
    const auto command = governor.update(crossing, kDt);
    ASSERT_TRUE(command.valid);
    if (command.linear_x == 0.0)
    {
      EXPECT_FALSE(launched);
      EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
      continue;
    }

    if (launched)
    {
      EXPECT_LE(
          command.linear_x - previous_speed,
          0.35 * kDt + 1.0e-12);
      EXPECT_LE(
          previous_speed - command.linear_x,
          0.60 * kDt + 1.0e-12);
    }
    launched = true;
    previous_speed = command.linear_x;
    EXPECT_GE(command.linear_x, 0.05 - 1.0e-12);
    EXPECT_NEAR(
        command.raw_yaw_rate,
        command.linear_x * crossing.curvature + crossing.heading_yaw,
        1.0e-12);
    EXPECT_GE(
        command.angular_z * command.raw_yaw_rate,
        -1.0e-12);
    if (command.raw_yaw_rate < 0.0)
      crossed_zero = true;
    if (command.angular_z != 0.0)
      EXPECT_GE(std::abs(command.angular_z), 0.10 - 1.0e-12);
    if (std::abs(command.raw_yaw_rate) >= 0.10)
    {
      EXPECT_LE(
          std::abs(command.angular_z),
          std::abs(command.raw_yaw_rate) + 1.0e-12);
    }
  }
  EXPECT_TRUE(launched);
  EXPECT_TRUE(crossed_zero);
  EXPECT_NEAR(previous_speed, crossing.linear_x, 1.0e-12);
}

TEST(PathCommandGovernor, OpposingTermsDetectZeroCrossingBelowSdkMinimum)
{
  PathCommandGovernor governor;
  // raw 零点 vx=0.0471 低于 SDK 的 0.05：首个可发布线速度对应的
  // raw=-0.0125 仍在 yaw deadband 内，不能依赖一次可见的符号翻转来启动
  // 过零治理，否则 raw 刚进入 strong 区时仍可能周期停车。
  const auto near_launch_zero = target(0.10, -4.25, 0.20, 0.30);
  bool launched = false;
  double final_speed = 0.0;
  for (int iteration = 0; iteration < 300; ++iteration)
  {
    const auto command = governor.update(near_launch_zero, kDt);
    ASSERT_TRUE(command.valid);
    if (command.linear_x == 0.0)
    {
      EXPECT_FALSE(launched);
      EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
      continue;
    }

    launched = true;
    final_speed = command.linear_x;
    EXPECT_GE(command.linear_x, 0.05 - 1.0e-12);
    EXPECT_NEAR(
        command.raw_yaw_rate,
        command.linear_x * near_launch_zero.curvature +
            near_launch_zero.heading_yaw,
        1.0e-12);
    EXPECT_GE(
        command.angular_z * command.raw_yaw_rate,
        -1.0e-12);
    if (command.angular_z != 0.0)
      EXPECT_GE(std::abs(command.angular_z), 0.10 - 1.0e-12);
  }
  EXPECT_TRUE(launched);
  EXPECT_NEAR(final_speed, near_launch_zero.linear_x, 1.0e-12);
}

TEST(PathCommandGovernor, SubDeadbandHardLimitTurnsInPlaceInsteadOfDeadlocking)
{
  PathCommandGovernor governor;
  // 近终点大横偏可能请求 kappa>9.4，角速度能力反推的线速低于0.05。
  // 此时必须先原地朝正确方向准备，不能持续发布0/0。
  const auto command = governor.update(target(0.20, 12.0, 0.0, 0.04), kDt);
  ASSERT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_GT(command.angular_z, 0.0);
  EXPECT_TRUE(command.waiting_for_synchronized_launch);
}

TEST(PathCommandGovernor, EndpointSubDeadbandTargetCreepsFromReset)
{
  PathCommandGovernor governor;
  bool moved = false;
  for (int iteration = 0; iteration < 100; ++iteration)
  {
    // distance=0.151m 时默认制动目标约 0.0346m/s，低于 SDK 最小值。
    const auto command = governor.update(
        target(0.0346, 0.0, 0.0, 0.30, true), kDt);
    ASSERT_TRUE(command.valid);
    if (command.linear_x > 0.0)
    {
      moved = true;
      EXPECT_NEAR(command.linear_x, 0.05, 1.0e-12);
    }
  }
  EXPECT_TRUE(moved);
}

TEST(PathCommandGovernor, StartThresholdNeighborhoodHasNoStopBranch)
{
  const std::array<double, 3> raw_targets{0.0799, 0.0800, 0.0801};
  std::array<int, 3> first_move{};
  std::array<int, 3> first_yaw{};
  first_move.fill(-1);
  first_yaw.fill(-1);

  for (std::size_t case_index = 0; case_index < raw_targets.size(); ++case_index)
  {
    PathCommandGovernor governor;
    const double curvature = raw_targets[case_index] / 0.20;
    for (int iteration = 0; iteration < 200; ++iteration)
    {
      const auto command = governor.update(target(0.20, curvature), kDt);
      ASSERT_TRUE(command.valid);
      if (first_move[case_index] < 0 && command.linear_x > 0.0)
        first_move[case_index] = iteration;
      if (first_yaw[case_index] < 0 && command.angular_z > 0.0)
        first_yaw[case_index] = iteration;
    }
    EXPECT_GE(first_move[case_index], 0);
    EXPECT_GE(first_yaw[case_index], 0);
  }

  // 三个几乎相同的目标必须走同一状态分支，不能分别变成直走、死停和
  // 延迟近一秒才启动。
  EXPECT_EQ(first_move[0], first_move[1]);
  EXPECT_EQ(first_move[1], first_move[2]);
  EXPECT_LE(*std::max_element(first_yaw.begin(), first_yaw.end()) -
      *std::min_element(first_yaw.begin(), first_yaw.end()), 1);
}

TEST(PathCommandGovernor, PreviewCannotExceedActualRawYaw)
{
  PathCommandGovernor governor;
  const auto acute = target(0.062, 6.4, 0.0, 0.073);
  bool launched = false;
  double previous_speed = 0.0;
  for (int iteration = 0; iteration < 200; ++iteration)
  {
    const auto command = governor.update(acute, kDt);
    if (command.linear_x == 0.0)
    {
      EXPECT_FALSE(launched);
      continue;
    }
    if (launched)
    {
      EXPECT_GE(command.linear_x + 1.0e-12, previous_speed);
      EXPECT_LE(
          command.linear_x - previous_speed,
          0.35 * kDt + 1.0e-12);
    }
    launched = true;
    previous_speed = command.linear_x;
    ASSERT_GE(std::abs(command.raw_yaw_rate), 0.10);
    EXPECT_LE(
        std::abs(command.angular_z),
        std::abs(command.raw_yaw_rate) + 1.0e-12);
  }
  EXPECT_TRUE(launched);
  EXPECT_NEAR(previous_speed, 0.062, 1.0e-12);
}

TEST(PathCommandGovernor, ZeroRawYawNeverPublishesPreviewResidue)
{
  PathCommandGovernor governor;
  for (int iteration = 0; iteration < 100; ++iteration)
    governor.update(target(0.20, 1.0), kDt);

  const auto straight = governor.update(target(0.20, 0.0), kDt);
  ASSERT_TRUE(straight.valid);
  EXPECT_GT(straight.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(straight.raw_yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(straight.angular_z, 0.0);
}

TEST(PathCommandGovernor, WeakReversalNeverPublishesOldYawDirection)
{
  PathCommandGovernor governor;
  for (int iteration = 0; iteration < 100; ++iteration)
    governor.update(target(0.20, 1.0), kDt);

  bool saw_suppression = false;
  bool reached_right_turn = false;
  for (int iteration = 0; iteration < 200; ++iteration)
  {
    // 巡航时 raw yaw 约 -0.07rad/s，低于同步停车阈值，允许直行过渡，
    // 但任何时刻都不能继续发布旧的正向 yaw。
    const auto command = governor.update(target(0.20, -0.35), kDt);
    ASSERT_TRUE(command.valid);
    EXPECT_LE(command.angular_z, 0.0);
    if (command.suppressed_wrong_direction_yaw)
      saw_suppression = true;
    if (command.angular_z < 0.0)
      reached_right_turn = true;
  }
  EXPECT_TRUE(saw_suppression);
  EXPECT_TRUE(reached_right_turn);
}

TEST(PathCommandGovernor, StrongReversalStopsUntilCorrectYawIsReady)
{
  PathCommandGovernor governor;
  for (int iteration = 0; iteration < 100; ++iteration)
    governor.update(target(0.20, 1.0), kDt);

  bool stopped_for_reversal = false;
  bool relaunched_right = false;
  for (int iteration = 0; iteration < 300; ++iteration)
  {
    const auto command = governor.update(target(0.20, -1.0), kDt);
    ASSERT_TRUE(command.valid);
    EXPECT_FALSE(command.linear_x > 0.0 && command.angular_z > 0.0);
    if (command.linear_x == 0.0)
      stopped_for_reversal = true;
    if (command.linear_x >= 0.05 && command.angular_z < 0.0)
      relaunched_right = true;
  }
  EXPECT_TRUE(stopped_for_reversal);
  EXPECT_TRUE(relaunched_right);
}

TEST(PathCommandGovernor, NormalStopUsesLinearDecelerationBeforeReset)
{
  PathCommandGovernor governor;
  for (int iteration = 0; iteration < 100; ++iteration)
    governor.update(target(0.20, 0.0), kDt);

  const auto first_stop_step = governor.update(target(0.0, 0.0), kDt);
  ASSERT_TRUE(first_stop_step.valid);
  EXPECT_GT(first_stop_step.linear_x, 0.0);
  EXPECT_NEAR(first_stop_step.linear_x, 0.188, 1.0e-12);

  bool settled = false;
  double previous_speed = first_stop_step.linear_x;
  double last_nonzero_speed = first_stop_step.linear_x;
  for (int iteration = 0; iteration < 100; ++iteration)
  {
    const auto command = governor.update(target(0.0, 0.0), kDt);
    ASSERT_TRUE(command.valid);
    if (command.linear_x > 0.0)
    {
      EXPECT_GE(command.linear_x, 0.05 - 1.0e-12);
      EXPECT_LE(
          previous_speed - command.linear_x,
          0.60 * kDt + 1.0e-12);
      last_nonzero_speed = command.linear_x;
    }
    previous_speed = command.linear_x;
    if (command.settled)
    {
      settled = true;
      break;
    }
  }
  EXPECT_TRUE(settled);
  EXPECT_NEAR(last_nonzero_speed, 0.05, 1.0e-12);
}

}  // namespace
}  // namespace scan_planner
