// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_yaw_command_filter.cpp @brief Tests wheel-legged yaw smoothing. */

#include <gtest/gtest.h>

#include <plan_manage/yaw_command_filter.h>

#include <cmath>

namespace scan_planner
{
namespace
{

TEST(YawCommandFilter, AlternatingSmallCommandsDoNotReverseOutput)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.03, 0.20, 1.0, 0.15, 0.12, 0.06, 0.10});

  for (int iteration = 0; iteration < 100; ++iteration)
  {
    const double raw_command = iteration % 2 == 0 ? 0.11 : -0.11;
    EXPECT_GE(filter.update(raw_command, 0.01), 0.0);
  }
}

TEST(YawCommandFilter, AngularAccelerationBoundsEveryStep)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0});

  double previous = 0.0;
  for (int iteration = 0; iteration < 100; ++iteration)
  {
    const double output = filter.update(0.5, 0.01);
    EXPECT_LE(std::abs(output - previous), 0.008 + 1.0e-12);
    previous = output;
  }
  EXPECT_NEAR(previous, 0.5, 1.0e-9);
}

TEST(YawCommandFilter, StrongReversalCrossesZeroWithoutJump)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.03, 0.0, 1.0, 0.15, 0.0, 0.0, 0.0});

  for (int iteration = 0; iteration < 30; ++iteration)
    filter.update(0.4, 0.01);
  EXPECT_NEAR(filter.output(), 0.3, 1.0e-9);

  const double first_reversal_step = filter.update(-0.4, 0.01);
  EXPECT_NEAR(first_reversal_step, 0.29, 1.0e-9);
  EXPECT_GT(first_reversal_step, 0.0);
}

TEST(YawCommandFilter, DeadbandDecaysAndSnapsToZero)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.04, 0.10, 2.0, 0.15, 0.0, 0.0, 0.0});

  for (int iteration = 0; iteration < 100; ++iteration)
    filter.update(0.2, 0.01);
  for (int iteration = 0; iteration < 100; ++iteration)
    filter.update(0.01, 0.01);

  EXPECT_DOUBLE_EQ(filter.output(), 0.0);
}

TEST(YawCommandFilter, ResetImmediatelyClearsResidualCommand)
{
  YawCommandFilter filter;
  for (int iteration = 0; iteration < 20; ++iteration)
    filter.update(0.5, 0.01);
  ASSERT_GT(filter.output(), 0.0);

  filter.reset();
  EXPECT_DOUBLE_EQ(filter.output(), 0.0);
}

TEST(YawCommandFilter, UsesStartStopHysteresisAndRobotMinimumCommand)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.0, 0.0, 10.0, 0.15, 0.12, 0.06, 0.10});

  EXPECT_DOUBLE_EQ(filter.update(0.11, 0.1), 0.0);
  EXPECT_NEAR(filter.update(0.13, 0.1), 0.13, 1.0e-12);
  EXPECT_NEAR(filter.update(0.08, 0.1), 0.10, 1.0e-12);
  EXPECT_DOUBLE_EQ(filter.update(0.05, 0.1), 0.0);
}

TEST(YawCommandFilter, ReversalMustReenterThroughStartThreshold)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.0, 0.0, 10.0, 0.0, 0.12, 0.06, 0.10});

  EXPECT_NEAR(filter.update(0.20, 0.1), 0.20, 1.0e-12);
  EXPECT_DOUBLE_EQ(filter.update(-0.20, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(filter.update(-0.11, 0.1), 0.0);
  EXPECT_NEAR(filter.update(-0.13, 0.1), -0.13, 1.0e-12);
}

TEST(YawCommandFilter, ConservativeRobotDefaultsStartPersistentModerateTurn)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.025, 0.12, 1.0, 0.12, 0.08, 0.04, 0.10});

  double output = 0.0;
  for (int iteration = 0; iteration < 200; ++iteration)
    output = filter.update(0.09, 0.02);

  // SDK 量化要求决定最终输出只能是 0 或至少 0.1rad/s。
  EXPECT_NEAR(output, 0.10, 1.0e-12);
}

TEST(YawCommandFilter, ExactRawStartThresholdCannotDeadlock)
{
  YawCommandFilter filter(
      YawCommandFilter::Config{0.025, 0.12, 1.0, 0.12, 0.08, 0.04, 0.10});

  // 旧实现等待 filtered>=0.08；一阶低通只能从下方渐近 0.08，最终会
  // 停在相邻浮点数而永久输出零。现在由去死区后的 raw 目标触发启动。
  const double output = filter.update(0.08, 0.02);
  EXPECT_NEAR(output, 0.10, 1.0e-12);
}

}  // namespace
}  // namespace scan_planner
