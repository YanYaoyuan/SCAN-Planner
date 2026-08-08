// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_path_tracking_controller.cpp @brief Pure Pursuit 速度治理测试。 */

#include <gtest/gtest.h>

#include <plan_manage/path_tracking_controller.h>
#include <plan_manage/slew_rate_limiter.h>

#include <cmath>

namespace scan_planner
{
namespace
{

TEST(PathTrackingController, StraightTargetDrivesForwardWithoutYaw)
{
  PathTrackingController controller;
  const auto command = controller.compute(0.45, 0.0, 0.0, false, 2.0);
  ASSERT_TRUE(command.valid);
  EXPECT_GT(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.curvature, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(PathTrackingController, LeftAndRightTurnsAreSymmetric)
{
  PathTrackingController left_controller;
  PathTrackingController right_controller;
  const auto left = left_controller.compute(0.45, 0.15, 0.0, false, 2.0);
  const auto right = right_controller.compute(0.45, -0.15, 0.0, false, 2.0);
  EXPECT_NEAR(left.linear_x, right.linear_x, 1.0e-12);
  EXPECT_NEAR(left.curvature, -right.curvature, 1.0e-12);
  EXPECT_NEAR(left.angular_z, -right.angular_z, 1.0e-12);
}

TEST(PathTrackingController, YawCapabilityLimitsSpeedBeforeAngularClamp)
{
  PathTrackingController controller;
  const auto command = controller.compute(0.10, 0.20, 0.30, false, 2.0);
  ASSERT_TRUE(command.valid);
  EXPECT_LE(
      std::abs(command.linear_x * command.curvature + command.heading_yaw),
      0.50 - 0.03 + 1.0e-12);
  EXPECT_LE(command.linear_x, command.hard_speed_limit + 1.0e-12);
}

TEST(PathTrackingController, OpposingCurvatureAndHeadingUseSignedYawBudget)
{
  PathTrackingController controller;
  // 曲率右转、航向反馈左转时二者会抵消；不能按同向最坏情况减预算。
  const auto command = controller.compute(
      0.20404, -0.20, 0.70, false, 2.0);
  ASSERT_TRUE(command.valid);
  ASSERT_LT(command.curvature * command.heading_yaw, 0.0);
  EXPECT_GT(command.hard_speed_limit, 0.05);
  EXPECT_LE(
      std::abs(
          command.hard_speed_limit * command.curvature +
          command.heading_yaw),
      0.50 - 0.03 + 1.0e-9);
}

TEST(PathTrackingController, HeadingFeedbackWorksDuringNormalTrack)
{
  PathTrackingController controller;
  const auto command = controller.compute(0.45, 0.0, 0.20, false, 2.0);
  EXPECT_GT(command.linear_x, 0.0);
  EXPECT_NEAR(command.angular_z, 0.35 * 0.20, 1.0e-12);
}

TEST(PathTrackingController, AlignUsesEnterAndExitHysteresis)
{
  PathTrackingController controller;
  EXPECT_FALSE(controller.compute(0.45, 0.0, 0.79, false, 2.0).rotate_in_place);
  const auto align = controller.compute(0.45, 0.0, 0.81, false, 2.0);
  EXPECT_TRUE(align.rotate_in_place);
  EXPECT_DOUBLE_EQ(align.hard_speed_limit, 0.0);
  EXPECT_TRUE(controller.compute(0.45, 0.0, 0.60, false, 2.0).rotate_in_place);
  EXPECT_FALSE(controller.compute(0.45, 0.0, 0.44, false, 2.0).rotate_in_place);
}

TEST(PathTrackingController, AlignYawGainIsContinuousAtExit)
{
  PathTrackingController controller;
  ASSERT_TRUE(controller.compute(0.45, 0.0, 0.81, false, 2.0).rotate_in_place);
  const auto just_inside_align =
      controller.compute(0.45, 0.0, 0.451, false, 2.0);
  const auto just_outside_align =
      controller.compute(0.45, 0.0, 0.449, false, 2.0);
  ASSERT_TRUE(just_inside_align.rotate_in_place);
  ASSERT_FALSE(just_outside_align.rotate_in_place);
  EXPECT_NEAR(
      just_inside_align.angular_z,
      just_outside_align.angular_z,
      0.002);
}

TEST(PathTrackingController, TargetBehindRobotForcesRotation)
{
  PathTrackingController controller;
  const auto command = controller.compute(-0.2, 0.1, 0.0, false, 2.0);
  EXPECT_TRUE(command.rotate_in_place);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_GT(command.angular_z, 0.0);
}

TEST(PathTrackingController, HeadingSlowdownIsContinuousAndMonotonic)
{
  PathTrackingController controller;
  const auto small = controller.compute(0.45, 0.0, 0.20, false, 2.0);
  const auto medium = controller.compute(0.45, 0.0, 0.50, false, 2.0);
  const auto large = controller.compute(0.45, 0.0, 0.79, false, 2.0);
  EXPECT_GE(small.linear_x, medium.linear_x);
  EXPECT_GE(medium.linear_x, large.linear_x);
  EXPECT_GT(medium.linear_x, 0.0);
  EXPECT_GT(large.linear_x, 0.0);
}

TEST(PathTrackingController, EndpointBrakingIsNormalTargetNotHardClamp)
{
  PathTrackingController controller;
  const auto command = controller.compute(0.45, 0.0, 0.0, true, 0.151);
  EXPECT_LT(command.linear_x, 0.05);
  EXPECT_DOUBLE_EQ(command.hard_speed_limit, 0.30);
  EXPECT_TRUE(command.allow_minimum_speed_creep);
}

TEST(PathTrackingController, InsideFinishToleranceReturnsNormalZeroTarget)
{
  PathTrackingController controller;
  // endpoint 即使位于机体后方，也不能在已经到达后重新进入原地 ALIGN。
  const auto command = controller.compute(-0.10, 0.02, 1.0, true, 0.149);
  ASSERT_TRUE(command.valid);
  EXPECT_FALSE(command.rotate_in_place);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(command.hard_speed_limit, 0.30);
}

TEST(SlewRateLimiter, BoundsAccelerationAndDeceleration)
{
  SlewRateLimiter limiter(SlewRateLimiter::Config{0.35, 0.60});
  EXPECT_NEAR(limiter.update(0.20, 0.02), 0.007, 1.0e-12);
  limiter.reset(0.20);
  EXPECT_NEAR(limiter.update(0.0, 0.02), 0.188, 1.0e-12);
}

TEST(SlewRateLimiter, ReversalMustCrossZero)
{
  SlewRateLimiter limiter(SlewRateLimiter::Config{1.0, 1.0});
  limiter.reset(0.05);
  EXPECT_DOUBLE_EQ(limiter.update(-0.20, 0.10), 0.0);
  EXPECT_NEAR(limiter.update(-0.20, 0.10), -0.10, 1.0e-12);
}

}  // namespace
}  // namespace scan_planner
