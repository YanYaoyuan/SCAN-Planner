// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_bspline_message_validator.cpp @brief B-spline ROS 消息入口校验测试。 */

#include <gtest/gtest.h>

#include <plan_manage/bspline_message_validator.h>

#include <limits>

namespace scan_planner
{
namespace
{

scan_planner_msgs::msg::Bspline validMessage()
{
  scan_planner_msgs::msg::Bspline message;
  message.header.frame_id = "lio_map";
  message.order = 3;
  message.traj_id = 1;
  for (int index = 0; index < 6; ++index)
  {
    geometry_msgs::msg::Point point;
    point.x = 0.1 * index;
    message.pos_pts.push_back(point);
  }
  // N=6,p=3，因此 knot 数必须为 10；仓库生成器使用严格递增均匀 knot。
  for (int index = 0; index < 10; ++index)
    message.knots.push_back(-0.3 + 0.1 * index);
  return message;
}

TEST(BsplineMessageValidator, AcceptsValidCubicMessage)
{
  const auto result = validateBsplineMessage(
      validMessage(), "lio_map", 1000, 60.0);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.clear_command);
  EXPECT_NEAR(result.duration, 0.3, 1.0e-12);
}

TEST(BsplineMessageValidator, AcceptsOnlyExactClearSentinel)
{
  scan_planner_msgs::msg::Bspline clear;
  clear.order = 0;
  clear.traj_id = -1;
  const auto accepted = validateBsplineMessage(clear, "lio_map", 1000, 60.0);
  EXPECT_TRUE(accepted.valid);
  EXPECT_TRUE(accepted.clear_command);

  clear.traj_id = 0;
  EXPECT_FALSE(validateBsplineMessage(clear, "lio_map", 1000, 60.0).valid);
}

TEST(BsplineMessageValidator, RejectsWrongKnotCountAndDuplicateKnot)
{
  auto message = validMessage();
  message.knots.pop_back();
  EXPECT_FALSE(validateBsplineMessage(message, "lio_map", 1000, 60.0).valid);

  message = validMessage();
  message.knots[5] = message.knots[4];
  EXPECT_FALSE(validateBsplineMessage(message, "lio_map", 1000, 60.0).valid);
}

TEST(BsplineMessageValidator, RejectsNonFiniteData)
{
  auto message = validMessage();
  message.pos_pts[2].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validateBsplineMessage(message, "lio_map", 1000, 60.0).valid);

  message = validMessage();
  message.knots[3] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validateBsplineMessage(message, "lio_map", 1000, 60.0).valid);
}

TEST(BsplineMessageValidator, RejectsWrongFrameAndNonCubicOrder)
{
  auto message = validMessage();
  EXPECT_FALSE(validateBsplineMessage(message, "map", 1000, 60.0).valid);
  message.header.frame_id = "map";
  message.order = 2;
  EXPECT_FALSE(validateBsplineMessage(message, "map", 1000, 60.0).valid);
}

TEST(BsplineMessageValidator, RejectsInvalidStartTimestamp)
{
  auto message = validMessage();
  message.start_time.nanosec = 1000000000u;
  EXPECT_FALSE(validateBsplineMessage(message, "lio_map", 1000, 60.0).valid);
}

}  // namespace
}  // namespace scan_planner
