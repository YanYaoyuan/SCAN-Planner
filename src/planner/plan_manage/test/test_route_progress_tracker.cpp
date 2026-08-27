// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_route_progress_tracker.cpp @brief Topology-safe route progress tests. */

#include <gtest/gtest.h>

#include <plan_manage/route_progress_tracker.h>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace scan_planner
{
namespace
{

std::shared_ptr<const ReferenceRoute> squareLoop()
{
  return std::make_shared<const ReferenceRoute>(
      std::vector<Eigen::Vector3d>{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0}},
      true);
}

void driveSquareLap(RouteProgressTracker &tracker)
{
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> segments{
    {{0.0, 0.0}, {1.0, 0.0}},
    {{1.0, 0.0}, {1.0, 1.0}},
    {{1.0, 1.0}, {0.0, 1.0}},
    {{0.0, 1.0}, {0.0, 0.0}},
  };
  for (const auto &[start, end] : segments)
  {
    const Eigen::Vector2d forward = (end - start).normalized();
    for (int step = 1; step <= 10; ++step)
    {
      const double ratio = static_cast<double>(step) / 10.0;
      const Eigen::Vector2d xy = start + ratio * (end - start);
      const auto result = tracker.update(
          Eigen::Vector3d(xy.x(), xy.y(), 0.0),
          Eigen::Vector3d(forward.x(), forward.y(), 0.0));
      ASSERT_TRUE(result.accepted()) << routeProjectionStatusName(result.status);
    }
  }
}

TEST(RouteProgressTracker, StationaryClosedLoopCannotSpendInitialCredit)
{
  RouteProgressTracker tracker;
  tracker.reset(squareLoop(), Eigen::Vector3d::Zero());

  for (int iteration = 0; iteration < 1000; ++iteration)
  {
    const auto result = tracker.update(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
    EXPECT_TRUE(result.accepted());
  }

  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
  EXPECT_FALSE(tracker.completed());
}

TEST(RouteProgressTracker, SubDeadbandJitterDoesNotAccumulateCredit)
{
  RouteProgressTracker tracker;
  tracker.reset(squareLoop(), Eigen::Vector3d::Zero());
  const double initial_credit = tracker.movementCredit();

  for (int iteration = 0; iteration < 1000; ++iteration)
  {
    const double jitter = iteration % 2 == 0 ? 0.005 : -0.005;
    tracker.update(
        Eigen::Vector3d(jitter, 0.0, 0.0), Eigen::Vector3d::UnitX());
  }

  EXPECT_LT(tracker.progress(), 0.01);
  EXPECT_LE(tracker.movementCredit(), initial_credit);
}

TEST(RouteProgressTracker, UTurnUsesHeadingInsteadOfNearbyReturnBranch)
{
  RouteProgressConfig config;
  config.search_forward_distance = 10.0;
  config.max_projection_advance = 2.0;
  config.max_heading_error = 0.6;
  config.initial_credit = 1.0;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {0.0, 0.0, 0.0},
            {2.0, 0.0, 0.0},
            {2.0, 0.2, 0.0},
            {0.0, 0.2, 0.0}},
          false),
      Eigen::Vector3d::Zero());

  const auto result = tracker.update(
      {0.5, 0.18, 0.0}, Eigen::Vector3d::UnitX());

  ASSERT_TRUE(result.accepted())
      << routeProjectionStatusName(result.status)
      << " candidate=" << result.candidate_route_s
      << " credit=" << result.available_movement_credit;
  EXPECT_NEAR(tracker.progress(), 0.5, 1.0e-9);
  EXPECT_LT(result.candidate_route_s, 1.0);
}

TEST(RouteProgressTracker, FigureEightCrossingUsesForwardDirection)
{
  RouteProgressConfig config;
  config.search_forward_distance = 10.0;
  config.max_projection_advance = 10.0;
  config.max_heading_error = 0.4;
  config.movement_credit_scale = 10.0;
  config.initial_credit = 0.0;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {-1.0, -1.0, 0.0},
            {1.0, 1.0, 0.0},
            {-1.0, 1.0, 0.0},
            {1.0, -1.0, 0.0}},
          false),
      Eigen::Vector3d(-1.0, -1.0, 0.0));

  const auto result = tracker.update(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, -1.0, 0.0));

  ASSERT_TRUE(result.accepted())
      << routeProjectionStatusName(result.status)
      << " candidate=" << result.candidate_route_s
      << " credit=" << result.available_movement_credit;
  EXPECT_GT(result.candidate_route_s, 4.0);
}

TEST(RouteProgressTracker, ReverseMotionCannotDecreaseConfirmedProgress)
{
  RouteProgressConfig config;
  config.initial_credit = 1.0;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, false),
      Eigen::Vector3d::Zero());

  ASSERT_TRUE(tracker.update(
      {1.0, 0.0, 0.0}, Eigen::Vector3d::UnitX()).accepted());
  const double confirmed = tracker.progress();
  ASSERT_TRUE(tracker.update(
      {0.5, 0.0, 0.0}, Eigen::Vector3d::UnitX()).accepted());

  EXPECT_DOUBLE_EQ(tracker.progress(), confirmed);
}

TEST(RouteProgressTracker, OppositeBodyHeadingRejectsProjection)
{
  RouteProgressConfig config;
  config.initial_credit = 1.0;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, false),
      Eigen::Vector3d::Zero());

  const auto result = tracker.update(
      {0.5, 0.0, 0.0}, -Eigen::Vector3d::UnitX());

  EXPECT_EQ(result.status, RouteProjectionStatus::kHeadingGate);
  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
}

TEST(RouteProgressTracker, BestCandidateIsCheckedBeforeMovementBudget)
{
  RouteProgressConfig config;
  config.search_forward_distance = 30.0;
  config.max_projection_advance = 0.2;
  config.max_lateral_distance = 1.0;
  config.max_heading_error = std::acos(-1.0);
  config.movement_deadband = 1.0;
  config.initial_credit = 0.1;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {0.0, 0.0, 0.0},
            {10.0, 0.0, 0.0},
            {10.0, 0.1, 0.0},
            {0.0, 0.1, 0.0}},
          false),
      Eigen::Vector3d::Zero());

  const auto result = tracker.update(
      {0.1, 0.1, 0.0}, -Eigen::Vector3d::UnitX());

  EXPECT_EQ(result.status, RouteProjectionStatus::kMovementBudget);
  EXPECT_GT(result.candidate_route_s, 10.0);
  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
}

TEST(RouteProgressTracker, HeightNoiseWithinSeparateGateIsAccepted)
{
  RouteProgressConfig config;
  config.max_height_error = 0.5;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, false),
      Eigen::Vector3d::Zero());

  const auto result = tracker.update(
      {0.05, 0.0, 0.4}, Eigen::Vector3d::UnitX());

  EXPECT_TRUE(result.accepted());
  EXPECT_NEAR(tracker.progress(), 0.05, 1.0e-9);
}

TEST(RouteProgressTracker, HeightOutsideSeparateGateIsRejected)
{
  RouteProgressConfig config;
  config.max_height_error = 0.2;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, false),
      Eigen::Vector3d::Zero());

  const auto result = tracker.update(
      {0.05, 0.0, 0.3}, Eigen::Vector3d::UnitX());

  EXPECT_EQ(result.status, RouteProjectionStatus::kHeightGate);
  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
}

TEST(RouteProgressTracker, SeamCrossingRequiresFreshForwardMotionEvidence)
{
  RouteProgressTracker tracker;
  tracker.reset(squareLoop(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(tracker.update(
      {1.0, 0.0, 0.0}, Eigen::Vector3d::UnitX()).accepted());
  ASSERT_TRUE(tracker.update(
      {1.0, 1.0, 0.0}, Eigen::Vector3d::UnitY()).accepted());
  ASSERT_TRUE(tracker.update(
      {0.0, 1.0, 0.0}, -Eigen::Vector3d::UnitX()).accepted());
  ASSERT_TRUE(tracker.update(
      {0.0, 0.1, 0.0}, -Eigen::Vector3d::UnitY()).accepted());
  ASSERT_NEAR(tracker.progress(), 3.9, 1.0e-9);

  // Consume the new odometry sample with an invalid heading, then retry the
  // exact same pose. The retry must not reuse that displacement as seam proof.
  EXPECT_EQ(
      tracker.update(
          Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY()).status,
      RouteProjectionStatus::kHeadingGate);
  const auto retry = tracker.update(
      Eigen::Vector3d::Zero(), -Eigen::Vector3d::UnitY());

  EXPECT_EQ(retry.status, RouteProjectionStatus::kSeamGate);
  EXPECT_NEAR(tracker.progress(), 3.9, 1.0e-9);
}

TEST(RouteProgressTracker, ClosedRouteCompletesConfiguredNumberOfLaps)
{
  RouteProgressConfig config;
  config.loop_count = 2;
  RouteProgressTracker tracker(config);
  tracker.reset(squareLoop(), Eigen::Vector3d::Zero());

  driveSquareLap(tracker);
  EXPECT_FALSE(tracker.completed());
  EXPECT_NEAR(tracker.progress(), 4.0, 1.0e-6);
  driveSquareLap(tracker);

  EXPECT_TRUE(tracker.completed());
  EXPECT_NEAR(tracker.progress(), 8.0, 1.0e-6);
}

TEST(RouteProgressTracker, UnlimitedLoopModeNeverCompletes)
{
  RouteProgressConfig config;
  config.loop_count = 0;
  RouteProgressTracker tracker(config);
  tracker.reset(squareLoop(), Eigen::Vector3d::Zero());

  driveSquareLap(tracker);
  driveSquareLap(tracker);

  EXPECT_FALSE(tracker.completed());
  EXPECT_NEAR(tracker.progress(), 8.0, 1.0e-6);
}

TEST(RouteProgressTracker, SegmentCheckLimitFailsClosed)
{
  RouteProgressConfig config;
  config.search_forward_distance = 5.0;
  config.max_projection_advance = 1.0;
  config.max_segment_checks = 2;
  std::vector<Eigen::Vector3d> points;
  for (int index = 0; index <= 10; ++index)
    points.emplace_back(0.25 * index, 0.0, 0.0);
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(points, false),
      Eigen::Vector3d::Zero());

  const auto result = tracker.update(
      {0.1, 0.0, 0.0}, Eigen::Vector3d::UnitX());

  EXPECT_EQ(result.status, RouteProjectionStatus::kResourceLimit);
  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
}

TEST(RouteProgressTracker, ProgressNeverExceedsPhysicalMovementCredit)
{
  RouteProgressConfig config;
  config.movement_credit_scale = 1.25;
  config.initial_credit = 0.05;
  config.max_projection_advance = 2.0;
  RouteProgressTracker tracker(config);
  tracker.reset(
      std::make_shared<const ReferenceRoute>(
          std::vector<Eigen::Vector3d>{
            {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, false),
      Eigen::Vector3d::Zero());

  double physical_distance = 0.0;
  double previous_x = 0.0;
  for (double x : {0.10, 0.35, 0.60, 1.10, 1.40})
  {
    physical_distance += std::abs(x - previous_x);
    previous_x = x;
    tracker.update({x, 0.0, 0.0}, Eigen::Vector3d::UnitX());
    EXPECT_LE(
        tracker.progress(),
        config.initial_credit +
            config.movement_credit_scale * physical_distance + 1.0e-9);
  }
}

TEST(RouteProgressTracker, InvalidResetClearsPreviousMission)
{
  RouteProgressTracker tracker;
  tracker.reset(squareLoop(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(tracker.active());

  EXPECT_THROW(
      tracker.reset(nullptr, Eigen::Vector3d::Zero()),
      std::invalid_argument);
  EXPECT_FALSE(tracker.active());
  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
}

}  // namespace
}  // namespace scan_planner
