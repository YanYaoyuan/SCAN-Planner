// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_reference_path_tracker.cpp @brief Tests reference-route progress safety. */

#include <gtest/gtest.h>

#include <plan_manage/reference_path_tracker.h>

#include <Eigen/Core>

#include <vector>

namespace scan_planner
{
namespace
{

/** @brief Builds a four-metre square lap whose first and last points match. */
std::vector<Eigen::Vector3d> squareLoop()
{
  return {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {1.0, 1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0},
  };
}

TEST(ReferencePathTracker, StationaryClosedLoopNeverAdvancesOrArmsCompletion)
{
  ReferencePathTracker tracker;
  tracker.reset(squareLoop(), true, Eigen::Vector3d::Zero());

  for (int iteration = 0; iteration < 1000; ++iteration)
    tracker.update(Eigen::Vector3d::Zero());

  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
  EXPECT_DOUBLE_EQ(tracker.remainingLength(), 4.0);
  EXPECT_FALSE(tracker.departedStart());
}

TEST(ReferencePathTracker, SubDeadbandLocalizationJitterCannotAccumulateProgress)
{
  ReferencePathTracker tracker;
  tracker.reset(squareLoop(), true, Eigen::Vector3d::Zero());

  for (int iteration = 0; iteration < 1000; ++iteration)
  {
    const double jitter = iteration % 2 == 0 ? 0.005 : -0.005;
    tracker.update(Eigen::Vector3d(jitter, 0.0, 0.0));
  }

  EXPECT_LT(tracker.progress(), 0.01);
  EXPECT_FALSE(tracker.departedStart());
}

TEST(ReferencePathTracker, ClosedLoopCompletesOnlyAfterPhysicalLap)
{
  ReferencePathTracker tracker;
  tracker.reset(squareLoop(), true, Eigen::Vector3d::Zero());

  for (const auto &segment : std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>{
           {{0.0, 0.0}, {1.0, 0.0}},
           {{1.0, 0.0}, {1.0, 1.0}},
           {{1.0, 1.0}, {0.0, 1.0}},
           {{0.0, 1.0}, {0.0, 0.0}},
       })
  {
    for (int step = 1; step <= 20; ++step)
    {
      const double ratio = static_cast<double>(step) / 20.0;
      const Eigen::Vector2d position =
          segment.first + ratio * (segment.second - segment.first);
      tracker.update(Eigen::Vector3d(position.x(), position.y(), 0.0));
    }
  }

  EXPECT_TRUE(tracker.departedStart());
  EXPECT_NEAR(tracker.progress(), tracker.totalLength(), 1.0e-6);
  EXPECT_NEAR(tracker.remainingLength(), 0.0, 1.0e-6);
}

TEST(ReferencePathTracker, FutureCrossingCannotConsumeProgressWithoutMotionCredit)
{
  const std::vector<Eigen::Vector3d> crossing_route{
      {0.0, 0.0, 0.0},
      {1.0, 1.0, 0.0},
      {2.0, 0.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 0.0, 0.0},
      {-1.0, 1.0, 0.0},
  };
  ReferencePathTracker tracker;
  tracker.reset(crossing_route, false, Eigen::Vector3d::Zero());

  tracker.update(Eigen::Vector3d(0.05, 0.05, 0.0));
  const double progress_at_crossing = tracker.progress();
  for (int iteration = 0; iteration < 1000; ++iteration)
    tracker.update(Eigen::Vector3d(0.05, 0.05, 0.0));

  EXPECT_LT(progress_at_crossing, 0.20);
  EXPECT_DOUBLE_EQ(tracker.progress(), progress_at_crossing);
}

TEST(ReferencePathTracker, InterpolatesArcLengthOnOriginalPolyline)
{
  ReferencePathTracker tracker;
  tracker.reset(squareLoop(), true, Eigen::Vector3d::Zero());

  const Eigen::Vector3d point = tracker.pointAt(1.5);
  EXPECT_NEAR(point.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(point.y(), 0.5, 1.0e-9);
  EXPECT_NEAR(tracker.tangentAt(1.5).y(), 1.0, 1.0e-9);
}

TEST(ReferencePathTracker, CompactLoopArmsDepartureFromPhysicalArcProgress)
{
  const std::vector<Eigen::Vector3d> compact_loop{
      {0.0, 0.0, 0.0},
      {0.3, 0.0, 0.0},
      {0.3, 0.3, 0.0},
      {0.0, 0.3, 0.0},
      {0.0, 0.0, 0.0},
  };
  ReferencePathTracker tracker;
  tracker.reset(compact_loop, true, Eigen::Vector3d::Zero());

  for (const auto &segment : std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>{
           {{0.0, 0.0}, {0.3, 0.0}},
           {{0.3, 0.0}, {0.3, 0.3}},
           {{0.3, 0.3}, {0.0, 0.3}},
           {{0.0, 0.3}, {0.0, 0.0}},
       })
  {
    for (int step = 1; step <= 10; ++step)
    {
      const double ratio = static_cast<double>(step) / 10.0;
      const Eigen::Vector2d position =
          segment.first + ratio * (segment.second - segment.first);
      tracker.update(Eigen::Vector3d(position.x(), position.y(), 0.0));
    }
  }

  EXPECT_TRUE(tracker.departedStart());
  EXPECT_NEAR(tracker.remainingLength(), 0.0, 1.0e-6);
}

}  // namespace
}  // namespace scan_planner
