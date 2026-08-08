// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_local_trajectory_tracker.cpp @brief 局部轨迹真实空间进度测试。 */

#include <gtest/gtest.h>

#include <plan_manage/local_trajectory_tracker.h>

#include <Eigen/Core>

#include <vector>

namespace scan_planner
{
namespace
{

TEST(LocalTrajectoryTracker, StationaryOdomCannotAdvanceByRepeatedUpdates)
{
  LocalTrajectoryTracker tracker;
  tracker.reset(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      {0.0, 1.0, 2.0},
      Eigen::Vector3d::Zero());

  for (int iteration = 0; iteration < 1000; ++iteration)
    tracker.update(Eigen::Vector3d::Zero());

  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
  EXPECT_DOUBLE_EQ(tracker.progressTime(), 0.0);
}

TEST(LocalTrajectoryTracker, MapsActualSpatialProgressBackToSplineTime)
{
  LocalTrajectoryTracker tracker(
      LocalTrajectoryTracker::Config{2.0, 2.0, 0.0, 0.2});
  tracker.reset(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {3.0, 0.0, 0.0}},
      {0.0, 2.0, 3.0},
      Eigen::Vector3d::Zero());

  tracker.update(Eigen::Vector3d(0.5, 0.0, 0.0));
  EXPECT_NEAR(tracker.progress(), 0.5, 1.0e-9);
  EXPECT_NEAR(tracker.progressTime(), 1.0, 1.0e-9);

  tracker.update(Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_NEAR(tracker.progress(), 2.0, 1.0e-9);
  EXPECT_NEAR(tracker.progressTime(), 2.5, 1.0e-9);
}

TEST(LocalTrajectoryTracker, ProgressNeverMovesBackward)
{
  LocalTrajectoryTracker tracker(
      LocalTrajectoryTracker::Config{2.0, 2.0, 0.0, 0.5});
  tracker.reset(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      {0.0, 1.0, 2.0},
      Eigen::Vector3d::Zero());

  tracker.update(Eigen::Vector3d(1.0, 0.0, 0.0));
  const double forward_progress = tracker.progress();
  tracker.update(Eigen::Vector3d(0.2, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(tracker.progress(), forward_progress);
}

TEST(LocalTrajectoryTracker, FutureCrossingNeedsPhysicalMovementCredit)
{
  LocalTrajectoryTracker tracker(
      LocalTrajectoryTracker::Config{0.5, 1.5, 0.01, 0.1});
  tracker.reset(
      {{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {2.0, 0.0, 0.0},
       {1.0, -1.0, 0.0}, {0.0, 0.0, 0.0}},
      {0.0, 1.0, 2.0, 3.0, 4.0},
      Eigen::Vector3d::Zero());

  for (int iteration = 0; iteration < 1000; ++iteration)
    tracker.update(Eigen::Vector3d::Zero());

  EXPECT_DOUBLE_EQ(tracker.progress(), 0.0);
}

TEST(LocalTrajectoryTracker, SingleUpdateCannotJumpPastProjectionLimit)
{
  LocalTrajectoryTracker tracker(
      LocalTrajectoryTracker::Config{0.5, 2.0, 0.0, 0.1});
  tracker.reset(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      {0.0, 1.0, 2.0},
      Eigen::Vector3d::Zero());

  tracker.update(Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_NEAR(tracker.progress(), 0.5, 1.0e-12);
}

TEST(LocalTrajectoryTracker, TrustedHeartbeatProgressOnlyMovesForward)
{
  LocalTrajectoryTracker tracker;
  tracker.reset(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      {0.0, 1.0, 2.0},
      Eigen::Vector3d::Zero());

  tracker.advanceTo(1.4);
  EXPECT_NEAR(tracker.progress(), 1.4, 1.0e-12);
  EXPECT_NEAR(tracker.progressTime(), 1.4, 1.0e-12);
  tracker.advanceTo(0.2);
  EXPECT_NEAR(tracker.progress(), 1.4, 1.0e-12);
  tracker.advanceTo(100.0);
  EXPECT_NEAR(tracker.progress(), 2.0, 1.0e-12);
}

}  // namespace
}  // namespace scan_planner
