// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_trajectory_acceptance_validator.cpp @brief Final trajectory acceptance tests. */

#include <gtest/gtest.h>

#include <plan_manage/trajectory_acceptance_validator.h>

#include <Eigen/Core>

#include <cstddef>
#include <limits>

namespace scan_planner
{
namespace
{

UniformBspline linearSpline(double interval = 1.0)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (Eigen::Index column = 0; column < points.cols(); ++column)
    points(0, column) = 0.1 * static_cast<double>(column);
  return UniformBspline(points, 3, interval);
}

TrajectoryAcceptanceConfig validConfig()
{
  TrajectoryAcceptanceConfig config;
  config.max_velocity = 1.0;
  config.max_acceleration = 1.0;
  config.max_spatial_sample_step = 0.05;
  config.max_temporal_sample_step = 0.05;
  config.max_samples = 1000;
  return config;
}

TEST(TrajectoryAcceptanceValidator, AcceptsFiniteFreeBoundedTrajectory)
{
  std::size_t collision_queries = 0;
  const auto validation = TrajectoryAcceptanceValidator(validConfig()).validate(
      linearSpline(),
      [&collision_queries](const Eigen::Vector3d &, double) {
        ++collision_queries;
        return 0;
      },
      []() { return false; });

  EXPECT_TRUE(validation.accepted());
  EXPECT_EQ(validation.status, TrajectoryAcceptanceStatus::kAccepted);
  EXPECT_EQ(collision_queries, validation.sample_count);
  EXPECT_GT(validation.sample_count, 2u);
}

TEST(TrajectoryAcceptanceValidator, RejectsNonFiniteControlPoint)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  points(1, 2) = std::numeric_limits<double>::quiet_NaN();
  const auto validation = TrajectoryAcceptanceValidator(validConfig()).validate(
      UniformBspline(points, 3, 1.0),
      [](const Eigen::Vector3d &, double) { return 0; },
      []() { return false; });

  EXPECT_EQ(
      validation.status,
      TrajectoryAcceptanceStatus::kInvalidTrajectory);
}

TEST(TrajectoryAcceptanceValidator, RejectsContinuousVelocityControlBound)
{
  TrajectoryAcceptanceConfig config = validConfig();
  config.max_velocity = 0.05;
  const auto validation = TrajectoryAcceptanceValidator(config).validate(
      linearSpline(),
      [](const Eigen::Vector3d &, double) { return 0; },
      []() { return false; });

  EXPECT_EQ(validation.status, TrajectoryAcceptanceStatus::kVelocityLimit);
  EXPECT_GT(validation.observed_value, validation.limit_value);
}

TEST(TrajectoryAcceptanceValidator, TreatsUnknownMapAsCollision)
{
  const auto validation = TrajectoryAcceptanceValidator(validConfig()).validate(
      linearSpline(),
      [](const Eigen::Vector3d &, double) { return -1; },
      []() { return false; });

  EXPECT_EQ(validation.status, TrajectoryAcceptanceStatus::kCollision);
  EXPECT_EQ(validation.occupancy, -1);
  EXPECT_EQ(validation.sample_index, 0u);
}

TEST(TrajectoryAcceptanceValidator, IncludesExactTrajectoryEndpoint)
{
  const auto validation = TrajectoryAcceptanceValidator(validConfig()).validate(
      linearSpline(),
      [](const Eigen::Vector3d &position, double) {
        return position.x() >= 0.4 - 1.0e-12 ? 1 : 0;
      },
      []() { return false; });

  EXPECT_EQ(validation.status, TrajectoryAcceptanceStatus::kCollision);
  ASSERT_GT(validation.sample_count, 0u);
  EXPECT_EQ(validation.sample_index, validation.sample_count - 1);
}

TEST(TrajectoryAcceptanceValidator, RejectsInsteadOfCoarseningPastSampleBudget)
{
  TrajectoryAcceptanceConfig config = validConfig();
  config.max_temporal_sample_step = 0.001;
  config.max_samples = 10;
  std::size_t collision_queries = 0;
  const auto validation = TrajectoryAcceptanceValidator(config).validate(
      linearSpline(),
      [&collision_queries](const Eigen::Vector3d &, double) {
        ++collision_queries;
        return 0;
      },
      []() { return false; });

  EXPECT_EQ(validation.status, TrajectoryAcceptanceStatus::kResourceLimit);
  EXPECT_EQ(collision_queries, 0u);
}

TEST(TrajectoryAcceptanceValidator, DeadlineStopsInsideCollisionLoop)
{
  std::size_t collision_queries = 0;
  const auto validation = TrajectoryAcceptanceValidator(validConfig()).validate(
      linearSpline(),
      [&collision_queries](const Eigen::Vector3d &, double) {
        ++collision_queries;
        return 0;
      },
      [&collision_queries]() { return collision_queries >= 3; });

  EXPECT_EQ(
      validation.status,
      TrajectoryAcceptanceStatus::kDeadlineExceeded);
  EXPECT_EQ(collision_queries, 3u);
}

}  // namespace
}  // namespace scan_planner
