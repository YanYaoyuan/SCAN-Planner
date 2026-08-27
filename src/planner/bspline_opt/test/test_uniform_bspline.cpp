// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_uniform_bspline.cpp @brief Unit tests for uniform B-spline behavior. */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>

#include <chrono>

TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}

TEST(UniformBspline, ReportsFiniteVelocityAndAccelerationStatistics)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 0.5);
  double mean_velocity = -1.0;
  double max_velocity = -1.0;
  double mean_acceleration = -1.0;
  double max_acceleration = -1.0;

  spline.getMeanAndMaxVel(mean_velocity, max_velocity);
  spline.getMeanAndMaxAcc(mean_acceleration, max_acceleration);

  EXPECT_NEAR(mean_velocity, 2.0, 1e-9);
  EXPECT_NEAR(max_velocity, 2.0, 1e-9);
  EXPECT_NEAR(mean_acceleration, 0.0, 1e-9);
  EXPECT_NEAR(max_acceleration, 0.0, 1e-9);
}

TEST(UniformBspline, RejectsTruncatedKnotSpanInVelocityStatistics)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 3);
  scan_planner::UniformBspline spline(points, 1, 0.5);
  Eigen::VectorXd knot = spline.getKnot();
  spline.setKnot(knot.head(knot.rows() - 1));

  double mean_velocity = -1.0;
  double max_velocity = -1.0;
  spline.getMeanAndMaxVel(mean_velocity, max_velocity);

  EXPECT_DOUBLE_EQ(mean_velocity, 0.0);
  EXPECT_DOUBLE_EQ(max_velocity, 0.0);
}

TEST(BsplineOptimizerDeadline, ExposesSharedDeadlineState)
{
  scan_planner::BsplineOptimizer optimizer;
  optimizer.setDeadline(std::chrono::steady_clock::now());
  EXPECT_TRUE(optimizer.deadlineExceeded());

  optimizer.clearDeadline();
  EXPECT_FALSE(optimizer.deadlineExceeded());
}
