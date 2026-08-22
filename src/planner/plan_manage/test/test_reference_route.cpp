// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_reference_route.cpp @brief Immutable reference-route geometry tests. */

#include <gtest/gtest.h>

#include <plan_manage/reference_route.h>

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace scan_planner
{
namespace
{

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

bool containsVertexAt(
    const ReferenceRouteRangeResult &range,
    double route_s,
    double tolerance = 1.0e-9)
{
  return std::any_of(
      range.samples.begin(),
      range.samples.end(),
      [route_s, tolerance](const ReferenceRouteSample &sample) {
        return sample.is_original_vertex &&
            std::abs(sample.route_s - route_s) <= tolerance;
      });
}

TEST(ReferenceRoute, DeduplicatesConsecutiveInputWithoutChangingLength)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0},
       {0.0, 0.0, 0.0},
       {1.0, 0.0, 0.0},
       {1.0, 0.0, 0.0},
       {1.0, 2.0, 0.0}},
      false);

  EXPECT_EQ(route.points().size(), 3u);
  EXPECT_NEAR(route.totalLength(), 3.0, 1.0e-12);
}

TEST(ReferenceRoute, RejectsZeroLengthAndNonFiniteGeometry)
{
  EXPECT_THROW(
      ReferenceRoute(
          {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, false),
      std::invalid_argument);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      ReferenceRoute(
          {{0.0, 0.0, 0.0}, {nan, 1.0, 0.0}}, false),
      std::invalid_argument);
}

TEST(ReferenceRoute, InterpolatesUnevenPolylineByMetricArcLength)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}},
      false);

  EXPECT_TRUE(route.pointAt(1.0).isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), 1.0e-12));
  EXPECT_TRUE(route.pointAt(2.5).isApprox(
      Eigen::Vector3d(2.0, 0.5, 0.0), 1.0e-12));
  EXPECT_TRUE(route.tangentAt(2.0).isApprox(
      Eigen::Vector3d(0.0, 1.0, 0.0), 1.0e-12));
}

TEST(ReferenceRoute, PreservesRightAngleAndUTurnVertices)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0},
       {1.0, 0.0, 0.0},
       {1.0, 1.0, 0.0},
       {0.1, 1.0, 0.0}},
      false);
  const auto range = route.sampleRange(0.2, 2.8, 2.0);

  ASSERT_TRUE(range.success());
  EXPECT_TRUE(containsVertexAt(range, 1.0));
  EXPECT_TRUE(containsVertexAt(range, 2.0));
}

TEST(ReferenceRoute, OpenRouteRejectsOutOfBoundsCoordinates)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, false);

  EXPECT_THROW(route.pointAt(-0.01), std::out_of_range);
  EXPECT_THROW(route.pointAt(1.01), std::out_of_range);
  EXPECT_EQ(
      route.sampleRange(-0.01, 0.5, 0.1).status,
      ReferenceRouteRangeStatus::kOutOfRange);
  EXPECT_TRUE(route.pointAt(0.0).isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(route.pointAt(1.0).isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(ReferenceRoute, ClosedRouteSamplesAcrossSeamWithUnwrappedArcLength)
{
  const ReferenceRoute route(squareLoop(), true);
  const auto range = route.sampleRange(3.5, 4.5, 0.75);

  ASSERT_TRUE(range.success());
  ASSERT_TRUE(containsVertexAt(range, 4.0));
  const auto seam = std::find_if(
      range.samples.begin(),
      range.samples.end(),
      [](const ReferenceRouteSample &sample) {
        return std::abs(sample.route_s - 4.0) <= 1.0e-9;
      });
  ASSERT_NE(seam, range.samples.end());
  EXPECT_TRUE(seam->position.isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.0), 1.0e-12));
  EXPECT_NEAR(route.normalizeRouteS(4.5), 0.5, 1.0e-12);
  EXPECT_NEAR(route.normalizeRouteS(-0.5), 3.5, 1.0e-12);
  EXPECT_GT(route.pointAt(4.0 - 1.0e-6).y(), 0.0);
  EXPECT_LT(route.pointAt(4.0 - 1.0e-6).y(), 2.0e-6);
}

TEST(ReferenceRoute, AcceptsRoundedIntegralSpacingAcrossClosedSeam)
{
  const ReferenceRoute route(squareLoop(), true);
  const double start_s = route.totalLength() - 0.25;
  const auto range = route.sampleRange(start_s, start_s + 0.4, 0.05);

  ASSERT_TRUE(range.success());
  EXPECT_DOUBLE_EQ(range.samples.front().route_s, start_s);
  EXPECT_DOUBLE_EQ(range.samples.back().route_s, start_s + 0.4);
  EXPECT_TRUE(containsVertexAt(range, route.totalLength()));
}

TEST(ReferenceRoute, EnforcesSampleBudgetBeforeLargeAllocation)
{
  ReferenceRoute::Config config;
  config.max_sample_count = 8;
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}}, false, config);

  const auto result = route.sampleRange(0.0, 100.0, 0.01);
  EXPECT_EQ(result.status, ReferenceRouteRangeStatus::kResourceLimit);
  EXPECT_TRUE(result.samples.empty());
}

TEST(ReferenceRoute, DeduplicatesCornersBeforeApplyingFinalSampleLimit)
{
  ReferenceRoute::Config config;
  config.max_sample_count = 3;
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      false,
      config);

  const auto result = route.sampleRange(0.0, 2.0, 1.0);
  ASSERT_TRUE(result.success());
  ASSERT_EQ(result.samples.size(), 3u);
  EXPECT_TRUE(std::all_of(
      result.samples.begin(),
      result.samples.end(),
      [](const ReferenceRouteSample &sample) {
        return sample.is_original_vertex;
      }));
}

TEST(ReferenceRoute, VeryShortRangeStillIncludesBothExactBoundaries)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, false);
  const auto result = route.sampleRange(0.20, 0.21, 0.50);

  ASSERT_TRUE(result.success());
  ASSERT_EQ(result.samples.size(), 2u);
  EXPECT_DOUBLE_EQ(result.samples.front().route_s, 0.20);
  EXPECT_DOUBLE_EQ(result.samples.back().route_s, 0.21);
}

TEST(ReferenceRoute, UpstreamLengthDoesNotDependOnRobotOdom)
{
  const std::vector<Eigen::Vector3d> upstream{
    {10.0, 0.0, 0.0}, {11.0, 0.0, 0.0}, {11.0, 2.0, 0.0}};
  const ReferenceRoute route(upstream, false);

  EXPECT_NEAR(route.totalLength(), 3.0, 1.0e-12);
  EXPECT_TRUE(route.pointAt(0.0).isApprox(upstream.front(), 1.0e-12));
}

}  // namespace
}  // namespace scan_planner
