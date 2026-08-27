// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_merge_reference_builder.cpp @brief Merge-reference tests. */

#include <gtest/gtest.h>

#include <plan_manage/merge_reference_builder.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <vector>

namespace scan_planner
{
namespace
{

const MergeReferenceBuilder::DeadlineQuery kAlive = []() { return false; };

MergeReferenceConfig permissiveConfig()
{
  MergeReferenceConfig config;
  config.maximum_seed_curvature = 1000.0;
  config.maximum_forward_distance = 5.0;
  config.max_samples = 10000;
  return config;
}

const MergeReferenceSample * sampleAt(
    const MergeReferenceResult &result, double route_s)
{
  for (const auto &sample : result.samples)
  {
    if (std::abs(sample.route_s - route_s) <= 1.0e-9)
      return &sample;
  }
  return nullptr;
}

TEST(MergeReferenceBuilder, ZeroOffsetReturnsUnmodifiedOrderedRoute)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}, false);
  auto config = permissiveConfig();
  const auto result = MergeReferenceBuilder::build(
      route, {0.0, 0.0, 0.0}, 0.0, 2.0, 0.0, config, kAlive);

  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.no_merge_required);
  EXPECT_DOUBLE_EQ(result.actual_forward_distance, 0.0);
  ASSERT_NE(sampleAt(result, 1.0), nullptr);
  EXPECT_TRUE(sampleAt(result, 1.0)->is_original_route_vertex);
  for (const auto &sample : result.samples)
  {
    EXPECT_NEAR((sample.position - route.pointAt(sample.route_s)).norm(),
                0.0, 1.0e-12);
    EXPECT_EQ(sample.source, LocalRouteProgressSource::kDirect);
  }
}

TEST(MergeReferenceBuilder, LeftAndRightOffsetsAreSymmetric)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 1.0;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.sample_spacing = 0.1;

  const auto left = MergeReferenceBuilder::build(
      route, {0.0, 0.2, 0.0}, 0.0, 2.0, 0.0, config, kAlive);
  const auto right = MergeReferenceBuilder::build(
      route, {0.0, -0.2, 0.0}, 0.0, 2.0, 0.0, config, kAlive);
  ASSERT_TRUE(left.success());
  ASSERT_TRUE(right.success());
  ASSERT_EQ(left.samples.size(), right.samples.size());
  for (std::size_t index = 0; index < left.samples.size(); ++index)
  {
    EXPECT_NEAR(left.samples[index].position.x(),
                right.samples[index].position.x(), 1.0e-12);
    EXPECT_NEAR(left.samples[index].position.y(),
                -right.samples[index].position.y(), 1.0e-12);
  }
}

TEST(MergeReferenceBuilder, AnchorsCurrentPoseAndReturnsToRouteTangent)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 1.0;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.sample_spacing = 0.01;
  const auto result = MergeReferenceBuilder::build(
      route, {0.0, 0.25, 0.0}, 0.0, 2.0, 0.0, config, kAlive);

  ASSERT_TRUE(result.success());
  EXPECT_NEAR((result.samples.front().position -
               Eigen::Vector3d(0.0, 0.25, 0.0)).norm(), 0.0, 1.0e-12);
  const auto *merge_end = sampleAt(result, 1.0);
  ASSERT_NE(merge_end, nullptr);
  EXPECT_NEAR((merge_end->position - Eigen::Vector3d(1.0, 0.0, 0.0)).norm(),
              0.0, 1.0e-12);
  EXPECT_NEAR((merge_end->tangent - Eigen::Vector3d::UnitX()).norm(),
              0.0, 1.0e-12);
  EXPECT_EQ(merge_end->source, LocalRouteProgressSource::kDirect);
}

TEST(MergeReferenceBuilder, QuinticFadeHasZeroEndSlopeAndSecondDerivative)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 1.0;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.sample_spacing = 0.001;
  const auto result = MergeReferenceBuilder::build(
      route, {0.0, 0.5, 0.0}, 0.0, 1.2, 0.0, config, kAlive);
  ASSERT_TRUE(result.success());

  const auto *p0 = sampleAt(result, 1.000);
  const auto *p1 = sampleAt(result, 0.999);
  const auto *p2 = sampleAt(result, 0.998);
  ASSERT_NE(p0, nullptr);
  ASSERT_NE(p1, nullptr);
  ASSERT_NE(p2, nullptr);
  const double h = 0.001;
  const double slope = (p0->position.y() - p1->position.y()) / h;
  const double second =
      (p0->position.y() - 2.0 * p1->position.y() + p2->position.y()) /
      (h * h);
  EXPECT_NEAR(slope, 0.0, 1.0e-4);
  EXPECT_NEAR(second, 0.0, 0.04);
}

TEST(MergeReferenceBuilder, RejectsStartNearSharpOriginalCorner)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 2.0, 0.0}}, false);
  auto config = permissiveConfig();
  const auto result = MergeReferenceBuilder::build(
      route, {1.0, 0.2, 0.0}, 1.0, 2.0, 0.0, config, kAlive);
  EXPECT_EQ(result.status,
            MergeReferenceStatus::kStartNearUnsmoothedCorner);
}

TEST(MergeReferenceBuilder, ExtendsMergeEndPastCornerAndPreservesVertex)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 2.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 0.8;
  config.maximum_forward_distance = 2.0;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.corner_clearance = 0.1;
  config.sample_spacing = 0.05;
  const auto result = MergeReferenceBuilder::build(
      route, {0.2, -0.1, 0.0}, 0.2, 2.0, 0.0, config, kAlive);

  ASSERT_TRUE(result.success());
  EXPECT_NEAR(result.merge_end_route_s, 1.1, 1.0e-12);
  EXPECT_TRUE(result.crosses_unsmoothed_corner);
  const auto *corner = sampleAt(result, 1.0);
  ASSERT_NE(corner, nullptr);
  EXPECT_TRUE(corner->is_original_route_vertex);
  const auto *merge_end = sampleAt(result, result.merge_end_route_s);
  ASSERT_NE(merge_end, nullptr);
  const Eigen::Vector2d chord =
      merge_end->position.head<2>() -
      result.samples.front().position.head<2>();
  const Eigen::Vector2d corner_offset =
      corner->position.head<2>() - result.samples.front().position.head<2>();
  const double corner_distance_to_chord = std::abs(
      chord.x() * corner_offset.y() - chord.y() * corner_offset.x()) /
      chord.norm();
  EXPECT_GT(corner_distance_to_chord, 0.05);
}

TEST(MergeReferenceBuilder, ReportsClippedWhenCornerCannotBeCleared)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 2.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 0.8;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.corner_clearance = 0.1;
  const auto result = MergeReferenceBuilder::build(
      route, {0.2, -0.1, 0.0}, 0.2, 1.05, 0.0, config, kAlive);

  EXPECT_EQ(result.status, MergeReferenceStatus::kInsufficientLookahead);
  EXPECT_TRUE(result.clipped_by_local_target);
  EXPECT_FALSE(result.success());
}

TEST(MergeReferenceBuilder, LongerMergeReducesEstimatedCurvature)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}}, false);
  auto short_config = permissiveConfig();
  short_config.minimum_forward_distance = 0.5;
  short_config.lateral_distance_scale = 0.0;
  short_config.curvature_safety_factor = 0.0;
  short_config.sample_spacing = 0.01;
  auto long_config = short_config;
  long_config.minimum_forward_distance = 2.0;

  const auto short_result = MergeReferenceBuilder::build(
      route, {0.0, 0.3, 0.0}, 0.0, 4.0, 0.0, short_config, kAlive);
  const auto long_result = MergeReferenceBuilder::build(
      route, {0.0, 0.3, 0.0}, 0.0, 4.0, 0.0, long_config, kAlive);
  ASSERT_TRUE(short_result.success());
  ASSERT_TRUE(long_result.success());
  EXPECT_GT(short_result.estimated_max_curvature,
            long_result.estimated_max_curvature);
}

TEST(MergeReferenceBuilder, CurrentSpeedCanIncreaseRequestedMergeDistance)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 0.2;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.speed_lookahead_time = 1.0;
  const auto result = MergeReferenceBuilder::build(
      route, {0.0, 0.1, 0.0}, 0.0, 3.0, 1.23, config, kAlive);

  ASSERT_TRUE(result.success());
  EXPECT_NEAR(result.requested_forward_distance, 1.23, 1.0e-12);
  EXPECT_NEAR(result.actual_forward_distance, 1.23, 1.0e-12);
  const auto *merge_end = sampleAt(result, result.merge_end_route_s);
  ASSERT_NE(merge_end, nullptr);
  EXPECT_NEAR((merge_end->position - route.pointAt(1.23)).norm(),
              0.0, 1.0e-12);
  EXPECT_EQ(merge_end->source, LocalRouteProgressSource::kDirect);
}

TEST(MergeReferenceBuilder, RejectsSeedAboveConfiguredCurvature)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 0.5;
  config.maximum_seed_curvature = 0.1;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.sample_spacing = 0.01;
  const auto result = MergeReferenceBuilder::build(
      route, {0.0, 0.5, 0.0}, 0.0, 2.0, 0.0, config, kAlive);
  EXPECT_EQ(result.status, MergeReferenceStatus::kCurvatureLimit);
  EXPECT_TRUE(result.samples.empty());
}

TEST(MergeReferenceBuilder, EnforcesInputResourceAndDeadlineLimits)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, false);
  auto config = permissiveConfig();
  config.max_samples = 4;
  config.sample_spacing = 0.1;
  EXPECT_EQ(
      MergeReferenceBuilder::build(
          route, {0.0, 0.1, 0.0}, 0.0, 2.0, 0.0,
          config, kAlive).status,
      MergeReferenceStatus::kResourceLimit);
  EXPECT_EQ(
      MergeReferenceBuilder::build(
          route, {0.0, 0.1, 0.0}, 0.0, 2.0, 0.0,
          permissiveConfig(), []() { return true; }).status,
      MergeReferenceStatus::kDeadlineExceeded);
}

TEST(MergeReferenceBuilder, SupportsUnwrappedClosedRouteAcrossSeam)
{
  const ReferenceRoute route(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
       {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}}, true);
  auto config = permissiveConfig();
  config.minimum_forward_distance = 0.4;
  config.corner_clearance = 0.02;
  config.lateral_distance_scale = 0.0;
  config.curvature_safety_factor = 0.0;
  config.corner_angle_threshold = 2.0;
  const double progress_s = route.totalLength() - 0.25;
  ASSERT_TRUE(route.sampleRange(
      progress_s, progress_s + 0.4, config.sample_spacing).success());
  ASSERT_TRUE(route.sampleRange(
      progress_s + 0.4, route.totalLength() + 0.75,
      config.sample_spacing).success());
  const auto result = MergeReferenceBuilder::build(
      route, route.pointAt(progress_s) + Eigen::Vector3d(0.1, 0.0, 0.0),
      progress_s, route.totalLength() + 0.75, 0.0, config, kAlive);
  ASSERT_TRUE(result.success()) << mergeReferenceStatusName(result.status);
  EXPECT_GT(result.samples.back().route_s, route.totalLength());
  EXPECT_GT(result.merge_end_route_s, route.totalLength());
}

TEST(MergeReferenceBuilder, RejectsInvalidConfigurationAndLargeOffset)
{
  const ReferenceRoute route({{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, false);
  auto invalid = permissiveConfig();
  invalid.sample_spacing = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
      MergeReferenceBuilder::build(
          route, {0.0, 0.1, 0.0}, 0.0, 2.0, 0.0,
          invalid, kAlive).status,
      MergeReferenceStatus::kInvalidConfiguration);

  auto bounded = permissiveConfig();
  bounded.maximum_initial_offset = 0.2;
  EXPECT_EQ(
      MergeReferenceBuilder::build(
          route, {0.0, 0.3, 0.0}, 0.0, 2.0, 0.0,
          bounded, kAlive).status,
      MergeReferenceStatus::kOffsetTooLarge);
}

}  // namespace
}  // namespace scan_planner
