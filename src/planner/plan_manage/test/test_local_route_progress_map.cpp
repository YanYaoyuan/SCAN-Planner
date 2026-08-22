// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_local_route_progress_map.cpp @brief Local-to-route progress map tests. */

#include <gtest/gtest.h>

#include <plan_manage/local_route_progress_map.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <vector>

namespace scan_planner
{
namespace
{

TEST(LocalRouteProgressMap, CoincidentStraightRouteHasIdentityMapping)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      0.0,
      2.0,
      LocalRouteProgressSource::kDirect);

  ASSERT_TRUE(result.success());
  const auto lookup = result.map->routeSAt(1.25);
  ASSERT_TRUE(lookup.success());
  EXPECT_NEAR(lookup.route_s, 1.25, 1.0e-12);
}

TEST(LocalRouteProgressMap, NonzeroEntryAppliesConstantStraightOffset)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      5.0,
      7.0,
      LocalRouteProgressSource::kMerge);

  ASSERT_TRUE(result.success());
  EXPECT_NEAR(result.map->routeSAt(0.5).route_s, 5.5, 1.0e-12);
  EXPECT_NEAR(result.map->routeSAt(1.5).route_s, 6.5, 1.0e-12);
}

TEST(LocalRouteProgressMap, LongerAvoidancePathKeepsExactMonotonicAnchors)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0},
       {0.0, 1.0, 0.0},
       {1.0, 1.0, 0.0},
       {1.0, 0.0, 0.0}},
      10.0,
      11.0,
      LocalRouteProgressSource::kAvoidance);

  ASSERT_TRUE(result.success());
  EXPECT_DOUBLE_EQ(result.map->routeEntry(), 10.0);
  EXPECT_DOUBLE_EQ(result.map->routeExit(), 11.0);
  EXPECT_NEAR(result.map->totalLocalLength(), 3.0, 1.0e-12);
  EXPECT_NEAR(result.map->routeSAt(1.5).route_s, 10.5, 1.0e-12);
  double previous_route_s = result.map->routeEntry();
  for (const auto &sample : result.map->samples())
  {
    EXPECT_GE(sample.route_s, previous_route_s);
    previous_route_s = sample.route_s;
  }
}

TEST(LocalRouteProgressMap, AllowsHoldingMissionProgressAcrossTransition)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}},
      7.0,
      7.0,
      LocalRouteProgressSource::kTransition);

  ASSERT_TRUE(result.success());
  EXPECT_DOUBLE_EQ(result.map->routeEntry(), 7.0);
  EXPECT_DOUBLE_EQ(result.map->routeExit(), 7.0);
  EXPECT_DOUBLE_EQ(result.map->routeSAt(1.0).route_s, 7.0);
  EXPECT_DOUBLE_EQ(result.map->routeSAt(2.0).route_s, 7.0);
}

TEST(LocalRouteProgressMap, SelfIntersectionDoesNotChangeArcLengthIdentity)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0},
       {1.0, 1.0, 0.0},
       {0.0, 0.0, 0.0},
       {1.0, -1.0, 0.0}},
      20.0,
      23.0,
      LocalRouteProgressSource::kAvoidance);

  ASSERT_TRUE(result.success());
  const double crossing_again_local_s = 2.0 * std::sqrt(2.0);
  const auto crossing_again = result.map->routeSAt(crossing_again_local_s);
  ASSERT_TRUE(crossing_again.success());
  EXPECT_GT(crossing_again.route_s, result.map->routeSAt(0.0).route_s);
}

TEST(LocalRouteProgressMap, RejectsDuplicateLocalSDecreasingRouteAndNaN)
{
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const Eigen::Vector3d one = Eigen::Vector3d::UnitX();
  EXPECT_EQ(
      LocalRouteProgressMap::create(
          {{zero, 0.0, 1.0, LocalRouteProgressSource::kDirect},
           {one, 0.0, 2.0, LocalRouteProgressSource::kDirect}}).status,
      LocalRouteProgressMapStatus::kNonMonotonic);
  EXPECT_EQ(
      LocalRouteProgressMap::create(
          {{zero, 0.0, 2.0, LocalRouteProgressSource::kDirect},
           {one, 1.0, 1.0, LocalRouteProgressSource::kDirect}}).status,
      LocalRouteProgressMapStatus::kNonMonotonic);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
      LocalRouteProgressMap::create(
          {{zero, 0.0, 1.0, LocalRouteProgressSource::kDirect},
           {one, 1.0, nan, LocalRouteProgressSource::kDirect}}).status,
      LocalRouteProgressMapStatus::kNonFinite);
}

TEST(LocalRouteProgressMap, RejectsDishonestLocalArcLength)
{
  const auto result = LocalRouteProgressMap::create(
      {{{0.0, 0.0, 0.0}, 0.0, 1.0, LocalRouteProgressSource::kDirect},
       {{1.0, 0.0, 0.0}, 5.0, 2.0, LocalRouteProgressSource::kDirect}});

  EXPECT_EQ(
      result.status,
      LocalRouteProgressMapStatus::kInconsistentArcLength);
}

TEST(LocalRouteProgressMap, DuplicatePositionsAreRemovedByAnchoredBuilder)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0},
       {0.0, 0.0, 0.1},
       {1.0, 0.0, 0.1}},
      0.0,
      1.0,
      LocalRouteProgressSource::kTransition);

  ASSERT_TRUE(result.success());
  ASSERT_EQ(result.map->samples().size(), 2u);
  EXPECT_EQ(
      result.map->samples().back().source,
      LocalRouteProgressSource::kTransition);
}

TEST(LocalRouteProgressMap, LookupRejectsOutOfRangeInsteadOfClampingSilently)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      2.0,
      3.0,
      LocalRouteProgressSource::kDirect);
  ASSERT_TRUE(result.success());

  EXPECT_EQ(
      result.map->routeSAt(-0.01).status,
      LocalRouteProgressLookupStatus::kOutOfRange);
  EXPECT_EQ(
      result.map->routeSAt(1.01).status,
      LocalRouteProgressLookupStatus::kOutOfRange);
}

TEST(LocalRouteProgressMap, EnforcesHardInputSampleLimit)
{
  const auto result = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
      0.0,
      2.0,
      LocalRouteProgressSource::kDirect,
      2);

  EXPECT_EQ(result.status, LocalRouteProgressMapStatus::kResourceLimit);
}

TEST(LocalRouteProgressMap, RejectsUnknownGeometrySource)
{
  const auto invalid_source = static_cast<LocalRouteProgressSource>(99);
  const auto built = LocalRouteProgressMap::fromAnchoredPolyline(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      0.0,
      1.0,
      invalid_source);
  EXPECT_EQ(built.status, LocalRouteProgressMapStatus::kInvalidInput);

  const auto annotated = LocalRouteProgressMap::create(
      {{{0.0, 0.0, 0.0}, 0.0, 0.0, LocalRouteProgressSource::kDirect},
       {{1.0, 0.0, 0.0}, 1.0, 1.0, invalid_source}});
  EXPECT_EQ(annotated.status, LocalRouteProgressMapStatus::kInvalidInput);
}

}  // namespace
}  // namespace scan_planner
