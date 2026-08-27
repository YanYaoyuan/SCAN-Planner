// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file reference_route.h @brief Immutable arc-length reference-route geometry. */

#ifndef PLAN_MANAGE__REFERENCE_ROUTE_H_
#define PLAN_MANAGE__REFERENCE_ROUTE_H_

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <vector>

namespace scan_planner
{

/** @brief One geometry sample annotated in the route's arc-length domain. */
struct ReferenceRouteSample
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::Zero()};
  double route_s{0.0};
  bool is_original_vertex{false};
};

/** @brief Structured status for bounded route-range sampling. */
enum class ReferenceRouteRangeStatus
{
  kSuccess,
  kInvalidInput,
  kOutOfRange,
  kResourceLimit,
  kDeadlineExceeded
};

/** @brief Result of sampling an ordered route interval. */
struct ReferenceRouteRangeResult
{
  ReferenceRouteRangeStatus status{ReferenceRouteRangeStatus::kInvalidInput};
  std::vector<ReferenceRouteSample> samples;

  /** @brief Returns true only when the complete requested range was sampled. */
  bool success() const noexcept
  {
    return status == ReferenceRouteRangeStatus::kSuccess;
  }
};

/** @brief Returns a stable diagnostic name for a range-sampling status. */
const char * referenceRouteRangeStatusName(
    ReferenceRouteRangeStatus status) noexcept;

/**
 * @brief Immutable, deduplicated polyline in a metric arc-length coordinate.
 *
 * Open routes accept only route_s in [0, totalLength()]. Closed routes remove
 * an optional repeated terminal vertex, include the closing segment, and map
 * any finite unwrapped route_s onto the corresponding lap.
 */
class ReferenceRoute final
{
public:
  using DeadlineQuery = std::function<bool()>;

  /** @brief Construction and sampling resource limits. */
  struct Config
  {
    double duplicate_tolerance{1.0e-9};
    std::size_t max_sample_count{100000};
  };

  /** @brief Constructs and validates a route with default limits. */
  ReferenceRoute(
      const std::vector<Eigen::Vector3d> &points,
      bool closed);

  /** @brief Constructs and validates a route with explicit limits. */
  ReferenceRoute(
      const std::vector<Eigen::Vector3d> &points,
      bool closed,
      const Config &config);

  /** @brief Returns whether this route is topologically closed. */
  bool closed() const noexcept;

  /** @brief Returns the route length in metres, including the closing seam. */
  double totalLength() const noexcept;

  /** @brief Returns the cleaned original vertices without a repeated seam point. */
  const std::vector<Eigen::Vector3d> & points() const noexcept;

  /** @brief Returns each original vertex's arc length within the first lap. */
  const std::vector<double> & cumulativeLengths() const noexcept;

  /**
   * @brief Converts a finite route coordinate into the first-lap domain.
   * @throws std::out_of_range For an open-route coordinate outside its bounds.
   * @throws std::invalid_argument For a non-finite coordinate.
   */
  double normalizeRouteS(double route_s) const;

  /** @brief Interpolates position at a route coordinate. */
  Eigen::Vector3d pointAt(double route_s) const;

  /** @brief Returns the forward unit tangent at a route coordinate. */
  Eigen::Vector3d tangentAt(double route_s) const;

  /**
   * @brief Samples an ordered interval while preserving every original corner.
   *
   * For a closed route, start_s and end_s remain unwrapped in the result, so a
   * range may cross the seam without losing lap identity.
   */
  ReferenceRouteRangeResult sampleRange(
      double start_s,
      double end_s,
      double spacing,
      const DeadlineQuery &deadline_exceeded = {}) const;

  /** @brief Finds the first-lap segment containing a route coordinate. */
  std::size_t segmentIndex(double route_s) const;

private:
  bool closed_{false};
  Config config_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> cumulative_lengths_;
  double total_length_{0.0};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__REFERENCE_ROUTE_H_
