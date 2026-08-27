// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file local_route_progress_map.h @brief Immutable local_s to route_s mapping. */

#ifndef PLAN_MANAGE__LOCAL_ROUTE_PROGRESS_MAP_H_
#define PLAN_MANAGE__LOCAL_ROUTE_PROGRESS_MAP_H_

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <vector>

namespace scan_planner
{

/** @brief Geometry source responsible for a mapping sample. */
enum class LocalRouteProgressSource
{
  kDirect,
  kMerge,
  kAvoidance,
  kTransition
};

/** @brief One immutable local trajectory to mission-route correspondence. */
struct LocalRouteProgressSample
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double local_s{0.0};
  double route_s{0.0};
  LocalRouteProgressSource source{LocalRouteProgressSource::kDirect};
};

/** @brief Structured map-construction status. */
enum class LocalRouteProgressMapStatus
{
  kSuccess,
  kInvalidInput,
  kNonFinite,
  kNonMonotonic,
  kInconsistentArcLength,
  kZeroLength,
  kResourceLimit
};

class LocalRouteProgressMap;

/** @brief Result of creating an invariant-checked immutable map. */
struct LocalRouteProgressMapResult
{
  LocalRouteProgressMapStatus status{LocalRouteProgressMapStatus::kInvalidInput};
  std::shared_ptr<const LocalRouteProgressMap> map;

  /** @brief Returns true only when a complete valid map was created. */
  bool success() const noexcept
  {
    return status == LocalRouteProgressMapStatus::kSuccess &&
        static_cast<bool>(map);
  }
};

/** @brief Structured interpolation status. */
enum class LocalRouteProgressLookupStatus
{
  kSuccess,
  kInvalidInput,
  kOutOfRange
};

/** @brief Interpolated route progress and source interval. */
struct LocalRouteProgressLookup
{
  LocalRouteProgressLookupStatus status{
      LocalRouteProgressLookupStatus::kInvalidInput};
  double local_s{0.0};
  double route_s{0.0};
  std::size_t segment_index{0};
  LocalRouteProgressSource source{LocalRouteProgressSource::kDirect};

  /** @brief Returns true when the requested coordinate was mapped. */
  bool success() const noexcept
  {
    return status == LocalRouteProgressLookupStatus::kSuccess;
  }
};

/** @brief Returns a stable diagnostic name for map construction. */
const char * localRouteProgressMapStatusName(
    LocalRouteProgressMapStatus status) noexcept;

/**
 * @brief Immutable, strictly ordered local_s to monotonic route_s mapping.
 *
 * local_s is planar arc length measured on the actual candidate trajectory.
 * route_s is mission progress entitlement, not an unconstrained nearest-point
 * projection onto the permanent route.
 */
class LocalRouteProgressMap final
{
public:
  /** @brief Validates externally annotated samples and creates a map. */
  static LocalRouteProgressMapResult create(
      std::vector<LocalRouteProgressSample> samples,
      std::size_t max_samples = 4096);

  /**
   * @brief Creates an anchored monotonic map for a local polyline interval.
   *
   * Interior route_s is allocated by actual planar local arc length and is
   * clamped between the exact entry and exit anchors.
   */
  static LocalRouteProgressMapResult fromAnchoredPolyline(
      const std::vector<Eigen::Vector3d> &positions,
      double route_s_entry,
      double route_s_exit,
      LocalRouteProgressSource source,
      std::size_t max_samples = 4096);

  /** @brief Returns all validated samples. */
  const std::vector<LocalRouteProgressSample> & samples() const noexcept;

  /** @brief Returns total planar local trajectory length. */
  double totalLocalLength() const noexcept;

  /** @brief Returns the exact route entry anchor. */
  double routeEntry() const noexcept;

  /** @brief Returns the exact route exit anchor. */
  double routeExit() const noexcept;

  /** @brief Interpolates route_s at a bounded local_s coordinate. */
  LocalRouteProgressLookup routeSAt(double local_s) const noexcept;

private:
  explicit LocalRouteProgressMap(
      std::vector<LocalRouteProgressSample> samples);

  std::vector<LocalRouteProgressSample> samples_;
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__LOCAL_ROUTE_PROGRESS_MAP_H_
