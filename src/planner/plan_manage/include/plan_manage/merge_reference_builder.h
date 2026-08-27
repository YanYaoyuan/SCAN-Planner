// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file merge_reference_builder.h @brief Bounded route merge-seed construction. */

#ifndef PLAN_MANAGE__MERGE_REFERENCE_BUILDER_H_
#define PLAN_MANAGE__MERGE_REFERENCE_BUILDER_H_

#include <plan_manage/local_route_progress_map.h>
#include <plan_manage/reference_route.h>

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <vector>

namespace scan_planner
{

/** @brief Limits and heuristics for one merge-reference construction. */
struct MergeReferenceConfig
{
  double minimum_forward_distance{0.50};
  double lateral_distance_scale{1.0};
  double speed_lookahead_time{0.50};
  double curvature_safety_factor{6.0};
  double maximum_forward_distance{3.0};
  double maximum_initial_offset{2.0};
  double maximum_seed_curvature{3.0};
  double sample_spacing{0.05};
  double zero_offset_tolerance{0.01};
  double corner_clearance{0.05};
  double corner_angle_threshold{0.08726646259971647};
  std::size_t max_corner_extensions{32};
  std::size_t max_samples{4096};
};

/** @brief One ordered point in a merge seed. */
struct MergeReferenceSample
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::Zero()};
  double route_s{0.0};
  bool is_original_route_vertex{false};
  LocalRouteProgressSource source{LocalRouteProgressSource::kMerge};
};

/** @brief Structured outcome of merge-reference construction. */
enum class MergeReferenceStatus
{
  kSuccess,
  kInvalidConfiguration,
  kInvalidInput,
  kOutOfRange,
  kOffsetTooLarge,
  kStartNearUnsmoothedCorner,
  kInsufficientLookahead,
  kCannotAvoidCorner,
  kCurvatureLimit,
  kResourceLimit,
  kDeadlineExceeded,
  kRouteSamplingFailed
};

/** @brief Immutable candidate result; no planner state is modified. */
struct MergeReferenceResult
{
  MergeReferenceStatus status{MergeReferenceStatus::kInvalidInput};
  std::vector<MergeReferenceSample> samples;
  double merge_start_route_s{0.0};
  double merge_end_route_s{0.0};
  double requested_forward_distance{0.0};
  double actual_forward_distance{0.0};
  double estimated_max_curvature{0.0};
  bool clipped_by_local_target{false};
  bool crosses_unsmoothed_corner{false};
  bool no_merge_required{false};

  /** @brief Returns true only when a complete candidate seed was produced. */
  bool success() const noexcept
  {
    return status == MergeReferenceStatus::kSuccess && samples.size() >= 2;
  }
};

/** @brief Returns a stable diagnostic name for a builder status. */
const char * mergeReferenceStatusName(MergeReferenceStatus status) noexcept;

/**
 * @brief Builds a bounded quintic offset-fade seed over an immutable route.
 *
 * The fade has zero first and second offset derivatives at both ends.  This
 * does not make a polyline containing an original corner globally C2; such
 * corners are retained and reported in the result.  The caller must still run
 * final B-spline collision, curvature, and dynamics acceptance.
 */
class MergeReferenceBuilder final
{
public:
  using DeadlineQuery = std::function<bool()>;

  /**
   * @brief Builds a seed from current_position back to the confirmed route.
   * @param route Immutable mission route.
   * @param current_position Current body position in the route frame.
   * @param progress_route_s Confirmed, possibly unwrapped route progress.
   * @param local_target_route_s Ordered local target after progress_route_s.
   * @param current_speed Non-negative body speed in metres per second.
   * @param config Validated construction limits.
   * @param deadline_exceeded Shared absolute planning-deadline predicate.
   */
  static MergeReferenceResult build(
      const ReferenceRoute &route,
      const Eigen::Vector3d &current_position,
      double progress_route_s,
      double local_target_route_s,
      double current_speed,
      const MergeReferenceConfig &config,
      const DeadlineQuery &deadline_exceeded);
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__MERGE_REFERENCE_BUILDER_H_
