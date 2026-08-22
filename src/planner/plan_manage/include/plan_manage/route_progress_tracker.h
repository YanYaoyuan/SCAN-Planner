// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file route_progress_tracker.h @brief Topology-bounded route progress tracking. */

#ifndef PLAN_MANAGE__ROUTE_PROGRESS_TRACKER_H_
#define PLAN_MANAGE__ROUTE_PROGRESS_TRACKER_H_

#include <plan_manage/reference_route.h>

#include <Eigen/Core>

#include <cstddef>
#include <memory>

namespace scan_planner
{

/** @brief Projection, motion-credit, and loop-completion limits. */
struct RouteProgressConfig
{
  double search_forward_distance{2.0};
  double search_backward_distance{0.25};
  double max_projection_advance{1.0};
  double max_lateral_distance{0.75};
  double max_heading_error{1.2};
  double max_height_error{0.75};
  double movement_credit_scale{1.5};
  double movement_deadband{0.02};
  double initial_credit{0.10};
  double loop_seam_gate_distance{0.30};
  std::size_t max_segment_checks{4096};
  int loop_count{1};  ///< Zero means unlimited laps for a closed route.
};

/** @brief Structured outcome of one odometry-to-route projection. */
enum class RouteProjectionStatus
{
  kAccepted,
  kCompleted,
  kInactive,
  kInvalidInput,
  kNoCandidate,
  kLateralGate,
  kHeadingGate,
  kHeightGate,
  kMovementBudget,
  kSeamGate,
  kResourceLimit
};

/** @brief Projection decision and diagnostics for one update. */
struct RouteProjectionResult
{
  RouteProjectionStatus status{RouteProjectionStatus::kInactive};
  double previous_route_s{0.0};
  double candidate_route_s{0.0};
  double accepted_route_s{0.0};
  double lateral_distance{0.0};
  double heading_error{0.0};
  double height_error{0.0};
  double available_movement_credit{0.0};
  std::size_t checked_segments{0};

  /** @brief Returns true when the update preserved or advanced valid progress. */
  bool accepted() const noexcept
  {
    return status == RouteProjectionStatus::kAccepted ||
        status == RouteProjectionStatus::kCompleted;
  }
};

/** @brief Returns a stable diagnostic name for a projection status. */
const char * routeProjectionStatusName(RouteProjectionStatus status) noexcept;

/**
 * @brief Tracks monotonic unwrapped progress on an immutable ReferenceRoute.
 *
 * Geometry/topology chooses exactly one best projection candidate before the
 * movement budget is checked. If that candidate exceeds the available credit,
 * the whole update is rejected; the tracker never falls back to an older
 * segment merely to make the budget pass.
 */
class RouteProgressTracker final
{
public:
  /** @brief Constructs a tracker with validated configuration. */
  explicit RouteProgressTracker(
      const RouteProgressConfig &config = RouteProgressConfig{});

  /** @brief Replaces configuration after validation and clears active state. */
  void setConfig(const RouteProgressConfig &config);

  /** @brief Starts a route at unwrapped route_s zero. */
  void reset(
      std::shared_ptr<const ReferenceRoute> route,
      const Eigen::Vector3d &odom_position);

  /** @brief Clears route ownership and all progress/motion state. */
  void clear() noexcept;

  /** @brief Returns whether a valid route is active. */
  bool active() const noexcept;

  /** @brief Returns confirmed monotonic unwrapped progress in metres. */
  double progress() const noexcept;

  /** @brief Returns whether the configured finite mission has completed. */
  bool completed() const noexcept;

  /** @brief Returns currently unused movement credit in route metres. */
  double movementCredit() const noexcept;

  /**
   * @brief Projects a new odometry sample using an explicit body-forward vector.
   * @param odom_position Finite position in the route frame.
   * @param body_forward Finite forward direction in the same frame.
   */
  RouteProjectionResult update(
      const Eigen::Vector3d &odom_position,
      const Eigen::Vector3d &body_forward);

private:
  RouteProgressConfig config_;
  std::shared_ptr<const ReferenceRoute> route_;
  Eigen::Vector3d last_odom_position_{Eigen::Vector3d::Zero()};
  double progress_{0.0};
  double movement_credit_{0.0};
  bool have_last_odom_{false};
  bool completed_{false};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__ROUTE_PROGRESS_TRACKER_H_
