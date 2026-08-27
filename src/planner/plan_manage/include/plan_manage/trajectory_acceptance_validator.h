// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file trajectory_acceptance_validator.h @brief Final bounded B-spline safety validation. */

#ifndef PLAN_MANAGE__TRAJECTORY_ACCEPTANCE_VALIDATOR_H_
#define PLAN_MANAGE__TRAJECTORY_ACCEPTANCE_VALIDATOR_H_

#include <bspline_opt/uniform_bspline.h>

#include <Eigen/Core>

#include <cstddef>
#include <functional>

namespace scan_planner
{

/** @brief Hard limits used by final trajectory acceptance. */
struct TrajectoryAcceptanceConfig
{
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double velocity_tolerance{0.0};
  double acceleration_tolerance{0.0};
  double max_spatial_sample_step{0.0};
  double max_temporal_sample_step{0.01};
  std::size_t max_samples{4096};
};

/** @brief Structured outcome of final trajectory acceptance. */
enum class TrajectoryAcceptanceStatus
{
  kAccepted,
  kInvalidConfiguration,
  kInvalidTrajectory,
  kResourceLimit,
  kDeadlineExceeded,
  kVelocityLimit,
  kAccelerationLimit,
  kCollision
};

/** @brief Diagnostic detail for a final validation decision. */
struct TrajectoryAcceptanceResult
{
  TrajectoryAcceptanceStatus status{TrajectoryAcceptanceStatus::kInvalidTrajectory};
  std::size_t sample_index{0};
  std::size_t sample_count{0};
  double trajectory_time{0.0};
  double observed_value{0.0};
  double limit_value{0.0};
  int occupancy{0};

  /** @brief Returns true only for a fully accepted trajectory. */
  bool accepted() const noexcept
  {
    return status == TrajectoryAcceptanceStatus::kAccepted;
  }
};

/** @brief Returns a stable diagnostic name for a validation status. */
const char * trajectoryAcceptanceStatusName(
    TrajectoryAcceptanceStatus status) noexcept;

/**
 * @brief Validates a complete candidate without modifying planner state.
 *
 * Dynamic limits are checked conservatively against every derivative control
 * point. Collision validation samples the complete curve with both a temporal
 * and a velocity-derived spatial bound. Any nonzero collision-query result,
 * including unknown or out-of-map, is rejected.
 */
class TrajectoryAcceptanceValidator final
{
public:
  using CollisionQuery =
      std::function<int(const Eigen::Vector3d &, double)>;
  using DeadlineQuery = std::function<bool()>;

  /** @brief Constructs a validator with immutable limits. */
  explicit TrajectoryAcceptanceValidator(TrajectoryAcceptanceConfig config);

  /**
   * @brief Runs finite-value, resource, dynamics, and collision checks.
   * @param position_traj Candidate cubic position B-spline.
   * @param collision_query Double-cylinder occupancy query; zero means free.
   * @param deadline_exceeded Shared absolute-deadline predicate.
   */
  TrajectoryAcceptanceResult validate(
      UniformBspline position_traj,
      const CollisionQuery &collision_query,
      const DeadlineQuery &deadline_exceeded) const;

private:
  TrajectoryAcceptanceConfig config_;
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__TRAJECTORY_ACCEPTANCE_VALIDATOR_H_
