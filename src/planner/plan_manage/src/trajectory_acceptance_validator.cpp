// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file trajectory_acceptance_validator.cpp @brief Final bounded B-spline safety validation. */

#include <plan_manage/trajectory_acceptance_validator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace scan_planner
{
namespace
{

bool validConfig(const TrajectoryAcceptanceConfig &config) noexcept
{
  return std::isfinite(config.max_velocity) &&
      config.max_velocity > 0.0 &&
      std::isfinite(config.max_acceleration) &&
      config.max_acceleration > 0.0 &&
      std::isfinite(config.velocity_tolerance) &&
      config.velocity_tolerance >= 0.0 &&
      std::isfinite(config.acceleration_tolerance) &&
      config.acceleration_tolerance >= 0.0 &&
      std::isfinite(config.max_spatial_sample_step) &&
      config.max_spatial_sample_step > 0.0 &&
      std::isfinite(config.max_temporal_sample_step) &&
      config.max_temporal_sample_step > 0.0 &&
      config.max_samples >= 2;
}

TrajectoryAcceptanceResult result(
    TrajectoryAcceptanceStatus status,
    std::size_t sample_index = 0,
    std::size_t sample_count = 0,
    double trajectory_time = 0.0,
    double observed_value = 0.0,
    double limit_value = 0.0,
    int occupancy = 0) noexcept
{
  return {
    status,
    sample_index,
    sample_count,
    trajectory_time,
    observed_value,
    limit_value,
    occupancy};
}

bool deadlineExpired(
    const TrajectoryAcceptanceValidator::DeadlineQuery &query)
{
  return !query || query();
}

TrajectoryAcceptanceResult validateDerivativeControlPoints(
    const Eigen::MatrixXd &control_points,
    double limit,
    TrajectoryAcceptanceStatus limit_status)
{
  if (control_points.rows() != 3 || control_points.cols() < 1 ||
      !control_points.allFinite())
  {
    return result(TrajectoryAcceptanceStatus::kInvalidTrajectory);
  }

  for (Eigen::Index column = 0; column < control_points.cols(); ++column)
  {
    const double norm = control_points.col(column).norm();
    if (!std::isfinite(norm))
      return result(TrajectoryAcceptanceStatus::kInvalidTrajectory);
    if (norm > limit)
    {
      return result(
          limit_status,
          static_cast<std::size_t>(column),
          static_cast<std::size_t>(control_points.cols()),
          0.0,
          norm,
          limit);
    }
  }
  return result(TrajectoryAcceptanceStatus::kAccepted);
}

}  // namespace

const char * trajectoryAcceptanceStatusName(
    TrajectoryAcceptanceStatus status) noexcept
{
  switch (status)
  {
    case TrajectoryAcceptanceStatus::kAccepted:
      return "accepted";
    case TrajectoryAcceptanceStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case TrajectoryAcceptanceStatus::kInvalidTrajectory:
      return "invalid_trajectory";
    case TrajectoryAcceptanceStatus::kResourceLimit:
      return "resource_limit";
    case TrajectoryAcceptanceStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case TrajectoryAcceptanceStatus::kVelocityLimit:
      return "velocity_limit";
    case TrajectoryAcceptanceStatus::kAccelerationLimit:
      return "acceleration_limit";
    case TrajectoryAcceptanceStatus::kCollision:
      return "collision";
  }
  return "unknown";
}

TrajectoryAcceptanceValidator::TrajectoryAcceptanceValidator(
    TrajectoryAcceptanceConfig config)
: config_(std::move(config))
{
}

TrajectoryAcceptanceResult TrajectoryAcceptanceValidator::validate(
    UniformBspline position_traj,
    const CollisionQuery &collision_query,
    const DeadlineQuery &deadline_exceeded) const
{
  if (deadlineExpired(deadline_exceeded))
    return result(TrajectoryAcceptanceStatus::kDeadlineExceeded);
  if (!validConfig(config_) || !collision_query)
    return result(TrajectoryAcceptanceStatus::kInvalidConfiguration);

  const Eigen::MatrixXd position_control_points =
      position_traj.getControlPoint();
  if (position_control_points.rows() != 3 ||
      position_control_points.cols() < 4 ||
      !position_control_points.allFinite() ||
      !std::isfinite(position_traj.getInterval()) ||
      position_traj.getInterval() <= 0.0)
  {
    return result(TrajectoryAcceptanceStatus::kInvalidTrajectory);
  }

  const double duration = position_traj.getTimeSum();
  if (!std::isfinite(duration) || duration <= 0.0)
    return result(TrajectoryAcceptanceStatus::kInvalidTrajectory);

  if (deadlineExpired(deadline_exceeded))
    return result(TrajectoryAcceptanceStatus::kDeadlineExceeded);

  UniformBspline velocity_traj = position_traj.getDerivative();
  UniformBspline acceleration_traj = velocity_traj.getDerivative();
  const double velocity_limit =
      config_.max_velocity + config_.velocity_tolerance;
  const double acceleration_limit =
      config_.max_acceleration + config_.acceleration_tolerance;
  if (!std::isfinite(velocity_limit) || velocity_limit <= 0.0 ||
      !std::isfinite(acceleration_limit) || acceleration_limit <= 0.0)
  {
    return result(TrajectoryAcceptanceStatus::kInvalidConfiguration);
  }

  TrajectoryAcceptanceResult derivative_result =
      validateDerivativeControlPoints(
          velocity_traj.getControlPoint(),
          velocity_limit,
          TrajectoryAcceptanceStatus::kVelocityLimit);
  if (!derivative_result.accepted())
    return derivative_result;
  derivative_result = validateDerivativeControlPoints(
      acceleration_traj.getControlPoint(),
      acceleration_limit,
      TrajectoryAcceptanceStatus::kAccelerationLimit);
  if (!derivative_result.accepted())
    return derivative_result;

  if (deadlineExpired(deadline_exceeded))
    return result(TrajectoryAcceptanceStatus::kDeadlineExceeded);

  const double spatial_time_step =
      config_.max_spatial_sample_step / velocity_limit;
  const double sample_time_step =
      std::min(config_.max_temporal_sample_step, spatial_time_step);
  if (!std::isfinite(sample_time_step) || sample_time_step <= 0.0)
    return result(TrajectoryAcceptanceStatus::kInvalidConfiguration);

  const double segment_count_double = std::ceil(duration / sample_time_step);
  const double maximum_segments =
      static_cast<double>(config_.max_samples - 1);
  if (!std::isfinite(segment_count_double) ||
      segment_count_double > maximum_segments)
  {
    return result(
        TrajectoryAcceptanceStatus::kResourceLimit,
        0,
        config_.max_samples,
        duration,
        segment_count_double,
        maximum_segments);
  }

  const std::size_t segment_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(segment_count_double));
  const std::size_t sample_count = segment_count + 1;
  const double actual_time_step = duration /
      static_cast<double>(segment_count);
  double previous_yaw = 0.0;

  for (std::size_t index = 0; index < sample_count; ++index)
  {
    if (deadlineExpired(deadline_exceeded))
    {
      return result(
          TrajectoryAcceptanceStatus::kDeadlineExceeded,
          index,
          sample_count);
    }

    const double time = index == segment_count
        ? duration
        : actual_time_step * static_cast<double>(index);
    const Eigen::VectorXd position_dynamic =
        position_traj.evaluateDeBoorT(time);
    if (position_dynamic.size() != 3 || !position_dynamic.allFinite())
    {
      return result(
          TrajectoryAcceptanceStatus::kInvalidTrajectory,
          index,
          sample_count,
          time);
    }
    const Eigen::Vector3d position = position_dynamic;

    const double previous_time = std::max(0.0, time - actual_time_step);
    const double next_time = std::min(duration, time + actual_time_step);
    const Eigen::VectorXd previous_position =
        position_traj.evaluateDeBoorT(previous_time);
    const Eigen::VectorXd next_position =
        position_traj.evaluateDeBoorT(next_time);
    if (previous_position.size() != 3 || next_position.size() != 3 ||
        !previous_position.allFinite() || !next_position.allFinite())
    {
      return result(
          TrajectoryAcceptanceStatus::kInvalidTrajectory,
          index,
          sample_count,
          time);
    }

    const Eigen::Vector2d tangent =
        (next_position - previous_position).head<2>();
    const double yaw = tangent.squaredNorm() > 1.0e-12
        ? std::atan2(tangent.y(), tangent.x())
        : previous_yaw;
    if (!std::isfinite(yaw))
    {
      return result(
          TrajectoryAcceptanceStatus::kInvalidTrajectory,
          index,
          sample_count,
          time);
    }
    previous_yaw = yaw;

    const int occupancy = collision_query(position, yaw);
    if (deadlineExpired(deadline_exceeded))
    {
      return result(
          TrajectoryAcceptanceStatus::kDeadlineExceeded,
          index,
          sample_count,
          time);
    }
    if (occupancy != 0)
    {
      return result(
          TrajectoryAcceptanceStatus::kCollision,
          index,
          sample_count,
          time,
          0.0,
          0.0,
          occupancy);
    }
  }

  return result(
      TrajectoryAcceptanceStatus::kAccepted,
      sample_count - 1,
      sample_count,
      duration);
}

}  // namespace scan_planner
