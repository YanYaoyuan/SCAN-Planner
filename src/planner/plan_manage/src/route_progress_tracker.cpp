// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file route_progress_tracker.cpp @brief Topology-bounded route progress tracking. */

#include <plan_manage/route_progress_tracker.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace scan_planner
{
namespace
{

constexpr double kProgressTolerance = 1.0e-9;

bool validConfig(const RouteProgressConfig &config) noexcept
{
  return std::isfinite(config.search_forward_distance) &&
      config.search_forward_distance > 0.0 &&
      std::isfinite(config.search_backward_distance) &&
      config.search_backward_distance >= 0.0 &&
      std::isfinite(config.max_projection_advance) &&
      config.max_projection_advance > 0.0 &&
      config.search_forward_distance + kProgressTolerance >=
          config.max_projection_advance &&
      std::isfinite(config.max_lateral_distance) &&
      config.max_lateral_distance > 0.0 &&
      std::isfinite(config.max_heading_error) &&
      config.max_heading_error > 0.0 &&
      config.max_heading_error <= std::acos(-1.0) &&
      std::isfinite(config.max_height_error) &&
      config.max_height_error >= 0.0 &&
      std::isfinite(config.movement_credit_scale) &&
      config.movement_credit_scale >= 1.0 &&
      std::isfinite(config.movement_deadband) &&
      config.movement_deadband >= 0.0 &&
      std::isfinite(config.initial_credit) &&
      config.initial_credit >= 0.0 &&
      config.initial_credit <= config.max_projection_advance &&
      std::isfinite(config.loop_seam_gate_distance) &&
      config.loop_seam_gate_distance > 0.0 &&
      config.max_segment_checks >= 1 &&
      config.max_segment_checks <= 1000000 &&
      config.loop_count >= 0 &&
      config.loop_count <= 1000000;
}

double planarHeadingError(
    const Eigen::Vector2d &forward,
    const Eigen::Vector3d &tangent) noexcept
{
  const Eigen::Vector2d tangent_xy = tangent.head<2>();
  const double tangent_norm = tangent_xy.norm();
  if (!std::isfinite(tangent_norm) || tangent_norm <= 1.0e-12)
    return std::numeric_limits<double>::infinity();
  return std::acos(std::clamp(
      forward.dot(tangent_xy / tangent_norm), -1.0, 1.0));
}

struct ProjectionCandidate
{
  double route_s{0.0};
  double lateral_distance{std::numeric_limits<double>::infinity()};
  double heading_error{std::numeric_limits<double>::infinity()};
  double height_error{std::numeric_limits<double>::infinity()};
  double score{std::numeric_limits<double>::infinity()};
};

RouteProjectionResult rejection(
    RouteProjectionStatus status,
    double previous_route_s,
    double movement_credit,
    std::size_t checked_segments,
    const ProjectionCandidate *candidate = nullptr) noexcept
{
  RouteProjectionResult output;
  output.status = status;
  output.previous_route_s = previous_route_s;
  output.accepted_route_s = previous_route_s;
  output.available_movement_credit = movement_credit;
  output.checked_segments = checked_segments;
  if (candidate != nullptr)
  {
    output.candidate_route_s = candidate->route_s;
    output.lateral_distance = candidate->lateral_distance;
    output.heading_error = candidate->heading_error;
    output.height_error = candidate->height_error;
  }
  return output;
}

}  // namespace

const char * routeProjectionStatusName(RouteProjectionStatus status) noexcept
{
  switch (status)
  {
    case RouteProjectionStatus::kAccepted:
      return "accepted";
    case RouteProjectionStatus::kCompleted:
      return "completed";
    case RouteProjectionStatus::kInactive:
      return "inactive";
    case RouteProjectionStatus::kInvalidInput:
      return "invalid_input";
    case RouteProjectionStatus::kNoCandidate:
      return "no_candidate";
    case RouteProjectionStatus::kLateralGate:
      return "lateral_gate";
    case RouteProjectionStatus::kHeadingGate:
      return "heading_gate";
    case RouteProjectionStatus::kHeightGate:
      return "height_gate";
    case RouteProjectionStatus::kMovementBudget:
      return "movement_budget";
    case RouteProjectionStatus::kSeamGate:
      return "seam_gate";
    case RouteProjectionStatus::kResourceLimit:
      return "resource_limit";
  }
  return "unknown";
}

RouteProgressTracker::RouteProgressTracker(const RouteProgressConfig &config)
{
  setConfig(config);
}

void RouteProgressTracker::setConfig(const RouteProgressConfig &config)
{
  if (!validConfig(config))
    throw std::invalid_argument("invalid route-progress configuration");
  config_ = config;
  clear();
}

void RouteProgressTracker::reset(
    std::shared_ptr<const ReferenceRoute> route,
    const Eigen::Vector3d &odom_position)
{
  if (!route || !odom_position.allFinite())
  {
    clear();
    throw std::invalid_argument("route progress requires finite route and odometry");
  }
  if (route->closed() &&
      2.0 * config_.loop_seam_gate_distance >= route->totalLength())
  {
    clear();
    throw std::invalid_argument(
        "loop seam gate must be smaller than half the closed route length");
  }
  if (route->closed() && config_.loop_count > 0 &&
      !std::isfinite(
          route->totalLength() * static_cast<double>(config_.loop_count)))
  {
    clear();
    throw std::invalid_argument("configured closed-route mission length overflows");
  }
  route_ = std::move(route);
  last_odom_position_ = odom_position;
  progress_ = 0.0;
  movement_credit_ = config_.initial_credit;
  have_last_odom_ = true;
  completed_ = false;
}

void RouteProgressTracker::clear() noexcept
{
  route_.reset();
  last_odom_position_.setZero();
  progress_ = 0.0;
  movement_credit_ = 0.0;
  have_last_odom_ = false;
  completed_ = false;
}

bool RouteProgressTracker::active() const noexcept
{
  return static_cast<bool>(route_);
}

double RouteProgressTracker::progress() const noexcept
{
  return progress_;
}

bool RouteProgressTracker::completed() const noexcept
{
  return completed_;
}

double RouteProgressTracker::movementCredit() const noexcept
{
  return movement_credit_;
}

RouteProjectionResult RouteProgressTracker::update(
    const Eigen::Vector3d &odom_position,
    const Eigen::Vector3d &body_forward)
{
  if (!route_)
    return rejection(RouteProjectionStatus::kInactive, progress_, 0.0, 0);
  if (!odom_position.allFinite() || !body_forward.allFinite())
  {
    return rejection(
        RouteProjectionStatus::kInvalidInput,
        progress_, movement_credit_, 0);
  }
  Eigen::Vector2d forward_xy = body_forward.head<2>();
  const double forward_norm = forward_xy.norm();
  if (!std::isfinite(forward_norm) || forward_norm <= 1.0e-12)
  {
    return rejection(
        RouteProjectionStatus::kInvalidInput,
        progress_, movement_credit_, 0);
  }
  forward_xy /= forward_norm;

  if (completed_)
  {
    RouteProjectionResult output = rejection(
        RouteProjectionStatus::kCompleted,
        progress_, movement_credit_, 0);
    output.candidate_route_s = progress_;
    return output;
  }

  Eigen::Vector3d odom_delta = Eigen::Vector3d::Zero();
  bool positive_motion_evidence = false;
  if (have_last_odom_)
  {
    odom_delta = odom_position - last_odom_position_;
    const double displacement = odom_delta.head<2>().norm();
    if (std::isfinite(displacement) &&
        displacement >= config_.movement_deadband)
    {
      movement_credit_ = std::min(
          config_.max_projection_advance,
          movement_credit_ + config_.movement_credit_scale * displacement);
      last_odom_position_ = odom_position;
      positive_motion_evidence = displacement > 0.0;
    }
  }
  else
  {
    last_odom_position_ = odom_position;
    have_last_odom_ = true;
  }

  const double route_length = route_->totalLength();
  const bool closed = route_->closed();
  const double maximum_progress = closed && config_.loop_count == 0
      ? std::numeric_limits<double>::infinity()
      : route_length * static_cast<double>(closed ? config_.loop_count : 1);
  const double search_start = closed
      ? progress_ - config_.search_backward_distance
      : std::max(0.0, progress_ - config_.search_backward_distance);
  double search_end = progress_ + config_.search_forward_distance;
  if (closed)
    search_end = std::min(search_end, progress_ + route_length);
  search_end = std::min(search_end, maximum_progress);
  if (search_end < search_start + kProgressTolerance)
  {
    completed_ = std::isfinite(maximum_progress) &&
        progress_ >= maximum_progress - kProgressTolerance;
    return rejection(
        completed_ ? RouteProjectionStatus::kCompleted :
                     RouteProjectionStatus::kNoCandidate,
        progress_, movement_credit_, 0);
  }

  const auto &vertices = route_->points();
  const auto &vertex_s = route_->cumulativeLengths();
  std::size_t segment = route_->segmentIndex(search_start);
  double lap_start = closed
      ? std::floor(search_start / route_length) * route_length
      : 0.0;
  double segment_start_s = lap_start + vertex_s[segment];
  if (!closed)
    segment_start_s = vertex_s[segment];

  ProjectionCandidate best_valid;
  ProjectionCandidate best_lateral;
  ProjectionCandidate best_heading;
  ProjectionCandidate best_height;
  bool have_valid = false;
  bool have_lateral = false;
  bool have_heading = false;
  bool have_height = false;
  std::size_t checked_segments = 0;

  while (segment_start_s <= search_end + kProgressTolerance)
  {
    if (++checked_segments > config_.max_segment_checks)
    {
      return rejection(
          RouteProjectionStatus::kResourceLimit,
          progress_, movement_credit_, checked_segments - 1);
    }

    const std::size_t next = segment + 1 < vertices.size() ? segment + 1 : 0;
    double segment_end_s = 0.0;
    if (segment + 1 < vertex_s.size())
      segment_end_s = lap_start + vertex_s[segment + 1];
    else if (closed)
      segment_end_s = lap_start + route_length;
    else
      segment_end_s = route_length;

    const double overlap_start = std::max(search_start, segment_start_s);
    const double overlap_end = std::min(search_end, segment_end_s);
    if (overlap_end >= overlap_start - kProgressTolerance)
    {
      const Eigen::Vector3d start = route_->pointAt(overlap_start);
      const Eigen::Vector3d end = route_->pointAt(overlap_end);
      const Eigen::Vector2d delta_xy = end.head<2>() - start.head<2>();
      const double planar_length_squared = delta_xy.squaredNorm();
      double ratio = 0.0;
      if (planar_length_squared > 1.0e-12)
      {
        ratio = std::clamp(
            (odom_position.head<2>() - start.head<2>()).dot(delta_xy) /
                planar_length_squared,
            0.0,
            1.0);
      }
      const Eigen::Vector3d projected =
          start + ratio * (end - start);
      ProjectionCandidate candidate;
      candidate.route_s = overlap_start + ratio * (overlap_end - overlap_start);
      candidate.lateral_distance =
          (projected.head<2>() - odom_position.head<2>()).norm();
      candidate.height_error = std::abs(projected.z() - odom_position.z());
      candidate.heading_error = planarHeadingError(
          forward_xy,
          (vertices[next] - vertices[segment]).normalized());
      const double normalized_lateral =
          candidate.lateral_distance / config_.max_lateral_distance;
      const double normalized_heading =
          candidate.heading_error / config_.max_heading_error;
      const double normalized_topology = std::abs(candidate.route_s - progress_) /
          std::max(config_.search_forward_distance,
                   config_.search_backward_distance + kProgressTolerance);
      candidate.score = normalized_lateral * normalized_lateral +
          normalized_heading * normalized_heading +
          0.01 * normalized_topology * normalized_topology;

      if (!have_lateral || candidate.lateral_distance < best_lateral.lateral_distance)
      {
        best_lateral = candidate;
        have_lateral = true;
      }
      if (candidate.lateral_distance <= config_.max_lateral_distance)
      {
        if (!have_heading || candidate.heading_error < best_heading.heading_error)
        {
          best_heading = candidate;
          have_heading = true;
        }
        if (candidate.heading_error <= config_.max_heading_error)
        {
          if (!have_height || candidate.height_error < best_height.height_error)
          {
            best_height = candidate;
            have_height = true;
          }
          if (candidate.height_error <= config_.max_height_error &&
              (!have_valid || candidate.score < best_valid.score))
          {
            best_valid = candidate;
            have_valid = true;
          }
        }
      }
    }

    if (!closed && segment + 1 >= vertices.size() - 1)
      break;
    if (closed && segment + 1 >= vertices.size())
    {
      segment = 0;
      lap_start += route_length;
      segment_start_s = lap_start;
    }
    else
    {
      ++segment;
      segment_start_s = lap_start + vertex_s[segment];
    }
    if (segment_start_s > search_end + kProgressTolerance)
      break;
  }

  if (!have_valid)
  {
    if (!have_lateral)
    {
      return rejection(
          RouteProjectionStatus::kNoCandidate,
          progress_, movement_credit_, checked_segments);
    }
    if (best_lateral.lateral_distance > config_.max_lateral_distance)
    {
      return rejection(
          RouteProjectionStatus::kLateralGate,
          progress_, movement_credit_, checked_segments, &best_lateral);
    }
    if (!have_heading || best_heading.heading_error > config_.max_heading_error)
    {
      return rejection(
          RouteProjectionStatus::kHeadingGate,
          progress_, movement_credit_, checked_segments,
          have_heading ? &best_heading : &best_lateral);
    }
    return rejection(
        RouteProjectionStatus::kHeightGate,
        progress_, movement_credit_, checked_segments,
        have_height ? &best_height : &best_heading);
  }

  const double candidate_advance =
      std::max(0.0, best_valid.route_s - progress_);
  if (candidate_advance > config_.max_projection_advance + kProgressTolerance ||
      candidate_advance > movement_credit_ + kProgressTolerance)
  {
    return rejection(
        RouteProjectionStatus::kMovementBudget,
        progress_, movement_credit_, checked_segments, &best_valid);
  }

  if (closed && candidate_advance > kProgressTolerance)
  {
    const double current_lap = std::floor(progress_ / route_length);
    const double candidate_lap = std::floor(best_valid.route_s / route_length);
    if (candidate_lap > current_lap)
    {
      const double current_normalized = route_->normalizeRouteS(progress_);
      const double candidate_normalized =
          route_->normalizeRouteS(best_valid.route_s);
      const Eigen::Vector2d tangent_xy =
          route_->tangentAt(progress_).head<2>().normalized();
      const bool forward_motion = positive_motion_evidence &&
          odom_delta.head<2>().dot(tangent_xy) > 0.0;
      if (current_normalized <
              route_length - config_.loop_seam_gate_distance ||
          candidate_normalized > config_.loop_seam_gate_distance ||
          !forward_motion || candidate_lap > current_lap + 1.0)
      {
        return rejection(
            RouteProjectionStatus::kSeamGate,
            progress_, movement_credit_, checked_segments, &best_valid);
      }
    }
  }

  const double previous_progress = progress_;
  progress_ = std::min(maximum_progress, progress_ + candidate_advance);
  movement_credit_ = std::max(0.0, movement_credit_ - candidate_advance);
  completed_ = std::isfinite(maximum_progress) &&
      progress_ >= maximum_progress - kProgressTolerance;

  RouteProjectionResult output;
  output.status = completed_ ? RouteProjectionStatus::kCompleted :
                               RouteProjectionStatus::kAccepted;
  output.previous_route_s = previous_progress;
  output.candidate_route_s = best_valid.route_s;
  output.accepted_route_s = progress_;
  output.lateral_distance = best_valid.lateral_distance;
  output.heading_error = best_valid.heading_error;
  output.height_error = best_valid.height_error;
  output.available_movement_credit = movement_credit_;
  output.checked_segments = checked_segments;
  return output;
}

}  // namespace scan_planner
