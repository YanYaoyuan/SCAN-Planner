// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file merge_reference_builder.cpp @brief Bounded route merge seeds. */

#include <plan_manage/merge_reference_builder.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace scan_planner
{
namespace
{

constexpr double kArcTolerance = 1.0e-9;
constexpr double kPlanarTolerance = 1.0e-9;
constexpr std::size_t kMaximumConfiguredSamples = 1000000;
constexpr std::size_t kMaximumCornerExtensions = 1000;

bool expired(const MergeReferenceBuilder::DeadlineQuery &query) noexcept
{
  return !query || query();
}

bool validConfig(const MergeReferenceConfig &config) noexcept
{
  return std::isfinite(config.minimum_forward_distance) &&
      std::isfinite(config.lateral_distance_scale) &&
      std::isfinite(config.speed_lookahead_time) &&
      std::isfinite(config.curvature_safety_factor) &&
      std::isfinite(config.maximum_forward_distance) &&
      std::isfinite(config.maximum_initial_offset) &&
      std::isfinite(config.maximum_seed_curvature) &&
      std::isfinite(config.sample_spacing) &&
      std::isfinite(config.zero_offset_tolerance) &&
      std::isfinite(config.corner_clearance) &&
      std::isfinite(config.corner_angle_threshold) &&
      config.minimum_forward_distance > 0.0 &&
      config.lateral_distance_scale >= 0.0 &&
      config.speed_lookahead_time >= 0.0 &&
      config.curvature_safety_factor >= 0.0 &&
      config.maximum_forward_distance >= config.minimum_forward_distance &&
      config.maximum_initial_offset > 0.0 &&
      config.maximum_seed_curvature > 0.0 &&
      config.sample_spacing > 0.0 &&
      config.zero_offset_tolerance >= 0.0 &&
      config.zero_offset_tolerance <= config.maximum_initial_offset &&
      config.corner_clearance > 0.0 &&
      config.corner_angle_threshold > 0.0 &&
      config.corner_angle_threshold <= std::acos(-1.0) &&
      config.max_corner_extensions > 0 &&
      config.max_corner_extensions <= kMaximumCornerExtensions &&
      config.max_samples >= 2 &&
      config.max_samples <= kMaximumConfiguredSamples;
}

std::vector<double> sharpVertexCoordinates(
    const ReferenceRoute &route,
    double angle_threshold,
    const MergeReferenceBuilder::DeadlineQuery &deadline_exceeded,
    bool *deadline_hit)
{
  std::vector<double> sharp_vertices;
  const auto &points = route.points();
  const auto &arc_lengths = route.cumulativeLengths();
  sharp_vertices.reserve(points.size());

  const std::size_t begin = route.closed() ? 0 : 1;
  const std::size_t end = route.closed() ? points.size() : points.size() - 1;
  for (std::size_t index = begin; index < end; ++index)
  {
    if (expired(deadline_exceeded))
    {
      *deadline_hit = true;
      return {};
    }
    const std::size_t previous = index == 0 ? points.size() - 1 : index - 1;
    const std::size_t next = index + 1 == points.size() ? 0 : index + 1;
    const Eigen::Vector2d incoming =
        (points[index] - points[previous]).head<2>();
    const Eigen::Vector2d outgoing =
        (points[next] - points[index]).head<2>();
    const double incoming_norm = incoming.norm();
    const double outgoing_norm = outgoing.norm();
    if (!std::isfinite(incoming_norm) || !std::isfinite(outgoing_norm) ||
        incoming_norm <= kPlanarTolerance || outgoing_norm <= kPlanarTolerance)
    {
      sharp_vertices.push_back(arc_lengths[index]);
      continue;
    }
    const double cosine = std::clamp(
        incoming.dot(outgoing) / (incoming_norm * outgoing_norm), -1.0, 1.0);
    if (std::acos(cosine) > angle_threshold)
      sharp_vertices.push_back(arc_lengths[index]);
  }
  return sharp_vertices;
}

bool nearestSharpCorner(
    const ReferenceRoute &route,
    const std::vector<double> &sharp_vertices,
    double route_s,
    double clearance,
    double *corner_route_s) noexcept
{
  double best_distance = std::numeric_limits<double>::infinity();
  double best_coordinate = 0.0;
  for (double vertex_s : sharp_vertices)
  {
    double candidate = vertex_s;
    if (route.closed())
    {
      const double lap = std::round(
          (route_s - vertex_s) / route.totalLength());
      candidate += lap * route.totalLength();
    }
    const double distance = std::abs(candidate - route_s);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_coordinate = candidate;
    }
  }
  if (best_distance > clearance + kArcTolerance)
    return false;
  *corner_route_s = best_coordinate;
  return true;
}

bool isSharpCoordinate(
    const ReferenceRoute &route,
    const std::vector<double> &sharp_vertices,
    double route_s) noexcept
{
  double unused = 0.0;
  return nearestSharpCorner(
      route, sharp_vertices, route_s, kArcTolerance, &unused);
}

bool samplingFits(
    const ReferenceRoute &route,
    double start_s,
    double end_s,
    const MergeReferenceConfig &config,
    const MergeReferenceBuilder::DeadlineQuery &deadline_exceeded,
    bool *deadline_hit) noexcept
{
  const double span = end_s - start_s;
  const double regular_count = std::ceil(span / config.sample_spacing) + 1.0;
  if (!std::isfinite(regular_count) ||
      regular_count > static_cast<double>(config.max_samples))
  {
    return false;
  }

  std::size_t corner_count = 0;
  for (double vertex_s : route.cumulativeLengths())
  {
    if (expired(deadline_exceeded))
    {
      *deadline_hit = true;
      return false;
    }
    if (!route.closed())
    {
      if (vertex_s >= start_s - kArcTolerance &&
          vertex_s <= end_s + kArcTolerance)
      {
        ++corner_count;
      }
    }
    else
    {
      const double first_lap = std::ceil(
          (start_s - vertex_s - kArcTolerance) / route.totalLength());
      const double last_lap = std::floor(
          (end_s - vertex_s + kArcTolerance) / route.totalLength());
      if (!std::isfinite(first_lap) || !std::isfinite(last_lap))
        return false;
      if (last_lap >= first_lap)
      {
        const double occurrences = last_lap - first_lap + 1.0;
        if (occurrences > static_cast<double>(config.max_samples -
            std::min(corner_count, config.max_samples)))
        {
          return false;
        }
        corner_count += static_cast<std::size_t>(occurrences);
      }
    }
    if (corner_count >= config.max_samples)
      return false;
  }
  return regular_count + static_cast<double>(corner_count) + 2.0 <=
      static_cast<double>(config.max_samples);
}

double quinticFade(double unit_progress) noexcept
{
  const double u = std::clamp(unit_progress, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return 1.0 - (10.0 * u3 - 15.0 * u4 + 6.0 * u5);
}

double quinticFadeDerivative(double unit_progress) noexcept
{
  const double u = std::clamp(unit_progress, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  return -(30.0 * u2 - 60.0 * u3 + 30.0 * u4);
}

double maximumPlanarCurvature(
    const std::vector<MergeReferenceSample> &samples,
    double merge_end_route_s,
    const MergeReferenceBuilder::DeadlineQuery &deadline_exceeded,
    bool *deadline_hit) noexcept
{
  double maximum = 0.0;
  for (std::size_t index = 1; index + 1 < samples.size(); ++index)
  {
    if (expired(deadline_exceeded))
    {
      *deadline_hit = true;
      return 0.0;
    }
    if (samples[index].route_s > merge_end_route_s + kArcTolerance)
      break;
    const Eigen::Vector2d first = samples[index - 1].position.head<2>();
    const Eigen::Vector2d middle = samples[index].position.head<2>();
    const Eigen::Vector2d last = samples[index + 1].position.head<2>();
    const double side_a = (middle - first).norm();
    const double side_b = (last - middle).norm();
    const double side_c = (last - first).norm();
    if (side_a <= kPlanarTolerance || side_b <= kPlanarTolerance ||
        side_c <= kPlanarTolerance)
    {
      continue;
    }
    const double twice_area = std::abs(
        (middle.x() - first.x()) * (last.y() - first.y()) -
        (middle.y() - first.y()) * (last.x() - first.x()));
    const double curvature =
        2.0 * twice_area / (side_a * side_b * side_c);
    if (!std::isfinite(curvature))
      return std::numeric_limits<double>::infinity();
    maximum = std::max(maximum, curvature);
  }
  return maximum;
}

MergeReferenceResult fail(MergeReferenceStatus status) noexcept
{
  MergeReferenceResult result;
  result.status = status;
  return result;
}

MergeReferenceStatus convertRangeStatus(
    ReferenceRouteRangeStatus status) noexcept
{
  switch (status)
  {
    case ReferenceRouteRangeStatus::kSuccess:
      return MergeReferenceStatus::kSuccess;
    case ReferenceRouteRangeStatus::kOutOfRange:
      return MergeReferenceStatus::kOutOfRange;
    case ReferenceRouteRangeStatus::kResourceLimit:
      return MergeReferenceStatus::kResourceLimit;
    case ReferenceRouteRangeStatus::kDeadlineExceeded:
      return MergeReferenceStatus::kDeadlineExceeded;
    case ReferenceRouteRangeStatus::kInvalidInput:
      return MergeReferenceStatus::kRouteSamplingFailed;
  }
  return MergeReferenceStatus::kRouteSamplingFailed;
}

}  // namespace

const char * mergeReferenceStatusName(MergeReferenceStatus status) noexcept
{
  switch (status)
  {
    case MergeReferenceStatus::kSuccess:
      return "success";
    case MergeReferenceStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case MergeReferenceStatus::kInvalidInput:
      return "invalid_input";
    case MergeReferenceStatus::kOutOfRange:
      return "out_of_range";
    case MergeReferenceStatus::kOffsetTooLarge:
      return "offset_too_large";
    case MergeReferenceStatus::kStartNearUnsmoothedCorner:
      return "start_near_unsmoothed_corner";
    case MergeReferenceStatus::kInsufficientLookahead:
      return "insufficient_lookahead";
    case MergeReferenceStatus::kCannotAvoidCorner:
      return "cannot_avoid_corner";
    case MergeReferenceStatus::kCurvatureLimit:
      return "curvature_limit";
    case MergeReferenceStatus::kResourceLimit:
      return "resource_limit";
    case MergeReferenceStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case MergeReferenceStatus::kRouteSamplingFailed:
      return "route_sampling_failed";
  }
  return "unknown";
}

MergeReferenceResult MergeReferenceBuilder::build(
    const ReferenceRoute &route,
    const Eigen::Vector3d &current_position,
    double progress_route_s,
    double local_target_route_s,
    double current_speed,
    const MergeReferenceConfig &config,
    const DeadlineQuery &deadline_exceeded)
{
  if (!validConfig(config))
    return fail(MergeReferenceStatus::kInvalidConfiguration);
  if (!deadline_exceeded)
    return fail(MergeReferenceStatus::kInvalidInput);
  if (expired(deadline_exceeded))
    return fail(MergeReferenceStatus::kDeadlineExceeded);
  if (!current_position.allFinite() ||
      !std::isfinite(progress_route_s) ||
      !std::isfinite(local_target_route_s) ||
      !std::isfinite(current_speed) || current_speed < 0.0 ||
      local_target_route_s <= progress_route_s + kArcTolerance)
  {
    return fail(MergeReferenceStatus::kInvalidInput);
  }
  if (!route.closed() &&
      (progress_route_s < -kArcTolerance ||
       local_target_route_s > route.totalLength() + kArcTolerance))
  {
    return fail(MergeReferenceStatus::kOutOfRange);
  }

  const Eigen::Vector3d progress_position = route.pointAt(progress_route_s);
  const Eigen::Vector3d offset = current_position - progress_position;
  const double offset_norm = offset.norm();
  const double horizontal_offset = offset.head<2>().norm();
  if (!std::isfinite(offset_norm) || !std::isfinite(horizontal_offset))
    return fail(MergeReferenceStatus::kInvalidInput);
  if (offset_norm > config.maximum_initial_offset + kArcTolerance)
    return fail(MergeReferenceStatus::kOffsetTooLarge);

  bool deadline_hit = false;
  const std::vector<double> sharp_vertices = sharpVertexCoordinates(
      route, config.corner_angle_threshold, deadline_exceeded, &deadline_hit);
  if (deadline_hit)
    return fail(MergeReferenceStatus::kDeadlineExceeded);

  MergeReferenceResult result;
  result.merge_start_route_s = progress_route_s;
  result.merge_end_route_s = progress_route_s;
  result.no_merge_required = offset_norm <= config.zero_offset_tolerance;

  if (!result.no_merge_required)
  {
    double start_corner_s = 0.0;
    if (nearestSharpCorner(
        route, sharp_vertices, progress_route_s,
        config.corner_clearance, &start_corner_s))
    {
      return fail(MergeReferenceStatus::kStartNearUnsmoothedCorner);
    }

    const double curvature_distance = std::sqrt(
        config.curvature_safety_factor * horizontal_offset /
        config.maximum_seed_curvature);
    const double requested = std::max(
        {config.minimum_forward_distance,
         config.lateral_distance_scale * horizontal_offset,
         config.speed_lookahead_time * current_speed,
         curvature_distance});
    if (!std::isfinite(requested))
      return fail(MergeReferenceStatus::kInvalidInput);
    result.requested_forward_distance =
        std::min(requested, config.maximum_forward_distance);
    result.merge_end_route_s =
        progress_route_s + result.requested_forward_distance;

    if (result.merge_end_route_s >
        local_target_route_s + kArcTolerance)
    {
      result.status = MergeReferenceStatus::kInsufficientLookahead;
      result.clipped_by_local_target = true;
      return result;
    }

    bool cleared_corner = false;
    for (std::size_t attempt = 0;
         attempt < config.max_corner_extensions; ++attempt)
    {
      if (expired(deadline_exceeded))
        return fail(MergeReferenceStatus::kDeadlineExceeded);
      double corner_s = 0.0;
      if (!nearestSharpCorner(
          route, sharp_vertices, result.merge_end_route_s,
          config.corner_clearance, &corner_s))
      {
        cleared_corner = true;
        break;
      }
      const double extended_end = corner_s + config.corner_clearance;
      if (!std::isfinite(extended_end))
        return fail(MergeReferenceStatus::kCannotAvoidCorner);
      if (extended_end <= result.merge_end_route_s + kArcTolerance)
      {
        // The endpoint is already exactly on the configured safe boundary.
        cleared_corner = true;
        break;
      }
      if (extended_end - progress_route_s >
          config.maximum_forward_distance + kArcTolerance)
      {
        return fail(MergeReferenceStatus::kCannotAvoidCorner);
      }
      if (extended_end > local_target_route_s + kArcTolerance)
      {
        result.status = MergeReferenceStatus::kInsufficientLookahead;
        result.clipped_by_local_target = true;
        return result;
      }
      result.merge_end_route_s = extended_end;
    }
    if (!cleared_corner)
      return fail(MergeReferenceStatus::kCannotAvoidCorner);
    result.actual_forward_distance =
        result.merge_end_route_s - progress_route_s;
  }

  if (!samplingFits(
      route, progress_route_s, local_target_route_s, config,
      deadline_exceeded, &deadline_hit))
  {
    return fail(deadline_hit ? MergeReferenceStatus::kDeadlineExceeded :
                MergeReferenceStatus::kResourceLimit);
  }
  std::vector<ReferenceRouteSample> route_samples;
  const auto append_range =
      [&route_samples, &config](const ReferenceRouteRangeResult &range) {
        if (!range.success())
          return convertRangeStatus(range.status);
        for (const ReferenceRouteSample &sample : range.samples)
        {
          if (!route_samples.empty() &&
              std::abs(route_samples.back().route_s - sample.route_s) <=
                  kArcTolerance)
          {
            const bool original_vertex =
                route_samples.back().is_original_vertex ||
                sample.is_original_vertex;
            route_samples.back() = sample;
            route_samples.back().is_original_vertex = original_vertex;
            continue;
          }
          if (route_samples.size() >= config.max_samples)
            return MergeReferenceStatus::kResourceLimit;
          route_samples.push_back(sample);
        }
        return MergeReferenceStatus::kSuccess;
      };

  MergeReferenceStatus range_status = MergeReferenceStatus::kSuccess;
  if (result.no_merge_required)
  {
    range_status = append_range(route.sampleRange(
        progress_route_s, local_target_route_s, config.sample_spacing,
        deadline_exceeded));
  }
  else
  {
    range_status = append_range(route.sampleRange(
        progress_route_s, result.merge_end_route_s,
        config.sample_spacing, deadline_exceeded));
    if (range_status == MergeReferenceStatus::kSuccess)
    {
      range_status = append_range(route.sampleRange(
          result.merge_end_route_s, local_target_route_s,
          config.sample_spacing, deadline_exceeded));
    }
  }
  if (range_status != MergeReferenceStatus::kSuccess)
    return fail(range_status);
  if (route_samples.size() < 2 || route_samples.size() > config.max_samples)
    return fail(MergeReferenceStatus::kResourceLimit);

  result.samples.reserve(route_samples.size());
  for (const ReferenceRouteSample &route_sample : route_samples)
  {
    if (expired(deadline_exceeded))
      return fail(MergeReferenceStatus::kDeadlineExceeded);
    MergeReferenceSample sample;
    sample.position = route_sample.position;
    sample.tangent = route_sample.tangent;
    sample.route_s = route_sample.route_s;
    sample.is_original_route_vertex = route_sample.is_original_vertex;
    sample.source = LocalRouteProgressSource::kDirect;

    if (!result.no_merge_required &&
        route_sample.route_s < result.merge_end_route_s - kArcTolerance)
    {
      const double unit_progress =
          (route_sample.route_s - progress_route_s) /
          result.actual_forward_distance;
      sample.position += quinticFade(unit_progress) * offset;
      const Eigen::Vector3d derivative =
          route_sample.tangent +
          (quinticFadeDerivative(unit_progress) /
           result.actual_forward_distance) * offset;
      const double derivative_norm = derivative.norm();
      if (!std::isfinite(derivative_norm) ||
          derivative_norm <= kPlanarTolerance)
      {
        return fail(MergeReferenceStatus::kRouteSamplingFailed);
      }
      sample.tangent = derivative / derivative_norm;
      sample.source = LocalRouteProgressSource::kMerge;
    }
    if (result.samples.empty() && !result.no_merge_required)
      sample.position = current_position;
    if (!sample.position.allFinite() || !sample.tangent.allFinite())
      return fail(MergeReferenceStatus::kRouteSamplingFailed);

    if (route_sample.is_original_vertex &&
        route_sample.route_s > progress_route_s + kArcTolerance &&
        route_sample.route_s < result.merge_end_route_s - kArcTolerance &&
        isSharpCoordinate(route, sharp_vertices, route_sample.route_s))
    {
      result.crosses_unsmoothed_corner = true;
    }
    result.samples.push_back(std::move(sample));
  }

  if (!result.no_merge_required)
  {
    result.estimated_max_curvature = maximumPlanarCurvature(
        result.samples, result.merge_end_route_s,
        deadline_exceeded, &deadline_hit);
    if (deadline_hit)
      return fail(MergeReferenceStatus::kDeadlineExceeded);
    if (!std::isfinite(result.estimated_max_curvature) ||
        result.estimated_max_curvature >
            config.maximum_seed_curvature + kArcTolerance)
    {
      result.status = MergeReferenceStatus::kCurvatureLimit;
      result.samples.clear();
      return result;
    }
  }

  result.status = MergeReferenceStatus::kSuccess;
  return result;
}

}  // namespace scan_planner
