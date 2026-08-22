// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file local_route_progress_map.cpp @brief Immutable local_s to route_s mapping. */

#include <plan_manage/local_route_progress_map.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace scan_planner
{
namespace
{

constexpr double kArcTolerance = 1.0e-9;
constexpr std::size_t kMaximumConfiguredSamples = 1000000;

bool isValidSource(LocalRouteProgressSource source) noexcept
{
  switch (source)
  {
    case LocalRouteProgressSource::kDirect:
    case LocalRouteProgressSource::kMerge:
    case LocalRouteProgressSource::kAvoidance:
    case LocalRouteProgressSource::kTransition:
      return true;
  }
  return false;
}

LocalRouteProgressMapResult failure(
    LocalRouteProgressMapStatus status) noexcept
{
  LocalRouteProgressMapResult output;
  output.status = status;
  return output;
}

std::size_t segmentIndex(
    const std::vector<LocalRouteProgressSample> &samples,
    double local_s) noexcept
{
  const auto upper = std::upper_bound(
      samples.begin(),
      samples.end(),
      local_s,
      [](double value, const LocalRouteProgressSample &sample) {
        return value < sample.local_s;
      });
  if (upper == samples.begin())
    return 0;
  return std::min<std::size_t>(
      static_cast<std::size_t>(std::distance(samples.begin(), upper) - 1),
      samples.size() - 2);
}

}  // namespace

const char * localRouteProgressMapStatusName(
    LocalRouteProgressMapStatus status) noexcept
{
  switch (status)
  {
    case LocalRouteProgressMapStatus::kSuccess:
      return "success";
    case LocalRouteProgressMapStatus::kInvalidInput:
      return "invalid_input";
    case LocalRouteProgressMapStatus::kNonFinite:
      return "non_finite";
    case LocalRouteProgressMapStatus::kNonMonotonic:
      return "non_monotonic";
    case LocalRouteProgressMapStatus::kInconsistentArcLength:
      return "inconsistent_arc_length";
    case LocalRouteProgressMapStatus::kZeroLength:
      return "zero_length";
    case LocalRouteProgressMapStatus::kResourceLimit:
      return "resource_limit";
  }
  return "unknown";
}

LocalRouteProgressMap::LocalRouteProgressMap(
    std::vector<LocalRouteProgressSample> samples)
: samples_(std::move(samples))
{
}

LocalRouteProgressMapResult LocalRouteProgressMap::create(
    std::vector<LocalRouteProgressSample> samples,
    std::size_t max_samples)
{
  if (max_samples < 2 || max_samples > kMaximumConfiguredSamples)
    return failure(LocalRouteProgressMapStatus::kInvalidInput);
  if (samples.size() > max_samples)
    return failure(LocalRouteProgressMapStatus::kResourceLimit);
  if (samples.size() < 2)
    return failure(LocalRouteProgressMapStatus::kInvalidInput);

  for (std::size_t index = 0; index < samples.size(); ++index)
  {
    const LocalRouteProgressSample &sample = samples[index];
    if (!sample.position.allFinite() ||
        !std::isfinite(sample.local_s) ||
        !std::isfinite(sample.route_s))
    {
      return failure(LocalRouteProgressMapStatus::kNonFinite);
    }
    if (sample.local_s < 0.0 || sample.route_s < 0.0)
      return failure(LocalRouteProgressMapStatus::kInvalidInput);
    if (!isValidSource(sample.source))
      return failure(LocalRouteProgressMapStatus::kInvalidInput);
    if (index == 0)
    {
      if (std::abs(sample.local_s) > kArcTolerance)
        return failure(LocalRouteProgressMapStatus::kInvalidInput);
      continue;
    }

    const LocalRouteProgressSample &previous = samples[index - 1];
    const double local_delta = sample.local_s - previous.local_s;
    const double route_delta = sample.route_s - previous.route_s;
    if (local_delta <= kArcTolerance || route_delta < 0.0)
      return failure(LocalRouteProgressMapStatus::kNonMonotonic);

    const double planar_distance =
        (sample.position.head<2>() - previous.position.head<2>()).norm();
    if (!std::isfinite(planar_distance) || planar_distance <= kArcTolerance)
      return failure(LocalRouteProgressMapStatus::kZeroLength);
    const double allowed_arc_error =
        std::max(1.0e-6, 1.0e-3 * planar_distance);
    if (std::abs(local_delta - planar_distance) > allowed_arc_error)
    {
      return failure(
          LocalRouteProgressMapStatus::kInconsistentArcLength);
    }
  }

  samples.front().local_s = 0.0;
  LocalRouteProgressMapResult output;
  output.status = LocalRouteProgressMapStatus::kSuccess;
  output.map = std::shared_ptr<const LocalRouteProgressMap>(
      new LocalRouteProgressMap(std::move(samples)));
  return output;
}

LocalRouteProgressMapResult LocalRouteProgressMap::fromAnchoredPolyline(
    const std::vector<Eigen::Vector3d> &positions,
    double route_s_entry,
    double route_s_exit,
    LocalRouteProgressSource source,
    std::size_t max_samples)
{
  if (max_samples < 2 || max_samples > kMaximumConfiguredSamples ||
      positions.size() < 2 ||
      !std::isfinite(route_s_entry) || !std::isfinite(route_s_exit) ||
      route_s_entry < 0.0 || route_s_exit < route_s_entry ||
      !isValidSource(source))
  {
    return failure(LocalRouteProgressMapStatus::kInvalidInput);
  }
  if (positions.size() > max_samples)
    return failure(LocalRouteProgressMapStatus::kResourceLimit);

  std::vector<LocalRouteProgressSample> samples;
  samples.reserve(positions.size());
  for (const Eigen::Vector3d &position : positions)
  {
    if (!position.allFinite())
      return failure(LocalRouteProgressMapStatus::kNonFinite);
    if (!samples.empty())
    {
      const double distance =
          (position.head<2>() - samples.back().position.head<2>()).norm();
      if (!std::isfinite(distance))
        return failure(LocalRouteProgressMapStatus::kNonFinite);
      if (distance <= kArcTolerance)
        continue;
      samples.push_back(
          {position, samples.back().local_s + distance, 0.0, source});
    }
    else
    {
      samples.push_back({position, 0.0, 0.0, source});
    }
  }
  if (samples.size() < 2 || samples.back().local_s <= kArcTolerance)
    return failure(LocalRouteProgressMapStatus::kZeroLength);

  const double total_local_s = samples.back().local_s;
  const double route_span = route_s_exit - route_s_entry;
  for (LocalRouteProgressSample &sample : samples)
  {
    const double ratio = std::clamp(sample.local_s / total_local_s, 0.0, 1.0);
    sample.route_s = route_s_entry + ratio * route_span;
  }
  samples.front().route_s = route_s_entry;
  samples.back().route_s = route_s_exit;
  return create(std::move(samples), max_samples);
}

const std::vector<LocalRouteProgressSample> &
LocalRouteProgressMap::samples() const noexcept
{
  return samples_;
}

double LocalRouteProgressMap::totalLocalLength() const noexcept
{
  return samples_.back().local_s;
}

double LocalRouteProgressMap::routeEntry() const noexcept
{
  return samples_.front().route_s;
}

double LocalRouteProgressMap::routeExit() const noexcept
{
  return samples_.back().route_s;
}

LocalRouteProgressLookup LocalRouteProgressMap::routeSAt(
    double local_s) const noexcept
{
  LocalRouteProgressLookup output;
  if (!std::isfinite(local_s))
    return output;
  if (local_s < -kArcTolerance ||
      local_s > totalLocalLength() + kArcTolerance)
  {
    output.status = LocalRouteProgressLookupStatus::kOutOfRange;
    return output;
  }

  const double clamped = std::clamp(local_s, 0.0, totalLocalLength());
  const std::size_t segment = segmentIndex(samples_, clamped);
  const LocalRouteProgressSample &lower = samples_[segment];
  const LocalRouteProgressSample &upper = samples_[segment + 1];
  const double local_span = upper.local_s - lower.local_s;
  const double ratio = std::clamp(
      (clamped - lower.local_s) / local_span, 0.0, 1.0);

  output.status = LocalRouteProgressLookupStatus::kSuccess;
  output.local_s = clamped;
  output.route_s = lower.route_s + ratio * (upper.route_s - lower.route_s);
  output.segment_index = segment;
  output.source = ratio >= 1.0 - kArcTolerance ? upper.source : lower.source;
  return output;
}

}  // namespace scan_planner
