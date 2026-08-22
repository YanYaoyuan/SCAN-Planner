// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file reference_route.cpp @brief Immutable arc-length reference-route geometry. */

#include <plan_manage/reference_route.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace scan_planner
{
namespace
{

constexpr double kArcTolerance = 1.0e-10;

bool withinInclusive(double value, double lower, double upper) noexcept
{
  return value >= lower - kArcTolerance && value <= upper + kArcTolerance;
}

}  // namespace

const char * referenceRouteRangeStatusName(
    ReferenceRouteRangeStatus status) noexcept
{
  switch (status)
  {
    case ReferenceRouteRangeStatus::kSuccess:
      return "success";
    case ReferenceRouteRangeStatus::kInvalidInput:
      return "invalid_input";
    case ReferenceRouteRangeStatus::kOutOfRange:
      return "out_of_range";
    case ReferenceRouteRangeStatus::kResourceLimit:
      return "resource_limit";
  }
  return "unknown";
}

ReferenceRoute::ReferenceRoute(
    const std::vector<Eigen::Vector3d> &points,
    bool closed)
: ReferenceRoute(points, closed, Config{})
{
}

ReferenceRoute::ReferenceRoute(
    const std::vector<Eigen::Vector3d> &points,
    bool closed,
    const Config &config)
: closed_(closed), config_(config)
{
  if (!std::isfinite(config_.duplicate_tolerance) ||
      config_.duplicate_tolerance < 0.0 ||
      config_.max_sample_count < 2 ||
      config_.max_sample_count > 1000000)
  {
    throw std::invalid_argument("invalid reference-route configuration");
  }
  if (points.size() < 2)
    throw std::invalid_argument("reference route requires at least two points");
  if (points.size() > config_.max_sample_count)
    throw std::invalid_argument("reference route exceeds the configured point limit");

  points_.reserve(points.size());
  for (const Eigen::Vector3d &point : points)
  {
    if (!point.allFinite())
      throw std::invalid_argument("reference route contains a non-finite point");
    if (points_.empty() ||
        (point - points_.back()).norm() > config_.duplicate_tolerance)
    {
      points_.push_back(point);
    }
  }

  if (closed_ && points_.size() >= 2 &&
      (points_.front() - points_.back()).norm() <=
          config_.duplicate_tolerance)
  {
    points_.pop_back();
  }

  const std::size_t minimum_point_count = closed_ ? 3 : 2;
  if (points_.size() < minimum_point_count)
  {
    throw std::invalid_argument(
        closed_ ? "closed reference route requires three distinct vertices" :
                  "reference route has fewer than two distinct points");
  }

  cumulative_lengths_.assign(points_.size(), 0.0);
  for (std::size_t index = 1; index < points_.size(); ++index)
  {
    const double segment_length =
        (points_[index] - points_[index - 1]).norm();
    if (!std::isfinite(segment_length) || segment_length <= 0.0)
      throw std::invalid_argument("reference route contains an invalid segment");
    cumulative_lengths_[index] =
        cumulative_lengths_[index - 1] + segment_length;
  }

  total_length_ = cumulative_lengths_.back();
  if (closed_)
  {
    const double closing_length =
        (points_.front() - points_.back()).norm();
    if (!std::isfinite(closing_length) ||
        closing_length <= config_.duplicate_tolerance)
    {
      throw std::invalid_argument("closed reference route has an invalid seam");
    }
    total_length_ += closing_length;
  }
  if (!std::isfinite(total_length_) || total_length_ <= 0.0)
    throw std::invalid_argument("reference route has zero or invalid length");
}

bool ReferenceRoute::closed() const noexcept
{
  return closed_;
}

double ReferenceRoute::totalLength() const noexcept
{
  return total_length_;
}

const std::vector<Eigen::Vector3d> & ReferenceRoute::points() const noexcept
{
  return points_;
}

const std::vector<double> & ReferenceRoute::cumulativeLengths() const noexcept
{
  return cumulative_lengths_;
}

double ReferenceRoute::normalizeRouteS(double route_s) const
{
  if (!std::isfinite(route_s))
    throw std::invalid_argument("route_s must be finite");
  if (!closed_)
  {
    if (route_s < -kArcTolerance ||
        route_s > total_length_ + kArcTolerance)
    {
      throw std::out_of_range("open-route route_s is outside route bounds");
    }
    return std::clamp(route_s, 0.0, total_length_);
  }

  double normalized = std::fmod(route_s, total_length_);
  if (normalized < 0.0)
    normalized += total_length_;
  return normalized;
}

std::size_t ReferenceRoute::segmentIndex(double route_s) const
{
  const double normalized = normalizeRouteS(route_s);
  if (closed_ && normalized >= cumulative_lengths_.back())
    return points_.size() - 1;
  if (!closed_ && normalized >= total_length_ - kArcTolerance)
    return points_.size() - 2;

  const auto upper = std::upper_bound(
      cumulative_lengths_.begin(),
      cumulative_lengths_.end(),
      normalized + kArcTolerance);
  if (upper == cumulative_lengths_.begin())
    return 0;
  return std::min<std::size_t>(
      static_cast<std::size_t>(
          std::distance(cumulative_lengths_.begin(), upper) - 1),
      points_.size() - 1 - static_cast<std::size_t>(!closed_));
}

Eigen::Vector3d ReferenceRoute::pointAt(double route_s) const
{
  const double normalized = normalizeRouteS(route_s);
  if (!closed_ && normalized >= total_length_ - kArcTolerance)
    return points_.back();

  const std::size_t segment = segmentIndex(normalized);
  const std::size_t next = segment + 1 < points_.size() ? segment + 1 : 0;
  const double segment_start = cumulative_lengths_[segment];
  const double segment_end = segment + 1 < cumulative_lengths_.size()
      ? cumulative_lengths_[segment + 1]
      : total_length_;
  const double segment_length = segment_end - segment_start;
  const double ratio = segment_length > 0.0
      ? std::clamp((normalized - segment_start) / segment_length, 0.0, 1.0)
      : 0.0;
  return points_[segment] + ratio * (points_[next] - points_[segment]);
}

Eigen::Vector3d ReferenceRoute::tangentAt(double route_s) const
{
  const std::size_t segment = segmentIndex(route_s);
  const std::size_t next = segment + 1 < points_.size() ? segment + 1 : 0;
  const Eigen::Vector3d delta = points_[next] - points_[segment];
  const double norm = delta.norm();
  if (!std::isfinite(norm) || norm <= 0.0)
    throw std::logic_error("reference route contains an invalid tangent");
  return delta / norm;
}

ReferenceRouteRangeResult ReferenceRoute::sampleRange(
    double start_s,
    double end_s,
    double spacing) const
{
  ReferenceRouteRangeResult output;
  if (!std::isfinite(start_s) || !std::isfinite(end_s) ||
      !std::isfinite(spacing) || spacing <= 0.0 ||
      end_s < start_s - kArcTolerance)
  {
    output.status = ReferenceRouteRangeStatus::kInvalidInput;
    return output;
  }
  if (!closed_ &&
      (start_s < -kArcTolerance || end_s > total_length_ + kArcTolerance))
  {
    output.status = ReferenceRouteRangeStatus::kOutOfRange;
    return output;
  }

  if (!closed_)
  {
    start_s = std::clamp(start_s, 0.0, total_length_);
    end_s = std::clamp(end_s, 0.0, total_length_);
  }
  const double span = std::max(0.0, end_s - start_s);
  const double regular_segment_count = std::ceil(span / spacing);
  if (!std::isfinite(regular_segment_count) ||
      regular_segment_count + 1.0 >
          static_cast<double>(config_.max_sample_count))
  {
    output.status = ReferenceRouteRangeStatus::kResourceLimit;
    return output;
  }

  std::vector<std::pair<double, bool>> coordinates;
  coordinates.reserve(
      static_cast<std::size_t>(regular_segment_count + 1.0) +
      config_.max_sample_count);
  coordinates.emplace_back(start_s, false);
  const std::size_t regular_segments =
      static_cast<std::size_t>(regular_segment_count);
  for (std::size_t index = 1; index < regular_segments; ++index)
  {
    const double route_s =
        start_s + spacing * static_cast<double>(index);
    if (!std::isfinite(route_s) || route_s <= start_s)
    {
      output.status = ReferenceRouteRangeStatus::kInvalidInput;
      return output;
    }
    // ceil(span / spacing) can be one too large after floating-point
    // rounding. The exact end is appended below, so stop instead of rejecting
    // an otherwise valid range or inserting a near-duplicate endpoint.
    if (route_s >= end_s - kArcTolerance)
      break;
    coordinates.emplace_back(route_s, false);
  }
  if (end_s > start_s + kArcTolerance)
    coordinates.emplace_back(end_s, false);

  if (closed_)
  {
    std::size_t corner_count = 0;
    for (double vertex_s : cumulative_lengths_)
    {
      const double first_lap = std::ceil(
          (start_s - vertex_s - kArcTolerance) / total_length_);
      const double last_lap = std::floor(
          (end_s - vertex_s + kArcTolerance) / total_length_);
      if (!std::isfinite(first_lap) || !std::isfinite(last_lap) ||
          first_lap <= static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
          last_lap >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
      {
        output.status = ReferenceRouteRangeStatus::kResourceLimit;
        return output;
      }
      const std::int64_t first_lap_index =
          static_cast<std::int64_t>(first_lap);
      const std::int64_t last_lap_index =
          static_cast<std::int64_t>(last_lap);
      if (first_lap_index > last_lap_index)
        continue;
      for (std::int64_t lap = first_lap_index; ; ++lap)
      {
        if (++corner_count > config_.max_sample_count)
        {
          output.status = ReferenceRouteRangeStatus::kResourceLimit;
          return output;
        }
        const double unwrapped_s =
            static_cast<double>(lap) * total_length_ + vertex_s;
        if (!std::isfinite(unwrapped_s))
        {
          output.status = ReferenceRouteRangeStatus::kResourceLimit;
          return output;
        }
        coordinates.emplace_back(unwrapped_s, true);
        if (lap == last_lap_index)
          break;
      }
    }
  }
  else
  {
    for (double vertex_s : cumulative_lengths_)
    {
      if (withinInclusive(vertex_s, start_s, end_s))
        coordinates.emplace_back(vertex_s, true);
    }
  }

  std::sort(
      coordinates.begin(),
      coordinates.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });

  std::vector<std::pair<double, bool>> unique_coordinates;
  unique_coordinates.reserve(coordinates.size());
  for (const auto &coordinate : coordinates)
  {
    if (!unique_coordinates.empty() &&
        std::abs(coordinate.first - unique_coordinates.back().first) <=
            kArcTolerance)
    {
      unique_coordinates.back().second =
          unique_coordinates.back().second || coordinate.second;
    }
    else
    {
      unique_coordinates.push_back(coordinate);
    }
  }

  if (unique_coordinates.size() > config_.max_sample_count)
  {
    output.status = ReferenceRouteRangeStatus::kResourceLimit;
    return output;
  }
  output.samples.reserve(unique_coordinates.size());
  for (const auto &[route_s, is_original_vertex] : unique_coordinates)
  {
    output.samples.push_back(
        {pointAt(route_s), tangentAt(route_s), route_s, is_original_vertex});
  }
  output.status = ReferenceRouteRangeStatus::kSuccess;
  return output;
}

}  // namespace scan_planner
