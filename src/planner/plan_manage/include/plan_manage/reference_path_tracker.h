// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file reference_path_tracker.h @brief Arc-length progress tracking for reference routes. */

#ifndef PLAN_MANAGE__REFERENCE_PATH_TRACKER_H_
#define PLAN_MANAGE__REFERENCE_PATH_TRACKER_H_

#include <Eigen/Core>
#include <plan_manage/reference_route.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace scan_planner
{

/**
 * @brief Tracks monotonic robot progress on an immutable reference polyline.
 *
 * Progress is expressed as polyline arc length instead of global-polynomial
 * time. A movement-credit budget prevents repeated projection calls from
 * advancing a stationary robot through a closed or self-intersecting route.
 */
class ReferencePathTracker
{
public:
  /** @brief Runtime limits controlling projection and closed-loop departure. */
  struct Config
  {
    double max_projection_advance{2.0};  ///< Maximum forward search window in metres.
    double movement_credit_scale{1.5};   ///< Route metres credited per odometry metre.
    double movement_deadband{0.02};      ///< Odom displacement accumulated before crediting motion.
    double initial_credit{0.10};         ///< One-shot projection allowance at route start.
    double departure_distance{0.60};     ///< Start distance or credited arc progress required to arm completion.
  };

  /** @brief Constructs a tracker with conservative default limits. */
  ReferencePathTracker()
  {
    setConfig(Config{});
  }

  /**
   * @brief Constructs a tracker with explicit tracking limits.
   * @param config Valid positive tracking limits.
   */
  explicit ReferencePathTracker(const Config &config)
  {
    setConfig(config);
  }

  /**
   * @brief Replaces runtime tracking limits.
   * @param config Valid positive tracking limits.
   * @throws std::invalid_argument If any limit is invalid.
   */
  void setConfig(const Config &config)
  {
    if (!std::isfinite(config.max_projection_advance) ||
        config.max_projection_advance <= 0.0 ||
        !std::isfinite(config.movement_credit_scale) ||
        config.movement_credit_scale < 1.0 ||
        !std::isfinite(config.movement_deadband) ||
        config.movement_deadband < 0.0 ||
        !std::isfinite(config.initial_credit) ||
        config.initial_credit < 0.0 ||
        !std::isfinite(config.departure_distance) ||
        config.departure_distance <= 0.0)
    {
      throw std::invalid_argument("invalid reference-path tracker configuration");
    }
    config_ = config;
  }

  /**
   * @brief Starts tracking a new immutable route.
   * @param points Ordered route points, including mission start and finish.
   * @param closed Whether the route represents one closed lap.
   * @param odom_position Robot position when the mission is accepted.
   * @throws std::invalid_argument If the route contains fewer than two
   * distinct finite points.
   */
  void reset(
      const std::vector<Eigen::Vector3d> &points,
      bool closed,
      const Eigen::Vector3d &odom_position)
  {
    if (points.size() < 2 || !odom_position.allFinite())
      throw std::invalid_argument("reference route requires finite start data");

    // The legacy tracker intentionally preserves the supplied polyline as an
    // open geometric domain. Its closed-loop completion gate remains tracker
    // state until the FSM adopts ReferenceRoute's native seam semantics.
    route_.emplace(points, false);

    closed_ = closed;
    departed_start_ = !closed;
    progress_ = 0.0;
    movement_credit_ =
        std::min(config_.initial_credit, config_.max_projection_advance);
    last_odom_position_ = odom_position;
    have_last_odom_ = true;
  }

  /** @brief Clears the active route and all progress state. */
  void clear()
  {
    route_.reset();
    closed_ = false;
    departed_start_ = false;
    progress_ = 0.0;
    movement_credit_ = 0.0;
    have_last_odom_ = false;
  }

  /** @brief Returns whether a valid route is active. */
  bool active() const
  {
    return route_.has_value();
  }

  /** @brief Returns whether the active route is closed. */
  bool closed() const
  {
    return closed_;
  }

  /** @brief Returns whether the robot has moved far enough from the loop start. */
  bool departedStart() const
  {
    return departed_start_;
  }

  /** @brief Returns current monotonic route progress in metres. */
  double progress() const
  {
    return progress_;
  }

  /** @brief Returns total reference-route arc length in metres. */
  double totalLength() const
  {
    return active() ? route_->totalLength() : 0.0;
  }

  /** @brief Returns remaining reference-route arc length in metres. */
  double remainingLength() const
  {
    return std::max(0.0, totalLength() - progress_);
  }

  /**
   * @brief Updates monotonic progress using the latest robot position.
   * @param odom_position Latest global robot position.
   * @return Updated route progress in metres.
   *
   * The forward projection window is funded by actual odometry displacement.
   * Unused credit is bounded, and accepted progress consumes that credit.
   * Repeated calls at an unchanged pose therefore cannot walk through a loop.
   */
  double update(const Eigen::Vector3d &odom_position)
  {
    if (!active() || !odom_position.allFinite())
      return progress_;

    if (have_last_odom_)
    {
      const double displacement =
          (odom_position.head<2>() - last_odom_position_.head<2>()).norm();
      if (std::isfinite(displacement) &&
          displacement >= config_.movement_deadband)
      {
        movement_credit_ = std::min(
            config_.max_projection_advance,
            movement_credit_ + config_.movement_credit_scale * displacement);
        last_odom_position_ = odom_position;
      }
    }
    else
    {
      last_odom_position_ = odom_position;
      have_last_odom_ = true;
    }

    if ((odom_position.head<2>() - route_->points().front().head<2>()).norm() >=
        config_.departure_distance)
    {
      departed_start_ = true;
    }

    const double search_end = std::min(
        totalLength(),
        progress_ + std::min(config_.max_projection_advance, movement_credit_));
    if (search_end <= progress_ + 1.0e-9)
      return progress_;

    double best_progress = progress_;
    double best_distance_squared =
        (pointAt(progress_).head<2>() - odom_position.head<2>()).squaredNorm();

    const auto &points = route_->points();
    const auto &cumulative_lengths = route_->cumulativeLengths();
    for (std::size_t segment = segmentIndex(progress_);
         segment + 1 < points.size(); ++segment)
    {
      const double segment_start =
          std::max(progress_, cumulative_lengths[segment]);
      const double segment_end =
          std::min(search_end, cumulative_lengths[segment + 1]);
      if (segment_end < segment_start)
        continue;

      const Eigen::Vector3d start = pointAt(segment_start);
      const Eigen::Vector3d end = pointAt(segment_end);
      const Eigen::Vector2d delta = end.head<2>() - start.head<2>();
      const double length_squared = delta.squaredNorm();
      double ratio = 0.0;
      if (length_squared > 1.0e-12)
      {
        ratio = std::clamp(
            (odom_position.head<2>() - start.head<2>()).dot(delta) /
                length_squared,
            0.0,
            1.0);
      }
      const Eigen::Vector2d projected =
          start.head<2>() + ratio * delta;
      const double distance_squared =
          (projected - odom_position.head<2>()).squaredNorm();
      const double projected_progress =
          segment_start + ratio * (segment_end - segment_start);

      if (distance_squared + 1.0e-12 < best_distance_squared)
      {
        best_distance_squared = distance_squared;
        best_progress = projected_progress;
      }

      if (cumulative_lengths[segment + 1] >= search_end - 1.0e-9)
        break;
    }

    const double advance = std::max(0.0, best_progress - progress_);
    progress_ = std::min(totalLength(), progress_ + advance);
    movement_credit_ = std::max(0.0, movement_credit_ - advance);
    if (progress_ >=
        std::min(config_.departure_distance, 0.5 * totalLength()))
    {
      departed_start_ = true;
    }
    return progress_;
  }

  /**
   * @brief Interpolates the immutable route at an arc length.
   * @param arc_length Arc length in metres, clamped to route bounds.
   * @return Interpolated 3D point.
   */
  Eigen::Vector3d pointAt(double arc_length) const
  {
    if (!active())
      return Eigen::Vector3d::Zero();
    return route_->pointAt(std::clamp(arc_length, 0.0, totalLength()));
  }

  /**
   * @brief Returns the forward unit tangent at an arc length.
   * @param arc_length Arc length in metres.
   * @return Unit 3D direction, or zero when inactive.
   */
  Eigen::Vector3d tangentAt(double arc_length) const
  {
    if (!active())
      return Eigen::Vector3d::Zero();
    return route_->tangentAt(std::clamp(arc_length, 0.0, totalLength()));
  }

private:
  /** @brief Finds the segment containing an arc length. */
  std::size_t segmentIndex(double arc_length) const
  {
    if (!route_)
      return 0;
    return route_->segmentIndex(arc_length);
  }

  Config config_;
  std::optional<ReferenceRoute> route_;
  Eigen::Vector3d last_odom_position_{Eigen::Vector3d::Zero()};
  double progress_{0.0};
  double movement_credit_{0.0};
  bool closed_{false};
  bool departed_start_{false};
  bool have_last_odom_{false};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__REFERENCE_PATH_TRACKER_H_
