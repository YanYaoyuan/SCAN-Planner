// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file local_trajectory_tracker.h @brief 基于实际里程计的局部轨迹空间进度跟踪器。 */

#ifndef PLAN_MANAGE__LOCAL_TRAJECTORY_TRACKER_H_
#define PLAN_MANAGE__LOCAL_TRAJECTORY_TRACKER_H_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

namespace scan_planner
{

/**
 * @brief 把机器人位置单调投影到局部 B-spline 的采样折线上。
 *
 * 控制器和 FSM 都使用这个类，确保“已经执行到哪里”只由真实位姿决定，
 * 不再由 now - start_time 推算。movement credit 会限制一次投影最多向前
 * 走多远，避免定位噪声或自交路径把静止机器人瞬间投影到很远的位置。
 */
class LocalTrajectoryTracker
{
public:
  /** @brief 空间投影的安全限制。 */
  struct Config
  {
    double max_projection_advance{0.75};  ///< 单次累计可搜索的最大前向弧长，单位 m。
    double movement_credit_scale{1.5};    ///< 每移动 1m 可补充的轨迹弧长额度。
    double movement_deadband{0.01};       ///< 小于该值的累计定位抖动不补充额度。
    double initial_credit{0.15};          ///< 新轨迹起点允许吸收的初始位置误差，单位 m。
  };

  LocalTrajectoryTracker()
  {
    setConfig(Config{});
  }

  explicit LocalTrajectoryTracker(const Config &config)
  {
    setConfig(config);
  }

  /** @brief 更新投影配置；非法值会抛出异常，避免带病运行。 */
  void setConfig(const Config &config)
  {
    if (!std::isfinite(config.max_projection_advance) ||
        config.max_projection_advance <= 0.0 ||
        !std::isfinite(config.movement_credit_scale) ||
        config.movement_credit_scale < 1.0 ||
        !std::isfinite(config.movement_deadband) ||
        config.movement_deadband < 0.0 ||
        !std::isfinite(config.initial_credit) ||
        config.initial_credit < 0.0)
    {
      throw std::invalid_argument("invalid local-trajectory tracker configuration");
    }
    config_ = config;
  }

  /**
   * @brief 开始跟踪一条新的采样轨迹。
   * @param points 按时间排列的轨迹点。
   * @param times 每个轨迹点对应的 B-spline 参数时间。
   * @param odom_position 接收轨迹时机器人真实位置。
   */
  void reset(
      const std::vector<Eigen::Vector3d> &points,
      const std::vector<double> &times,
      const Eigen::Vector3d &odom_position)
  {
    if (points.size() != times.size() || points.size() < 2 ||
        !odom_position.allFinite())
    {
      throw std::invalid_argument("local trajectory requires matching finite samples");
    }

    clear();
    points_.reserve(points.size());
    times_.reserve(times.size());
    cumulative_lengths_.reserve(points.size());

    double previous_time = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < points.size(); ++index)
    {
      if (!points[index].allFinite() || !std::isfinite(times[index]) ||
          times[index] + 1.0e-12 < previous_time)
      {
        clear();
        throw std::invalid_argument("local trajectory contains invalid samples");
      }
      previous_time = times[index];

      if (!points_.empty())
      {
        const double planar_length =
            (points[index].head<2>() - points_.back().head<2>()).norm();
        // 纯 XY 控制无法区分同一平面位置上的重复点。保留第一个时间，
        // 防止机器人静止时仅靠重复采样点把执行时间向前推进。
        if (planar_length <= 1.0e-8)
          continue;
      }

      points_.push_back(points[index]);
      times_.push_back(times[index]);
      cumulative_lengths_.push_back(
          cumulative_lengths_.empty()
              ? 0.0
              : cumulative_lengths_.back() +
                    (points_.back().head<2>() -
                     points_[points_.size() - 2].head<2>()).norm());
    }

    if (points_.size() < 2 || cumulative_lengths_.back() <= 1.0e-8)
    {
      clear();
      throw std::invalid_argument("local trajectory has zero planar length");
    }

    progress_ = 0.0;
    movement_credit_ =
        std::min(config_.initial_credit, config_.max_projection_advance);
    last_odom_position_ = odom_position;
    have_last_odom_ = true;
  }

  /** @brief 清除轨迹及全部进度状态。 */
  void clear()
  {
    points_.clear();
    times_.clear();
    cumulative_lengths_.clear();
    progress_ = 0.0;
    movement_credit_ = 0.0;
    have_last_odom_ = false;
  }

  /** @brief 当前是否有可跟踪的有效轨迹。 */
  bool active() const
  {
    return points_.size() >= 2 && times_.size() == points_.size() &&
           cumulative_lengths_.size() == points_.size();
  }

  /**
   * @brief 用最新里程计更新单调空间进度。
   * @return 当前轨迹弧长进度，单位 m。
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

    const double search_end = std::min(
        totalLength(),
        progress_ +
            std::min(config_.max_projection_advance, movement_credit_));
    if (search_end <= progress_ + 1.0e-10)
      return progress_;

    double best_progress = progress_;
    double best_distance_squared =
        (pointAt(progress_).head<2>() - odom_position.head<2>()).squaredNorm();

    for (std::size_t segment = segmentIndex(progress_);
         segment + 1 < points_.size(); ++segment)
    {
      const double segment_start =
          std::max(progress_, cumulative_lengths_[segment]);
      const double segment_end =
          std::min(search_end, cumulative_lengths_[segment + 1]);
      if (segment_end < segment_start)
        continue;

      const Eigen::Vector2d start = pointAt(segment_start).head<2>();
      const Eigen::Vector2d end = pointAt(segment_end).head<2>();
      const Eigen::Vector2d delta = end - start;
      const double length_squared = delta.squaredNorm();
      double ratio = 0.0;
      if (length_squared > 1.0e-12)
      {
        ratio = std::clamp(
            (odom_position.head<2>() - start).dot(delta) / length_squared,
            0.0,
            1.0);
      }
      const Eigen::Vector2d projection = start + ratio * delta;
      const double distance_squared =
          (projection - odom_position.head<2>()).squaredNorm();
      const double candidate =
          segment_start + ratio * (segment_end - segment_start);

      if (distance_squared + 1.0e-12 < best_distance_squared)
      {
        best_distance_squared = distance_squared;
        best_progress = candidate;
      }

      if (cumulative_lengths_[segment + 1] >= search_end - 1.0e-10)
        break;
    }

    const double advance = std::max(0.0, best_progress - progress_);
    progress_ = std::min(totalLength(), progress_ + advance);
    movement_credit_ = std::max(0.0, movement_credit_ - advance);
    return progress_;
  }

  /** @brief 当前单调弧长进度，单位 m。 */
  double progress() const { return progress_; }

  /**
   * @brief 从同一轨迹的可信外部状态单调恢复进度。
   *
   * 仅允许前进且自动限制在总弧长内，供控制器重启后采用 FSM 心跳进度；
   * 绝不允许旧消息把本地进度拉回去。
   */
  void advanceTo(double arc_length)
  {
    if (!active() || !std::isfinite(arc_length))
      return;
    progress_ = std::max(
        progress_, std::clamp(arc_length, 0.0, totalLength()));
  }

  /** @brief 轨迹总平面弧长，单位 m。 */
  double totalLength() const
  {
    return active() ? cumulative_lengths_.back() : 0.0;
  }

  /** @brief 未执行轨迹弧长，单位 m。 */
  double remainingLength() const
  {
    return std::max(0.0, totalLength() - progress_);
  }

  /** @brief 当前空间进度所对应的 B-spline 参数时间。 */
  double progressTime() const
  {
    return timeAt(progress_);
  }

  /** @brief 在给定弧长处线性插值轨迹点。 */
  Eigen::Vector3d pointAt(double arc_length) const
  {
    if (!active())
      return Eigen::Vector3d::Zero();
    const double clamped = std::clamp(arc_length, 0.0, totalLength());
    const std::size_t segment = segmentIndex(clamped);
    if (segment + 1 >= points_.size())
      return points_.back();
    const double length =
        cumulative_lengths_[segment + 1] - cumulative_lengths_[segment];
    if (length <= 1.0e-12)
      return points_[segment];
    const double ratio =
        (clamped - cumulative_lengths_[segment]) / length;
    return points_[segment] + ratio * (points_[segment + 1] - points_[segment]);
  }

  /** @brief 把弧长映射回原 B-spline 参数时间。 */
  double timeAt(double arc_length) const
  {
    if (!active())
      return 0.0;
    const double clamped = std::clamp(arc_length, 0.0, totalLength());
    const std::size_t segment = segmentIndex(clamped);
    if (segment + 1 >= times_.size())
      return times_.back();
    const double length =
        cumulative_lengths_[segment + 1] - cumulative_lengths_[segment];
    if (length <= 1.0e-12)
      return times_[segment];
    const double ratio =
        (clamped - cumulative_lengths_[segment]) / length;
    return times_[segment] + ratio * (times_[segment + 1] - times_[segment]);
  }

  /** @brief 把 B-spline 参数时间映射成采样折线弧长。 */
  double arcLengthAtTime(double time) const
  {
    if (!active() || !std::isfinite(time))
      return 0.0;
    if (time <= times_.front())
      return 0.0;
    if (time >= times_.back())
      return totalLength();
    const auto upper = std::lower_bound(times_.begin(), times_.end(), time);
    const std::size_t upper_index =
        static_cast<std::size_t>(std::distance(times_.begin(), upper));
    const std::size_t lower_index = upper_index - 1;
    const double dt = times_[upper_index] - times_[lower_index];
    if (dt <= 1.0e-12)
      return cumulative_lengths_[lower_index];
    const double ratio = (time - times_[lower_index]) / dt;
    return cumulative_lengths_[lower_index] +
        ratio * (cumulative_lengths_[upper_index] -
                 cumulative_lengths_[lower_index]);
  }

  /** @brief 返回给定弧长所在的采样段起点索引。 */
  std::size_t segmentIndex(double arc_length) const
  {
    if (!active())
      return 0;
    const double clamped = std::clamp(arc_length, 0.0, totalLength());
    const auto upper = std::upper_bound(
        cumulative_lengths_.begin(), cumulative_lengths_.end(), clamped);
    if (upper == cumulative_lengths_.begin())
      return 0;
    return std::min(
        static_cast<std::size_t>(
            std::distance(cumulative_lengths_.begin(), upper) - 1),
        points_.size() - 1);
  }

  const std::vector<Eigen::Vector3d> &points() const { return points_; }
  const std::vector<double> &times() const { return times_; }
  const std::vector<double> &arcLengths() const { return cumulative_lengths_; }

private:
  Config config_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> times_;
  std::vector<double> cumulative_lengths_;
  Eigen::Vector3d last_odom_position_{Eigen::Vector3d::Zero()};
  double progress_{0.0};
  double movement_credit_{0.0};
  bool have_last_odom_{false};
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__LOCAL_TRAJECTORY_TRACKER_H_
