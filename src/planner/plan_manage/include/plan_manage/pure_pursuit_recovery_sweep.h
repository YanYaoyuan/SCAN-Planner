// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file pure_pursuit_recovery_sweep.h @brief Pure Pursuit 恢复段保守扫掠采样。 */

#ifndef PLAN_MANAGE__PURE_PURSUIT_RECOVERY_SWEEP_H_
#define PLAN_MANAGE__PURE_PURSUIT_RECOVERY_SWEEP_H_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace scan_planner
{

/** @brief 单一维度的采样数量上限，防止异常输入在取整时溢出。 */
inline constexpr int kRecoverySweepMaxAxisSamples = 4096;
/** @brief 一次恢复检查最多生成的中心位姿数；超限时返回空数组并由 FSM 急停。 */
inline constexpr std::size_t kRecoverySweepMaxPoseSamples = 25000;
/** @brief 双圆柱朝向扫掠的最大 yaw 步长，单位 rad。 */
inline constexpr double kRecoverySweepYawSampleStep = 0.10;
/** @brief 单次 TRACK 预测最多覆盖半圈转向；超过后按末端切线继续保守外推。 */
inline constexpr double kRecoverySweepMaxTrackTurn = 3.14159265358979323846;

/** @brief 一个待交给双圆柱碰撞模型检查的机体中心位姿。 */
struct RecoverySweepPose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

/**
 * @brief 与 PathTrackingController/PathCommandGovernor 对齐的恢复控制约束。
 *
 * FSM 和控制器属于不同 ROS 节点，无法共享有状态滤波器；这里使用同一组
 * 几何/底盘能力参数生成保守包络，而不是假定真机永远精确执行裸 PP 圆弧。
 */
struct RecoverySweepControlLimits
{
  double heading_error{0.0};              ///< 路径切线航向减当前机体 yaw，单位 rad。
  double heading_feedback_gain{0.35};     ///< TRACK 航向反馈增益。
  double align_yaw_gain{0.80};            ///< ALIGN 原地转向增益。
  double align_enter_error{0.80};         ///< TRACK 进入 ALIGN 的航向误差，单位 rad。
  double align_exit_error{0.45};          ///< ALIGN 返回 TRACK 的航向误差，单位 rad。
  double lateral_error_deadband{0.04};    ///< 前视点横向误差死区，单位 m。
  double heading_error_deadband{0.035};   ///< 路径切线航向误差死区，单位 rad。
  double curvature_deadband{0.04};        ///< Pure Pursuit 曲率死区，单位 1/m。
  double minimum_lookahead{0.05};         ///< 曲率计算允许的最小前视距离，单位 m。
  double min_forward_speed{0.05};         ///< SDK 最小非零前进速度，单位 m/s。
  double yaw_deadband{0.025};             ///< yaw 滤波器输入死区，单位 rad/s。
  double yaw_start_threshold{0.06};       ///< yaw 滤波器开始转向阈值，单位 rad/s。
  double min_yaw_rate{0.10};              ///< SDK 最小非零 yaw，单位 rad/s。
  double max_yaw_rate{0.50};              ///< 最终 yaw 绝对值上限，单位 rad/s。
};

/** @brief 把角度规整到 [-pi, pi]，避免跨越正负 pi 时走长方向。 */
inline double normalizeRecoveryAngle(double angle)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  // remainder 是 O(1)；异常但有限的极大角度也不会让 while 循环长期占用
  // 单线程安全回调。非有限结果会在调用方的样本数量校验中失败关闭。
  return std::remainder(angle, kTwoPi);
}

/**
 * @brief 生成从当前位姿到前视点的保守扫掠采样。
 *
 * 对机体系前视点 (x,y)，与控制器相同地使用
 *
 *   kappa = 2 y / (x^2 + y^2)
 *
 * 预测恒曲率圆弧。仅检查圆弧仍不足以覆盖 yaw 滤波关闭时沿当前航向直行、
 * TRACK 航向反馈、SDK 最小角速度量化以及急弯转向准备，因此本函数生成这些
 * 可执行极值轨迹与起终点弦的保守包络。扫掠带内每个中心位置都会完整采样
 * 所有候选朝向之间的 yaw，供 GridMap 的双圆柱模型检查机身前后端空间。
 *
 * 当前视点位于机体侧后方时，控制器会先原地转向；此时先采样原地旋转的
 * 全部中间 yaw，再采样转正后的直线接近路径。
 *
 * @param start_position 当前机体中心世界坐标。
 * @param start_yaw 当前机体世界系 yaw。
 * @param target_position 世界系前视点。
 * @param sample_distance 中心位置最大采样间距，单位 m。
 * @param control 与闭环控制器对齐的航向反馈、死区和底盘量化边界。
 * @return 需要检查的中心位姿；输入非法时返回空数组。
 */
inline std::vector<RecoverySweepPose> samplePurePursuitRecoverySweep(
    const Eigen::Vector3d &start_position,
    double start_yaw,
    const Eigen::Vector3d &target_position,
    double sample_distance,
    const RecoverySweepControlLimits &control = RecoverySweepControlLimits{})
{
  std::vector<RecoverySweepPose> samples;
  if (!start_position.allFinite() || !target_position.allFinite() ||
      !std::isfinite(start_yaw) || !std::isfinite(sample_distance) ||
      sample_distance <= 0.0 ||
      !std::isfinite(control.heading_error) ||
      !std::isfinite(control.heading_feedback_gain) ||
      control.heading_feedback_gain < 0.0 ||
      !std::isfinite(control.align_yaw_gain) ||
      control.align_yaw_gain < 0.0 ||
      !std::isfinite(control.align_enter_error) ||
      control.align_enter_error <= 0.0 ||
      !std::isfinite(control.align_exit_error) ||
      control.align_exit_error < 0.0 ||
      control.align_exit_error >= control.align_enter_error ||
      !std::isfinite(control.lateral_error_deadband) ||
      control.lateral_error_deadband < 0.0 ||
      !std::isfinite(control.heading_error_deadband) ||
      control.heading_error_deadband < 0.0 ||
      !std::isfinite(control.curvature_deadband) ||
      control.curvature_deadband < 0.0 ||
      !std::isfinite(control.minimum_lookahead) ||
      control.minimum_lookahead <= 0.0 ||
      !std::isfinite(control.min_forward_speed) ||
      control.min_forward_speed <= 0.0 ||
      !std::isfinite(control.yaw_deadband) || control.yaw_deadband < 0.0 ||
      !std::isfinite(control.yaw_start_threshold) ||
      control.yaw_start_threshold < control.yaw_deadband ||
      !std::isfinite(control.min_yaw_rate) || control.min_yaw_rate < 0.0 ||
      !std::isfinite(control.max_yaw_rate) || control.max_yaw_rate <= 0.0 ||
      control.min_yaw_rate > control.max_yaw_rate)
  {
    return samples;
  }

  const Eigen::Vector3d world_delta = target_position - start_position;
  const double planar_distance = world_delta.head<2>().norm();
  if (!std::isfinite(planar_distance))
    return samples;

  // 原地转向采样没有必要再增加一个真机参数。0.10rad 对双圆柱 offset=0.18m
  // 只造成约 1.8cm 的端点位移，小于允许的最小中心采样间距 2cm。
  const auto append_yaw_sweep =
      [&samples](const Eigen::Vector3d &position,
                 double from_yaw,
                 double to_yaw)
      {
        const double yaw_delta = normalizeRecoveryAngle(to_yaw - from_yaw);
        const double raw_sample_count =
            std::ceil(std::abs(yaw_delta) /
                      kRecoverySweepYawSampleStep);
        if (!std::isfinite(raw_sample_count) ||
            raw_sample_count > kRecoverySweepMaxAxisSamples)
        {
          return false;
        }
        const int yaw_sample_count =
            static_cast<int>(raw_sample_count);
        for (int index = 0; index <= yaw_sample_count; ++index)
        {
          if (samples.size() >= kRecoverySweepMaxPoseSamples)
            return false;
          const double ratio =
              yaw_sample_count == 0
                  ? 0.0
                  : static_cast<double>(index) / yaw_sample_count;
          samples.push_back(
              {position, from_yaw + ratio * yaw_delta});
        }
        return true;
      };

  // 即使当前位置恰好等于前视点、且无需 ALIGN，也必须至少返回一个合法
  // 位姿。FSM 把空数组定义为“验证无法完成”并会 fail-closed 急停。
  samples.push_back({start_position, start_yaw});

  // controller 的 ALIGN 由路径切线航向触发，未必等于前视点 bearing。
  // FSM 不掌握控制器上一周期的滞回状态；处在 ALIGN 滞回带时保守覆盖
  // 当前位置从当前 yaw 到切线 yaw 的完整双圆柱扫掠。
  const double raw_heading_error =
      normalizeRecoveryAngle(control.heading_error);
  const double heading_error =
      std::abs(raw_heading_error) < control.heading_error_deadband
          ? 0.0
          : raw_heading_error;
  const double align_raw_yaw = std::clamp(
      control.align_yaw_gain * heading_error,
      -control.max_yaw_rate,
      control.max_yaw_rate);
  // ALIGN 带有进入/退出滞回。FSM 不知道控制器上一周期的 aligning 状态，
  // 因此误差仍高于退出阈值且命令足以启动 yaw 滤波时，必须按“可能仍在
  // ALIGN”处理，并覆盖当前位置到路径切线方向的完整机身旋转扫掠。
  const bool align_is_possible =
      std::abs(heading_error) > control.align_exit_error &&
      std::abs(align_raw_yaw) + 1.0e-12 >= control.yaw_start_threshold;
  if (align_is_possible && !append_yaw_sweep(
          start_position,
          start_yaw,
          start_yaw + heading_error))
  {
    return {};
  }
  if (std::abs(heading_error) >= control.align_enter_error)
  {
    // 超过进入阈值时 controller 无论上一周期状态如何都必定进入 ALIGN，
    // 本周期不会平移；下一次安全回调会用转动后的 odom 重新验证 TRACK。
    return samples;
  }

  if (planar_distance <= 1.0e-9)
    return samples;

  const double cosine = std::cos(start_yaw);
  const double sine = std::sin(start_yaw);
  const double local_x =
      cosine * world_delta.x() + sine * world_delta.y();
  const double local_y =
      -sine * world_delta.x() + cosine * world_delta.y();
  const double chord_yaw = std::atan2(world_delta.y(), world_delta.x());

  // PathTrackingController 对 local_x < 0.02m 的目标先原地转正。
  if (local_x < 0.02)
  {
    if (!append_yaw_sweep(start_position, start_yaw, chord_yaw))
      return {};
    const double raw_chord_sample_count =
        std::ceil(planar_distance / sample_distance);
    if (!std::isfinite(raw_chord_sample_count) ||
        raw_chord_sample_count > kRecoverySweepMaxAxisSamples)
    {
      return {};
    }
    const int chord_sample_count = std::max(
        1, static_cast<int>(raw_chord_sample_count));
    for (int index = 1; index <= chord_sample_count; ++index)
    {
      if (samples.size() >= kRecoverySweepMaxPoseSamples)
        return {};
      const double ratio =
          static_cast<double>(index) / chord_sample_count;
      samples.push_back(
          {start_position + ratio * world_delta, chord_yaw});
    }
    return samples;
  }

  const double raw_lookahead_squared =
      local_x * local_x + local_y * local_y;
  const double filtered_y =
      std::abs(local_y) < control.lateral_error_deadband ? 0.0 : local_y;
  const double lookahead_squared = std::max(
      control.minimum_lookahead * control.minimum_lookahead,
      raw_lookahead_squared);
  double curvature = 2.0 * filtered_y / lookahead_squared;
  if (std::abs(curvature) < control.curvature_deadband)
    curvature = 0.0;
  const double turn_angle = 2.0 * std::atan2(local_y, local_x);
  const bool use_arc =
      std::isfinite(curvature) && std::isfinite(turn_angle) &&
      std::abs(curvature) > 1.0e-8;
  const double arc_length = use_arc
      ? std::abs(turn_angle / curvature)
      : planar_distance;

  // TRACK 实际发布的是 w=v*kappa+k_heading*e_heading，并受 SDK 最小非零
  // yaw 量化影响。用最小线速处的有效曲率，以及 yaw 第一次越过输入死区
  // 时可能被量化成 min_yaw 的有效曲率，构造裸 PP 之外的转弯包络。
  const double heading_yaw =
      control.heading_feedback_gain * heading_error;
  const auto raw_yaw_at_speed =
      [&control, curvature, heading_yaw](double speed)
      {
        return std::clamp(
            speed * curvature + heading_yaw,
            -control.max_yaw_rate,
            control.max_yaw_rate);
      };

  if (std::abs(curvature) > 1.0e-8)
  {
    const double curvature_sign = std::copysign(1.0, curvature);
    const double minimum_required_yaw =
        control.min_forward_speed * std::abs(curvature) +
        curvature_sign * heading_yaw;
    if (minimum_required_yaw > control.max_yaw_rate + 1.0e-12)
    {
      // 即使不预留 controller 的 yaw_rate_reserve，最小可执行线速度也已
      // 超出角速度能力；PathCommandGovernor 此时只能原地转向准备。覆盖
      // 当前 yaw 到目标 bearing 和路径切线两种可能方向即可，下一次 20Hz
      // 安全回调会在允许平移前用新的机体坐标重新计算扫掠区域。
      if (!append_yaw_sweep(start_position, start_yaw, chord_yaw) ||
          !append_yaw_sweep(
              start_position,
              start_yaw,
              start_yaw + heading_error))
      {
        return {};
      }
      return samples;
    }
  }

  std::vector<double> candidate_curvatures;
  const auto append_candidate_curvature =
      [&candidate_curvatures](double candidate)
      {
        if (!std::isfinite(candidate))
          return false;
        const bool duplicate = std::any_of(
            candidate_curvatures.begin(),
            candidate_curvatures.end(),
            [candidate](double existing)
            {
              return std::abs(existing - candidate) <= 1.0e-8;
            });
        if (!duplicate)
          candidate_curvatures.push_back(candidate);
        return true;
      };
  // yaw 滤波关闭或换向抑制时，机器人会沿当前航向直行；这条轨迹通常
  // 位于 PP 圆弧—目标弦扫掠带之外，必须显式作为候选边界。
  if (!append_candidate_curvature(0.0) ||
      !append_candidate_curvature(curvature))
    return {};

  const double raw_min_yaw = raw_yaw_at_speed(control.min_forward_speed);
  const auto append_if_outside_curvature_envelope =
      [&candidate_curvatures, &append_candidate_curvature](double candidate)
      {
        if (!std::isfinite(candidate))
          return false;
        const auto limits = std::minmax_element(
            candidate_curvatures.begin(), candidate_curvatures.end());
        if (candidate + 1.0e-8 < *limits.first ||
            candidate - 1.0e-8 > *limits.second)
        {
          return append_candidate_curvature(candidate);
        }
        return true;
      };
  if (!append_if_outside_curvature_envelope(
          raw_min_yaw / control.min_forward_speed))
  {
    return {};
  }

  // turning_ 可能由上一周期保持；此时即便当前 raw yaw 只略高于输入死区，
  // SDK 最终也只能发布至少 min_yaw_rate。以最小可执行线速反推的曲率是
  // 该量化误差的最保守边界。若 kappa 与 heading 反号，raw 会随速度过零，
  // 正负两侧都可能出现，必须分别纳入包络。
  bool may_turn_positive = raw_min_yaw > control.yaw_deadband;
  bool may_turn_negative = raw_min_yaw < -control.yaw_deadband;
  if (curvature > 1.0e-12)
    may_turn_positive = true;
  else if (curvature < -1.0e-12)
    may_turn_negative = true;
  else if (heading_yaw > control.yaw_deadband)
    may_turn_positive = true;
  else if (heading_yaw < -control.yaw_deadband)
    may_turn_negative = true;

  const double quantized_curvature =
      control.min_yaw_rate / control.min_forward_speed;
  if ((may_turn_positive &&
       !append_if_outside_curvature_envelope(quantized_curvature)) ||
      (may_turn_negative &&
       !append_if_outside_curvature_envelope(-quantized_curvature)))
  {
    return {};
  }

  // 发布 yaw 在“关闭、连续 raw、最小非零量化”之间变化时，有效曲率落在
  // 上述候选的闭区间内。扫掠带只需连接最小/最大两个极值到目标弦；中间
  // 候选继续全部做精确中心线检查，但不再重复生成重叠的二维 ribbon。
  const auto curvature_limits = std::minmax_element(
      candidate_curvatures.begin(), candidate_curvatures.end());
  std::vector<double> envelope_curvatures{*curvature_limits.first};
  if (*curvature_limits.second - *curvature_limits.first > 1.0e-8)
    envelope_curvatures.push_back(*curvature_limits.second);

  double maximum_candidate_turn = std::abs(turn_angle);
  for (double candidate : envelope_curvatures)
  {
    maximum_candidate_turn = std::max(
        maximum_candidate_turn,
        std::min(
            kRecoverySweepMaxTrackTurn,
            std::abs(candidate * arc_length)));
  }
  const double raw_longitudinal_sample_count = std::max(
      std::ceil(std::max(planar_distance, arc_length) / sample_distance),
      std::ceil(maximum_candidate_turn /
                kRecoverySweepYawSampleStep));
  if (!std::isfinite(raw_longitudinal_sample_count) ||
      raw_longitudinal_sample_count > kRecoverySweepMaxAxisSamples)
  {
    return {};
  }
  const int longitudinal_sample_count = std::max(
      1, static_cast<int>(raw_longitudinal_sample_count));
  for (int index = 0; index <= longitudinal_sample_count; ++index)
  {
    const double ratio =
        static_cast<double>(index) / longitudinal_sample_count;
    const Eigen::Vector3d chord_position =
        start_position + ratio * world_delta;

    std::vector<Eigen::Vector3d> candidate_positions{chord_position};
    std::vector<double> candidate_yaws{chord_yaw};
    const double travelled_length = ratio * arc_length;
    const auto append_candidate_geometry =
        [&candidate_positions,
         &candidate_yaws,
         &chord_position,
         &start_position,
         cosine,
         sine,
         start_yaw,
         travelled_length](double candidate_curvature)
        {
          double local_candidate_x = travelled_length;
          double local_candidate_y = 0.0;
          double candidate_turn = 0.0;
          if (std::abs(candidate_curvature) > 1.0e-8)
          {
            const double turning_length = std::min(
                travelled_length,
                kRecoverySweepMaxTrackTurn /
                    std::abs(candidate_curvature));
            candidate_turn = candidate_curvature * turning_length;
            local_candidate_x =
                std::sin(candidate_turn) / candidate_curvature;
            local_candidate_y =
                (1.0 - std::cos(candidate_turn)) / candidate_curvature;
            // 固定前视点下把 heading_yaw 当恒定曲率无限积分会人为生成多圈
            // 螺旋；真实 controller 每 20ms 重算航向，最迟半圈前目标已转到
            // 机体后方并切入原地对准。达到半圈后沿当时切线外推，既保持
            // 有界又覆盖下一次安全回调前可能到达的中心区域。
            const double remaining_length =
                travelled_length - turning_length;
            local_candidate_x +=
                remaining_length * std::cos(candidate_turn);
            local_candidate_y +=
                remaining_length * std::sin(candidate_turn);
          }
          Eigen::Vector3d candidate_position = chord_position;
          candidate_position.x() =
              start_position.x() + cosine * local_candidate_x -
              sine * local_candidate_y;
          candidate_position.y() =
              start_position.y() + sine * local_candidate_x +
              cosine * local_candidate_y;
          candidate_positions.push_back(candidate_position);
          candidate_yaws.push_back(start_yaw + candidate_turn);
        };
    for (double candidate_curvature : envelope_curvatures)
    {
      append_candidate_geometry(candidate_curvature);
    }

    const bool controller_arc_is_envelope = std::any_of(
        envelope_curvatures.begin(),
        envelope_curvatures.end(),
        [curvature](double candidate)
        {
          return std::abs(candidate - curvature) <= 1.0e-8;
        });
    std::vector<double> all_candidate_yaws = candidate_yaws;
    if (!controller_arc_is_envelope)
    {
      // 实际 PathTrackingController 曲率若落在包络内部，仍单独检查其精确
      // 中心线/朝向；只省略它与 chord 之间已被极值带覆盖的重复笛卡尔积。
      double local_x_on_arc = travelled_length;
      double local_y_on_arc = 0.0;
      const double controller_turn = curvature * travelled_length;
      if (std::abs(curvature) > 1.0e-8)
      {
        local_x_on_arc = std::sin(controller_turn) / curvature;
        local_y_on_arc = (1.0 - std::cos(controller_turn)) / curvature;
      }
      Eigen::Vector3d controller_position = chord_position;
      controller_position.x() =
          start_position.x() + cosine * local_x_on_arc -
          sine * local_y_on_arc;
      controller_position.y() =
          start_position.y() + sine * local_x_on_arc +
          cosine * local_y_on_arc;
      if (samples.size() >= kRecoverySweepMaxPoseSamples)
        return {};
      samples.push_back(
          {controller_position, start_yaw + controller_turn});
      all_candidate_yaws.push_back(start_yaw + controller_turn);
    }

    const auto append_pose_envelope =
        [&samples, &all_candidate_yaws](
            const Eigen::Vector3d &position,
            double first_yaw,
            double second_yaw)
        {
          std::vector<double> appended_yaws;
          const auto append_exact_yaw =
              [&samples, &appended_yaws, &position](double yaw)
              {
                const bool duplicate = std::any_of(
                    appended_yaws.begin(),
                    appended_yaws.end(),
                    [yaw](double existing)
                    {
                      return std::abs(normalizeRecoveryAngle(
                          existing - yaw)) <= 1.0e-10;
                    });
                if (duplicate)
                  return true;
                if (samples.size() >= kRecoverySweepMaxPoseSamples)
                  return false;
                samples.push_back({position, yaw});
                appended_yaws.push_back(yaw);
                return true;
              };

          // 先显式加入每条候选中心线的精确 yaw。不能只依赖等距区间端点，
          // 否则候选 yaw 落在半格位置时可能被离散采样跨过。
          for (double candidate_yaw : all_candidate_yaws)
          {
            if (!append_exact_yaw(candidate_yaw))
              return false;
          }

          const double yaw_span = normalizeRecoveryAngle(
              second_yaw - first_yaw);
          const double raw_yaw_count = std::ceil(
              std::abs(yaw_span) / kRecoverySweepYawSampleStep);
          if (!std::isfinite(raw_yaw_count) ||
              raw_yaw_count > kRecoverySweepMaxAxisSamples)
          {
            return false;
          }
          const int yaw_count = static_cast<int>(raw_yaw_count);
          // 两个端点也通过同一个去重入口显式加入，中间点才做等距插值。
          if (!append_exact_yaw(first_yaw) ||
              !append_exact_yaw(second_yaw))
          {
            return false;
          }
          for (int yaw_index = 1; yaw_index < yaw_count; ++yaw_index)
          {
            const double yaw_ratio =
                static_cast<double>(yaw_index) / yaw_count;
            if (!append_exact_yaw(
                    first_yaw + yaw_ratio * yaw_span))
            {
              return false;
            }
          }
          return true;
        };

    if (candidate_positions.size() == 1)
    {
      if (!append_pose_envelope(
              candidate_positions.front(), chord_yaw, chord_yaw))
        return {};
      continue;
    }

    // 第 0 条始终是起终点弦。把每个可执行候选中心线分别连接到这条弦，
    // 即可覆盖“候选轨迹—目标直达线”的扫掠带；候选轨迹彼此之间已经
    // 通过共同的弦带重叠。避免对 N 条候选做 O(N^2) 重复 ribbon，否则
    // 正常 0.45m 前视也可能触发总样本预算并导致无意义的 fail-closed。
    for (std::size_t second = 1;
         second < candidate_positions.size();
         ++second)
    {
      const double ribbon_width =
          (candidate_positions[second].head<2>() -
           candidate_positions.front().head<2>()).norm();
      const double raw_ribbon_count =
          std::ceil(ribbon_width / sample_distance);
      if (!std::isfinite(raw_ribbon_count) ||
          raw_ribbon_count > kRecoverySweepMaxAxisSamples)
      {
        return {};
      }
      const int ribbon_count = static_cast<int>(raw_ribbon_count);
      for (int ribbon_index = 0;
           ribbon_index <= ribbon_count;
           ++ribbon_index)
      {
        const double ribbon_ratio = ribbon_count == 0
            ? 0.0
            : static_cast<double>(ribbon_index) / ribbon_count;
        const Eigen::Vector3d position =
            candidate_positions.front() + ribbon_ratio *
            (candidate_positions[second] - candidate_positions.front());
        // 每个 ribbon 位置都覆盖该候选轨迹 yaw 与固定 chord bearing yaw
        // 之间的全部姿态，而不是只取一个配对插值 yaw。
        if (!append_pose_envelope(
                position, chord_yaw, candidate_yaws[second]))
          return {};
      }
    }
  }

  return samples;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE__PURE_PURSUIT_RECOVERY_SWEEP_H_
