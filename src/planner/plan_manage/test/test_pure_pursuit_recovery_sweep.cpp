// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_pure_pursuit_recovery_sweep.cpp @brief 恢复段保守扫掠几何测试。 */

#include <gtest/gtest.h>

#include <plan_manage/pure_pursuit_recovery_sweep.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace scan_planner
{
namespace
{

bool containsPosition(
    const std::vector<RecoverySweepPose> &samples,
    const Eigen::Vector3d &expected,
    double tolerance = 1.0e-6)
{
  return std::any_of(
      samples.begin(),
      samples.end(),
      [&expected, tolerance](const RecoverySweepPose &sample)
      {
        return (sample.position - expected).norm() <= tolerance;
      });
}

bool containsPose(
    const std::vector<RecoverySweepPose> &samples,
    const Eigen::Vector3d &expected_position,
    double expected_yaw,
    double tolerance = 1.0e-6)
{
  return std::any_of(
      samples.begin(),
      samples.end(),
      [&expected_position, expected_yaw, tolerance](
          const RecoverySweepPose &sample)
      {
        return (sample.position - expected_position).norm() <= tolerance &&
            std::abs(normalizeRecoveryAngle(sample.yaw - expected_yaw)) <=
                tolerance;
      });
}

TEST(PurePursuitRecoverySweep, SamplesPredictedArcWithChangingYaw)
{
  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  const Eigen::Vector3d target(0.20, 0.40, 0.0);
  const auto samples = samplePurePursuitRecoverySweep(
      start, 0.0, target, 0.05);

  ASSERT_FALSE(samples.empty());
  EXPECT_TRUE(containsPosition(samples, start));
  EXPECT_TRUE(containsPosition(samples, target));

  // x=0.2,y=0.4 时 kappa=4，半程转角为 atan2(0.4,0.2)。
  const double half_turn = std::atan2(0.40, 0.20);
  const Eigen::Vector3d arc_midpoint(
      std::sin(half_turn) / 4.0,
      (1.0 - std::cos(half_turn)) / 4.0,
      0.0);
  // yaw 步长也参与纵向采样数量，样本数不保证为偶数；验证理论半程圆弧
  // 落在一个空间采样半径内，而不是错误要求恰好存在 ratio=0.5。
  EXPECT_TRUE(containsPosition(samples, arc_midpoint, 0.03));

  const auto maximum_yaw = std::max_element(
      samples.begin(),
      samples.end(),
      [](const RecoverySweepPose &lhs, const RecoverySweepPose &rhs)
      {
        return lhs.yaw < rhs.yaw;
      });
  ASSERT_NE(maximum_yaw, samples.end());
  EXPECT_NEAR(maximum_yaw->yaw, 2.0 * half_turn, 1.0e-9);
}

TEST(PurePursuitRecoverySweep, FillsRegionBetweenArcAndChord)
{
  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  const Eigen::Vector3d target(0.20, 0.40, 0.0);
  const auto samples = samplePurePursuitRecoverySweep(
      start, 0.0, target, 0.05);

  const double half_turn = std::atan2(0.40, 0.20);
  constexpr double curvature = 4.0;
  const double turn_angle = 2.0 * half_turn;
  const double planar_distance = target.head<2>().norm();
  const double arc_length = std::abs(turn_angle / curvature);
  const int longitudinal_count = static_cast<int>(std::max(
      std::ceil(std::max(planar_distance, arc_length) / 0.05),
      std::ceil(std::abs(turn_angle) /
                kRecoverySweepYawSampleStep)));
  const int slice_index = longitudinal_count / 2;
  const double slice_ratio =
      static_cast<double>(slice_index) / longitudinal_count;
  const double slice_turn = slice_ratio * turn_angle;
  const Eigen::Vector3d arc_slice(
      std::sin(slice_turn) / curvature,
      (1.0 - std::cos(slice_turn)) / curvature,
      0.0);
  const Eigen::Vector3d chord_slice = slice_ratio * target;
  ASSERT_TRUE(containsPosition(samples, arc_slice, 1.0e-9));
  ASSERT_TRUE(containsPosition(samples, chord_slice, 1.0e-9));

  const double ribbon_width =
      (chord_slice.head<2>() - arc_slice.head<2>()).norm();
  const int ribbon_count = std::max(
      1, static_cast<int>(std::ceil(ribbon_width / 0.05)));
  ASSERT_GT(ribbon_count, 1);

  // 任选内部 1/ribbon_count 位置，必须同时覆盖圆弧 yaw 和平滑直线候选
  // yaw，而不是只保存一个配对插值朝向。
  const Eigen::Vector3d interior_position =
      arc_slice + (chord_slice - arc_slice) /
          static_cast<double>(ribbon_count);
  const double arc_yaw = slice_turn;
  // 直线弦在任意位置都保持 target bearing，不能错误地从 start_yaw 插值。
  const double chord_motion_yaw = half_turn;
  EXPECT_TRUE(containsPose(
      samples, interior_position, arc_yaw, 1.0e-9));
  EXPECT_TRUE(containsPose(
      samples, interior_position, chord_motion_yaw, 1.0e-9));
}

TEST(PurePursuitRecoverySweep, SmallLateralOffsetIsStillChecked)
{
  // 1cm 横向分量小于原先 5cm 的恢复门槛；新逻辑仍必须产生完整检查样本。
  const Eigen::Vector3d target(0.45, 0.01, 0.0);
  const auto samples = samplePurePursuitRecoverySweep(
      Eigen::Vector3d::Zero(), 0.0, target, 0.05);

  ASSERT_GT(samples.size(), 2u);
  EXPECT_TRUE(containsPosition(samples, target, 1.0e-9));
  EXPECT_TRUE(std::all_of(
      samples.begin(),
      samples.end(),
      [](const RecoverySweepPose &sample)
      {
        return sample.position.allFinite() && std::isfinite(sample.yaw);
      }));
}

TEST(PurePursuitRecoverySweep, TargetBehindSamplesRotationBeforeTranslation)
{
  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  const Eigen::Vector3d target(-0.20, 0.10, 0.0);
  const auto samples = samplePurePursuitRecoverySweep(
      start, 0.0, target, 0.05);

  const double target_yaw = std::atan2(target.y(), target.x());
  int stationary_samples = 0;
  bool contains_rotated_pose = false;
  for (const RecoverySweepPose &sample : samples)
  {
    if ((sample.position - start).norm() <= 1.0e-12)
    {
      ++stationary_samples;
      if (std::abs(normalizeRecoveryAngle(sample.yaw - target_yaw)) < 1.0e-9)
        contains_rotated_pose = true;
    }
  }
  EXPECT_GT(stationary_samples, 2);
  EXPECT_TRUE(contains_rotated_pose);
  EXPECT_TRUE(containsPosition(samples, target, 1.0e-9));
}

TEST(PurePursuitRecoverySweep, CoincidentTargetStillReturnsCheckedStartPose)
{
  const Eigen::Vector3d start(1.0, -2.0, 0.3);
  const auto samples = samplePurePursuitRecoverySweep(
      start, 0.4, start, 0.05);

  ASSERT_FALSE(samples.empty());
  EXPECT_TRUE(containsPose(samples, start, 0.4, 1.0e-12));
}

TEST(PurePursuitRecoverySweep, AlignCoversRotationAtCurrentPosition)
{
  RecoverySweepControlLimits control;
  control.heading_error = 0.80;
  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  const auto samples = samplePurePursuitRecoverySweep(
      start, 0.0, Eigen::Vector3d(0.45, 0.0, 0.0), 0.05, control);

  ASSERT_GT(samples.size(), 2u);
  EXPECT_TRUE(containsPose(samples, start, 0.80, 1.0e-12));
  EXPECT_TRUE(std::all_of(
      samples.begin(),
      samples.end(),
      [&start](const RecoverySweepPose &sample)
      {
        return (sample.position - start).norm() <= 1.0e-12;
      }));
}

TEST(PurePursuitRecoverySweep, TrackIncludesHeadingFeedbackCurvature)
{
  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  const Eigen::Vector3d target(0.45, 0.10, 0.0);
  RecoverySweepControlLimits control;
  control.heading_error = 0.30;
  const auto samples = samplePurePursuitRecoverySweep(
      start, 0.0, target, 0.05, control);

  ASSERT_FALSE(samples.empty());
  const double base_curvature =
      2.0 * target.y() / target.head<2>().squaredNorm();
  const double arc_length = std::abs(
      2.0 * std::atan2(target.y(), target.x()) / base_curvature);
  const double heading_yaw =
      control.heading_feedback_gain * control.heading_error;
  const double effective_curvature =
      (control.min_forward_speed * base_curvature + heading_yaw) /
      control.min_forward_speed;
  const double effective_turn = effective_curvature * arc_length;
  const Eigen::Vector3d expected_extreme(
      std::sin(effective_turn) / effective_curvature,
      (1.0 - std::cos(effective_turn)) / effective_curvature,
      0.0);

  EXPECT_TRUE(containsPose(
      samples, expected_extreme, effective_turn, 1.0e-9));
}

TEST(PurePursuitRecoverySweep, IncludesMinimumLinearAndYawEnvelope)
{
  // 该目标产生约 1/m 的裸曲率，最小线速下 raw yaw=0.05rad/s。
  // 若 yaw 滤波由上一周期保持 turning，SDK 会把它量化到 0.10rad/s，
  // 因而必须额外覆盖有效曲率 0.10/0.05=2/m 的极值轨迹。
  const double target_y = 1.0 - std::sqrt(1.0 - 0.45 * 0.45);
  const Eigen::Vector3d target(0.45, target_y, 0.0);
  RecoverySweepControlLimits control;
  const auto samples = samplePurePursuitRecoverySweep(
      Eigen::Vector3d::Zero(), 0.0, target, 0.05, control);

  ASSERT_FALSE(samples.empty());
  const double base_curvature =
      2.0 * target.y() / target.head<2>().squaredNorm();
  ASSERT_NEAR(base_curvature, 1.0, 1.0e-12);
  const double arc_length = std::abs(
      2.0 * std::atan2(target.y(), target.x()) / base_curvature);
  const double quantized_curvature =
      control.min_yaw_rate / control.min_forward_speed;
  const double quantized_turn = quantized_curvature * arc_length;
  const Eigen::Vector3d expected_extreme(
      std::sin(quantized_turn) / quantized_curvature,
      (1.0 - std::cos(quantized_turn)) / quantized_curvature,
      0.0);
  EXPECT_TRUE(containsPose(
      samples, expected_extreme, quantized_turn, 1.0e-9));
}

TEST(PurePursuitRecoverySweep, DefaultRecoveryReachStaysWithinBoundedBudget)
{
  // 最大允许横偏 0.40m 与 0.45m 沿轨迹前视叠加时，世界系目标距离可达
  // 约 0.85m。覆盖最苛刻的“几乎在正侧方”以及 TRACK 滞回边界。
  for (double y : {-0.84, 0.84})
  {
    for (double heading_error : {-0.45, 0.0, 0.45})
    {
      RecoverySweepControlLimits control;
      control.heading_error = heading_error;
      const auto samples = samplePurePursuitRecoverySweep(
          Eigen::Vector3d::Zero(),
          0.0,
          Eigen::Vector3d(0.02, y, 0.0),
          0.05,
          control);
      EXPECT_FALSE(samples.empty())
          << "y=" << y << ", heading_error=" << heading_error;
      EXPECT_LE(samples.size(), kRecoverySweepMaxPoseSamples);
    }
  }
}

TEST(PurePursuitRecoverySweep, OversizedInputFailsClosedWithinSampleBudget)
{
  const auto samples = samplePurePursuitRecoverySweep(
      Eigen::Vector3d::Zero(),
      0.0,
      Eigen::Vector3d(1.0e6, 1.0e6, 0.0),
      0.02);
  EXPECT_TRUE(samples.empty());
}

TEST(PurePursuitRecoverySweep, ExcessiveRibbonYawProductFailsClosed)
{
  // 单个维度都未超过 4096，但“纵向×扫掠带×yaw”超过总位姿预算。
  const auto samples = samplePurePursuitRecoverySweep(
      Eigen::Vector3d::Zero(),
      0.0,
      Eigen::Vector3d(0.02, 0.84, 0.0),
      0.01);
  EXPECT_TRUE(samples.empty());
}

}  // namespace
}  // namespace scan_planner
