// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file path_command_governor.h @brief Pure Pursuit 线角速度联合发布治理。 */

#ifndef PLAN_MANAGE__PATH_COMMAND_GOVERNOR_H_
#define PLAN_MANAGE__PATH_COMMAND_GOVERNOR_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "plan_manage/slew_rate_limiter.h"
#include "plan_manage/yaw_command_filter.h"

namespace scan_planner
{

/**
 * @brief 把几何控制目标变成可以直接发给真机桥接层的离散速度命令。
 *
 * ZSL-1W 当前桥接配置只执行 0 或绝对值不小于 0.05m/s 的前向速度，
 * 角速度也只能是 0 或绝对值不小于 0.10rad/s。若分别处理两个通道，
 * 急弯启动时会先转不走或先走不转。本类因此同时管理：
 *
 * 1. 连续线加减速度限制；
 * 2. SDK 最小非零线速度；
 * 3. 急弯启动前的 yaw 滤波预热；
 * 4. 左右换向时旧方向角速度的抑制；
 * 5. 移动命令的最终角速度严格使用本周期实际发布的 vx；若能力反推线速
 *    低于 SDK 最小值，则显式进入零平移的转向准备态，避免几何死锁。
 *
 * 安全急停不应经过本类的正常减速，调用者必须直接 reset() 并发布零。
 */
class PathCommandGovernor
{
public:
  struct Config
  {
    double max_forward_speed{0.30};
    double max_yaw_rate{0.50};
    double min_nonzero_linear_speed{0.05};
    double yaw_sync_threshold{0.10};
    double yaw_direction_deadband{0.025};
    double launch_yaw_readiness_ratio{0.85};
    SlewRateLimiter::Config linear_limiter{0.35, 0.60};
    YawCommandFilter::Config yaw_filter{
        0.025, 0.12, 1.0, 0.12, 0.06, 0.03, 0.10};
  };

  /** @brief PathTrackingController 给出的、尚未做发布时序治理的目标。 */
  struct Target
  {
    double linear_x{0.0};
    double hard_speed_limit{0.0};
    double curvature{0.0};
    double heading_yaw{0.0};
    bool rotate_in_place{false};
    bool allow_minimum_speed_creep{false};
  };

  struct Command
  {
    double linear_x{0.0};
    double angular_z{0.0};
    double raw_yaw_rate{0.0};
    bool waiting_for_synchronized_launch{false};
    bool suppressed_wrong_direction_yaw{false};
    bool settled{true};
    bool valid{false};
  };

  PathCommandGovernor()
  {
    setConfig(Config{});
  }

  explicit PathCommandGovernor(const Config &config)
  {
    setConfig(config);
  }

  void setConfig(const Config &config)
  {
    const bool valid =
        std::isfinite(config.max_forward_speed) &&
        config.max_forward_speed > 0.0 &&
        std::isfinite(config.max_yaw_rate) && config.max_yaw_rate > 0.0 &&
        std::isfinite(config.min_nonzero_linear_speed) &&
        config.min_nonzero_linear_speed > 0.0 &&
        config.min_nonzero_linear_speed <= config.max_forward_speed &&
        std::isfinite(config.yaw_sync_threshold) &&
        config.yaw_sync_threshold >= 0.0 &&
        config.yaw_sync_threshold <= config.max_yaw_rate &&
        std::isfinite(config.yaw_filter.start_threshold) &&
        config.yaw_filter.start_threshold < config.yaw_sync_threshold &&
        std::isfinite(config.yaw_direction_deadband) &&
        config.yaw_direction_deadband >= 0.0 &&
        config.yaw_direction_deadband <= config.max_yaw_rate &&
        std::isfinite(config.yaw_filter.min_nonzero_output) &&
        config.yaw_filter.min_nonzero_output <= config.max_yaw_rate &&
        std::isfinite(config.launch_yaw_readiness_ratio) &&
        config.launch_yaw_readiness_ratio > 0.0 &&
        config.launch_yaw_readiness_ratio < 1.0;
    if (!valid)
      throw std::invalid_argument("invalid path-command governor configuration");

    // 子组件会继续校验各自的细分参数。
    linear_limiter_.setConfig(config.linear_limiter);
    yaw_filter_.setConfig(config.yaw_filter);
    config_ = config;
    reset();
  }

  /** @brief 安全停车或切换轨迹时立即清除所有连续状态。 */
  void reset()
  {
    linear_limiter_.reset();
    yaw_filter_.reset();
    linear_output_active_ = false;
    launch_committed_ = false;
    linear_stop_committed_ = false;
    moving_raw_yaw_sign_ = 0;
    opposing_zero_crossing_active_ = false;
    opposing_yaw_ready_cycles_ = 0;
    last_command_ = Command{};
    last_command_.valid = true;
  }

  /**
   * @brief 推进一步线角联合治理状态机。
   *
   * @note yaw 预热周期始终发布零；开始运动后，滤波器每周期只用
   *       `published_vx * curvature + heading_yaw` 更新一次。
   */
  Command update(const Target &target, double dt)
  {
    Command command;
    if (!finiteTarget(target) || !std::isfinite(dt) || dt <= 0.0)
    {
      // 调度停顿、ROS 时间回跳或非有限目标都属于控制连续性断裂。
      // 本周期返回 invalid/零，同时清空内部速度，禁止下一周期从旧巡航值
      // 直接恢复，形成 0 -> 0.20m/s 一类危险阶跃。
      reset();
      return command;
    }
    command.valid = true;

    const double hard_speed_limit = std::clamp(
        target.hard_speed_limit, 0.0, config_.max_forward_speed);
    const double raw_requested_speed = std::clamp(
        target.linear_x, 0.0,
        std::min(config_.max_forward_speed, hard_speed_limit));
    // 终点制动速度可能落在 (0, SDK最小线速度) 内。若治理器刚 reset，
    // 直接使用这个目标会永远达不到 0.05m/s 的发布门槛，机器人也就永远
    // 进不了 finish_distance。仅当几何控制器明确标记“终点制动是当前
    // 限制来源”时把它量化为最小爬行速度，避免覆盖航向连续减速。
    const bool endpoint_minimum_creep =
        target.allow_minimum_speed_creep && raw_requested_speed > kEpsilon &&
        raw_requested_speed + kEpsilon < config_.min_nonzero_linear_speed &&
        hard_speed_limit + kEpsilon >= config_.min_nonzero_linear_speed;
    const double requested_speed = endpoint_minimum_creep
        ? config_.min_nonzero_linear_speed
        : raw_requested_speed;

    if (target.rotate_in_place)
    {
      // ALIGN 必须立即消除平移；其角速度仍按真实 vx=0 计算。
      clearLinearState();
      return finishYawCommand(
          command, 0.0, target.heading_yaw, dt, false);
    }

    double continuous_speed = linear_limiter_.update(requested_speed, dt);
    if (continuous_speed > hard_speed_limit)
    {
      // 曲率/角速度能力属于硬安全上限，可以绕过普通减速度限制。
      continuous_speed = hard_speed_limit;
      linear_limiter_.reset(continuous_speed);
    }

    if (hard_speed_limit + kEpsilon < config_.min_nonzero_linear_speed)
    {
      // 小于桥接死区时不存在可执行的非零线速度。若这是急曲率造成的，
      // 不能简单发 0/0，否则“近终点+大横偏+heading≈0”会永久死锁。
      // 用最小可执行线速对应的几何 yaw 作为原地转向准备；机体朝前视点
      // 转正后曲率会下降，hard limit 恢复到 0.05 以上才能重新同步起步。
      clearLinearState();
      double preparation_yaw = clampYaw(
          config_.min_nonzero_linear_speed * target.curvature +
          target.heading_yaw);
      if (std::abs(preparation_yaw) + kEpsilon <
          config_.yaw_filter.start_threshold)
      {
        // 防御未来参数组合：若曲率与航向项恰好抵消，选择幅值更大的主项
        // 做原地准备，不能再次退化为 hard<min 条件下的永久 0/0。
        const double curvature_yaw = clampYaw(
            config_.min_nonzero_linear_speed * target.curvature);
        const double heading_yaw = clampYaw(target.heading_yaw);
        preparation_yaw = std::abs(curvature_yaw) >= std::abs(heading_yaw)
            ? curvature_yaw
            : heading_yaw;
      }
      return finishYawCommand(
          command, 0.0, preparation_yaw, dt, true);
    }

    if (linear_output_active_)
    {
      if (linear_stop_committed_)
      {
        // 上一周期已经发布了 SDK 最小线速度；本周期完成不可避免的
        // 0.05->0 量化跳变，避免从 0.058 等任意值直接掉到零。
        clearLinearState();
        return finishYawCommand(
            command, 0.0, target.heading_yaw, dt, false);
      }
      if (continuous_speed + kEpsilon < config_.min_nonzero_linear_speed)
      {
        // 连续状态刚跨入 SDK 死区，先明确发布一周期最小非零速度，
        // 下一周期再归零；这也是底盘接口允许的最小停车阶跃。
        continuous_speed = config_.min_nonzero_linear_speed;
        linear_limiter_.reset(continuous_speed);
        linear_stop_committed_ = true;
      }

      continuous_speed = limitSpeedByAvailableYaw(
          continuous_speed, target.curvature, target.heading_yaw);
      command.linear_x = continuous_speed;
      const double actual_raw_yaw = clampYaw(
          command.linear_x * target.curvature + target.heading_yaw);
      return finishMovingCommand(command, actual_raw_yaw, dt);
    }

    if (requested_speed + kEpsilon < config_.min_nonzero_linear_speed)
    {
      // 尚未开始走且目标已进入线速度死区。正常终点减速会走到这里，
      // yaw 仍以真实 vx=0 的目标平滑收敛。
      launch_committed_ = false;
      return finishYawCommand(
          command, 0.0, target.heading_yaw, dt, false);
    }

    if (continuous_speed >= config_.min_nonzero_linear_speed)
    {
      // 等待期间不允许连续限速器偷偷积累到巡航速度，否则放行时会跳变。
      continuous_speed = config_.min_nonzero_linear_speed;
      linear_limiter_.reset(continuous_speed);
    }

    const double launch_speed = config_.min_nonzero_linear_speed;
    const double launch_raw_yaw = clampYaw(
        launch_speed * target.curvature + target.heading_yaw);
    const double full_raw_yaw = clampYaw(
        requested_speed * target.curvature + target.heading_yaw);
    const bool same_launch_direction =
        sameNonzeroDirection(launch_raw_yaw, full_raw_yaw);
    const bool synchronized_launch =
        std::abs(launch_raw_yaw) >= config_.yaw_sync_threshold ||
        (same_launch_direction &&
         std::abs(full_raw_yaw) >= config_.yaw_sync_threshold);

    if (launch_committed_ &&
        continuous_speed + kEpsilon >= config_.min_nonzero_linear_speed)
    {
      // 上一周期只完成“预热就绪”状态转换；本周期才使用真实启动 vx
      // 更新 yaw 并同时发布，避免同一周期把滤波器推进两次。
      launch_committed_ = false;
      command.linear_x = launch_speed;
      // 即使本周期入口处已经把等待状态钳在最小速度，这里仍显式同步
      // limiter，保证后续重构不会让首个增量从 0.05 跳到 0.064。
      linear_limiter_.reset(launch_speed);
      const double actual_raw_yaw = clampYaw(
          command.linear_x * target.curvature + target.heading_yaw);
      const double filtered_yaw = yaw_filter_.update(actual_raw_yaw, dt);
      if (synchronized_launch &&
          !yawReadyForLaunch(filtered_yaw, actual_raw_yaw))
      {
        command.linear_x = 0.0;
        command.waiting_for_synchronized_launch = true;
        command.settled = false;
        last_command_ = command;
        return command;
      }

      linear_output_active_ = true;
      linear_stop_committed_ = false;
      if (target.curvature * target.heading_yaw < 0.0 &&
          full_raw_yaw * target.curvature > 0.0 &&
          std::abs(actual_raw_yaw) < config_.yaw_sync_threshold &&
          std::abs(full_raw_yaw) >= config_.yaw_sync_threshold)
      {
        // 零点恰好位于 0 与 SDK 最小线速之间时，首个可执行 raw 可能仍在
        // deadband 内，因而看不到一次显式符号翻转。预先进入稳定观察态，
        // 防止刚出现一次量化 yaw 就立刻冲入 strong 区并被迫停车。
        opposing_zero_crossing_active_ = true;
        opposing_yaw_ready_cycles_ = 0;
      }
      updateMovingYawTransition(actual_raw_yaw, filtered_yaw);
      command.raw_yaw_rate = actual_raw_yaw;
      command.angular_z = safeYawOutput(
          actual_raw_yaw, filtered_yaw,
          command.suppressed_wrong_direction_yaw);
      command.settled = false;
      last_command_ = command;
      return command;
    }

    // 预热只用于判断急弯能否和最小线速度同步启动，此周期仍发布 0/0。
    // 只有启动速度与目标速度下的 yaw 同向时，才允许用较大的完整目标预热；
    // 这样 S 弯中曲率前馈与航向反馈反号时不会预热到错误方向。
    const double preview_yaw =
        same_launch_direction &&
        std::abs(full_raw_yaw) > std::abs(launch_raw_yaw)
        ? full_raw_yaw
        : launch_raw_yaw;
    const double preview_output = yaw_filter_.update(preview_yaw, dt);
    const bool line_ready =
        continuous_speed + kEpsilon >= config_.min_nonzero_linear_speed;
    const bool yaw_ready =
        !synchronized_launch ||
        yawReadyForLaunch(preview_output, launch_raw_yaw);
    launch_committed_ = line_ready && yaw_ready;
    command.waiting_for_synchronized_launch =
        synchronized_launch && !launch_committed_;
    command.settled = false;
    last_command_ = command;
    return command;
  }

  const Command &lastCommand() const { return last_command_; }

private:
  static constexpr double kEpsilon = 1.0e-9;

  static bool finiteTarget(const Target &target)
  {
    return std::isfinite(target.linear_x) &&
        std::isfinite(target.hard_speed_limit) &&
        std::isfinite(target.curvature) &&
        std::isfinite(target.heading_yaw);
  }

  static int signWithDeadband(double value, double deadband)
  {
    if (std::abs(value) <= deadband)
      return 0;
    return value > 0.0 ? 1 : -1;
  }

  bool sameNonzeroDirection(double lhs, double rhs) const
  {
    const int lhs_sign = signWithDeadband(
        lhs, config_.yaw_direction_deadband);
    const int rhs_sign = signWithDeadband(
        rhs, config_.yaw_direction_deadband);
    return lhs_sign != 0 && lhs_sign == rhs_sign;
  }

  double clampYaw(double yaw_rate) const
  {
    return std::clamp(
        yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate);
  }

  double limitSpeedByAvailableYaw(
      double candidate_speed,
      double curvature,
      double heading_yaw)
  {
    if (std::abs(curvature) <= kEpsilon)
      return candidate_speed;

    const double candidate_raw_yaw = clampYaw(
        candidate_speed * curvature + heading_yaw);
    const bool opposing_terms = curvature * heading_yaw < 0.0;
    const int candidate_sign = signWithDeadband(
        candidate_raw_yaw, config_.yaw_direction_deadband);
    if (opposing_terms && moving_raw_yaw_sign_ != 0 &&
        candidate_sign != 0 && candidate_sign != moving_raw_yaw_sign_)
    {
      // 记录真实 raw 在运动中跨过零点。后续即便量化后的 yaw 已短暂出现
      // 新方向，也要等它连续稳定两周期再放开强 yaw 区，避免滤波内部仍在
      // stop_threshold 附近而下一周期重新归零。
      moving_raw_yaw_sign_ = candidate_sign;
      opposing_zero_crossing_active_ = true;
      opposing_yaw_ready_cycles_ = 0;
    }
    if (std::abs(candidate_raw_yaw) < config_.yaw_sync_threshold ||
        candidate_raw_yaw * curvature <= 0.0)
    {
      // raw 与曲率反号说明当前仍在抵消区：继续增加 vx 只会让 |raw| 下降，
      // 不会让线速度跑到角速度能力前面。越过零点后两者同号，再进入下方
      // 的“随 vx 增长”分支治理。
      return candidate_speed;
    }

    const double available_yaw = std::abs(yaw_filter_.output());
    double supported_raw_yaw = 0.0;
    const bool hold_zero_crossing_corridor =
        opposing_terms && opposing_zero_crossing_active_ &&
        opposing_yaw_ready_cycles_ < kOpposingYawReadyCycles;
    if (hold_zero_crossing_corridor)
    {
      supported_raw_yaw = zeroCrossingCorridorYaw();
    }
    else if (available_yaw <= kEpsilon ||
        !sameNonzeroDirection(candidate_raw_yaw, yaw_filter_.output()))
    {
      if (!opposing_terms)
      {
        // 普通强换向没有“前馈/反馈抵消过零”的几何连续区，仍应停车后
        // 重新同步；不能把所有强换向都降级成带平移的弱 yaw 换向。
        return candidate_speed;
      }
      // 曲率前馈与航向反馈反号时，raw=v*kappa+w_heading 会在加速途中
      // 过零。若滤波器还保留零点另一侧的旧方向，直接继续加速会在
      // |raw| 达到强同步阈值后触发停车；reset 后又从旧方向起步，最终
      // 形成周期性的“走几步—停车”。先把 raw 限在 start 与 sync 之间的
      // 弱转向走廊，让 yaw 在不发布旧方向命令的前提下完成换向。
      if (config_.yaw_filter.start_threshold + kEpsilon >=
          config_.yaw_sync_threshold)
      {
        // 没有可用的弱转向走廊时保留原有强转向停车语义；外层会清除
        // 平移并重新同步启动，不能为了不断行而放宽安全条件。
        return candidate_speed;
      }
      opposing_zero_crossing_active_ = true;
      opposing_yaw_ready_cycles_ = 0;
      supported_raw_yaw = zeroCrossingCorridorYaw();
    }
    else
    {
      // yaw 已是本周期需要的方向时，按当前可发布角速度反推本周期可支持
      // 的 raw，线速度只能随 yaw 滤波状态一起向上爬升。
      supported_raw_yaw =
          available_yaw / config_.launch_yaw_readiness_ratio;
    }

    // 当前分支已位于 raw 过零后的曲率方向一侧，因此
    // |v*kappa+w_heading| = v*|kappa| + sign(kappa)*w_heading。
    // 这个带符号公式同时覆盖同号相加和反号抵消，不能再减 |heading|。
    const double supported_speed_unclamped =
        (supported_raw_yaw - std::copysign(1.0, curvature) * heading_yaw) /
        std::abs(curvature);
    if (supported_speed_unclamped + kEpsilon <
        config_.min_nonzero_linear_speed)
    {
      // SDK 最小线速度下已经超出该 yaw 状态可支持的范围，不能发布一个
      // 伪造的子死区线速度；让 finishMovingCommand() 走强转向停车/重启。
      return candidate_speed;
    }
    const double supported_speed = std::max(
        config_.min_nonzero_linear_speed, supported_speed_unclamped);
    if (candidate_speed <= supported_speed)
      return candidate_speed;

    const double limited_speed = std::min(candidate_speed, supported_speed);
    linear_limiter_.reset(limited_speed);
    return limited_speed;
  }

  bool yawReadyForLaunch(double filtered_yaw, double actual_raw_yaw) const
  {
    const int desired_sign = signWithDeadband(
        actual_raw_yaw, config_.yaw_direction_deadband);
    const int filtered_sign = signWithDeadband(filtered_yaw, kEpsilon);
    if (desired_sign == 0)
      return true;
    if (desired_sign != filtered_sign)
      return false;
    return std::abs(filtered_yaw) + kEpsilon >=
        config_.launch_yaw_readiness_ratio * std::abs(actual_raw_yaw);
  }

  double safeYawOutput(
      double actual_raw_yaw,
      double filtered_yaw,
      bool &suppressed_wrong_direction) const
  {
    const int desired_sign = signWithDeadband(
        actual_raw_yaw, config_.yaw_direction_deadband);
    const int filtered_sign = signWithDeadband(filtered_yaw, kEpsilon);
    if (desired_sign == 0)
    {
      // 真实 vx 和航向反馈都不需要转弯时，不能把 preview 或上一弯道的
      // 滤波残量继续发给底盘。滤波内部仍会正常衰减，用于 settled 判定。
      return 0.0;
    }
    if (filtered_sign != 0 && desired_sign != filtered_sign)
    {
      // 即使新目标很弱，也绝不允许带着上一方向的 yaw 继续行走。
      suppressed_wrong_direction = true;
      return 0.0;
    }
    if (filtered_sign == 0)
      return 0.0;

    const double raw_magnitude = std::abs(actual_raw_yaw);
    const double minimum_yaw = config_.yaw_filter.min_nonzero_output;
    if (raw_magnitude + kEpsilon < minimum_yaw)
    {
      // SDK 无法表达 (0, min_yaw)；只有滤波器已经决定开始转向时才
      // 量化成最小非零命令，不能发布 preview 遗留的更大角速度。
      return std::copysign(minimum_yaw, actual_raw_yaw);
    }

    // raw 已处在 SDK 可执行区时，滤波只允许减缓响应，绝不能超过按本周期
    // 最终 vx 计算出的 w_actual，从而避免启动阶段瞬时过曲率。
    return std::copysign(
        std::min(std::abs(filtered_yaw), raw_magnitude), actual_raw_yaw);
  }

  Command finishMovingCommand(
      Command command,
      double actual_raw_yaw,
      double dt)
  {
    command.raw_yaw_rate = actual_raw_yaw;
    const double filtered_yaw = yaw_filter_.update(actual_raw_yaw, dt);
    updateMovingYawTransition(actual_raw_yaw, filtered_yaw);
    const bool wrong_direction =
        signWithDeadband(actual_raw_yaw, config_.yaw_direction_deadband) != 0 &&
        signWithDeadband(filtered_yaw, kEpsilon) != 0 &&
        !sameNonzeroDirection(actual_raw_yaw, filtered_yaw);
    const bool strong_yaw_not_ready =
        std::abs(actual_raw_yaw) >= config_.yaw_sync_threshold &&
        !yawReadyForLaunch(filtered_yaw, actual_raw_yaw);

    command.angular_z = safeYawOutput(
        actual_raw_yaw, filtered_yaw,
        command.suppressed_wrong_direction_yaw);
    if ((wrong_direction || strong_yaw_not_ready) &&
        std::abs(actual_raw_yaw) >= config_.yaw_sync_threshold)
    {
      // 强换向或突然出现急弯时先停车。内部 yaw 状态已朝新目标推进，
      // 下一周期会从最小可执行线速度重新做同步启动。
      clearLinearState();
      command.linear_x = 0.0;
      command.angular_z = 0.0;
      command.waiting_for_synchronized_launch = true;
    }
    command.settled =
        command.linear_x == 0.0 && command.angular_z == 0.0 &&
        std::abs(linear_limiter_.output()) <= kEpsilon &&
        std::abs(yaw_filter_.output()) <= kEpsilon;
    last_command_ = command;
    return command;
  }

  Command finishYawCommand(
      Command command,
      double linear_x,
      double heading_yaw,
      double dt,
      bool waiting_for_launch)
  {
    command.linear_x = linear_x;
    command.raw_yaw_rate = clampYaw(heading_yaw);
    const double filtered_yaw = yaw_filter_.update(command.raw_yaw_rate, dt);
    command.angular_z = safeYawOutput(
        command.raw_yaw_rate, filtered_yaw,
        command.suppressed_wrong_direction_yaw);
    command.waiting_for_synchronized_launch = waiting_for_launch;
    command.settled =
        command.linear_x == 0.0 && command.angular_z == 0.0 &&
        std::abs(linear_limiter_.output()) <= kEpsilon &&
        std::abs(yaw_filter_.output()) <= kEpsilon;
    last_command_ = command;
    return command;
  }

  void clearLinearState()
  {
    linear_limiter_.reset();
    linear_output_active_ = false;
    launch_committed_ = false;
    linear_stop_committed_ = false;
    moving_raw_yaw_sign_ = 0;
    opposing_zero_crossing_active_ = false;
    opposing_yaw_ready_cycles_ = 0;
  }

  double zeroCrossingCorridorYaw() const
  {
    // 走廊必须低于 strong sync 阈值，同时尽量贴近它，避免仅为等待 yaw
    // 稳定就对线速度做明显的反向阶跃。默认参数下结果为 0.099rad/s。
    const double corridor_floor = std::max(
        config_.yaw_filter.start_threshold,
        0.98 * config_.yaw_sync_threshold);
    return 0.5 * (corridor_floor + config_.yaw_sync_threshold);
  }

  void updateMovingYawTransition(
      double actual_raw_yaw,
      double filtered_yaw)
  {
    const int actual_sign = signWithDeadband(
        actual_raw_yaw, config_.yaw_direction_deadband);
    if (actual_sign != 0)
      moving_raw_yaw_sign_ = actual_sign;

    if (!opposing_zero_crossing_active_ || actual_sign == 0)
      return;

    const int filtered_sign = signWithDeadband(filtered_yaw, kEpsilon);
    if (filtered_sign == actual_sign)
      ++opposing_yaw_ready_cycles_;
    else
      opposing_yaw_ready_cycles_ = 0;

    if (opposing_yaw_ready_cycles_ >= kOpposingYawReadyCycles)
      opposing_zero_crossing_active_ = false;
  }

  Config config_;
  SlewRateLimiter linear_limiter_;
  YawCommandFilter yaw_filter_;
  bool linear_output_active_{false};
  bool launch_committed_{false};
  bool linear_stop_committed_{false};
  static constexpr int kOpposingYawReadyCycles = 2;
  int moving_raw_yaw_sign_{0};
  bool opposing_zero_crossing_active_{false};
  int opposing_yaw_ready_cycles_{0};
  Command last_command_;
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__PATH_COMMAND_GOVERNOR_H_
