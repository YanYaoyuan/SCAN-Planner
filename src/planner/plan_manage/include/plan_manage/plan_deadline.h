// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file plan_deadline.h @brief Monotonic absolute deadline for one planning attempt. */

#ifndef PLAN_MANAGE__PLAN_DEADLINE_H_
#define PLAN_MANAGE__PLAN_DEADLINE_H_

#include <chrono>

namespace scan_planner
{

/**
 * @brief Immutable steady-clock deadline shared by every stage of one plan.
 *
 * A zero or negative budget is fail-closed and is already expired.  The class
 * deliberately has no "disabled" state so production callers cannot silently
 * turn a hard planning bound into diagnostic-only timing.
 */
class PlanDeadline final
{
public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  /** @brief Creates a deadline starting now. */
  explicit PlanDeadline(Duration budget)
  : PlanDeadline(Clock::now(), budget)
  {
  }

  /** @brief Creates a deterministic deadline from an explicit start time. */
  PlanDeadline(TimePoint start, Duration budget)
  : start_(start), budget_(computeBudget(start, budget)),
    deadline_(start + budget_)
  {
  }

  /** @brief Returns the start of this planning budget. */
  TimePoint startTime() const noexcept
  {
    return start_;
  }

  /** @brief Returns the absolute expiration time. */
  TimePoint expirationTime() const noexcept
  {
    return deadline_;
  }

  /** @brief Returns the effective non-negative budget. */
  Duration budget() const noexcept
  {
    return budget_;
  }

  /** @brief Returns true at or after the deadline. */
  bool expired(TimePoint now = Clock::now()) const noexcept
  {
    return now >= deadline_;
  }

  /** @brief Returns remaining time, saturated at zero after expiration. */
  Duration remaining(TimePoint now = Clock::now()) const noexcept
  {
    if (now <= start_)
      return budget_;
    return now >= deadline_ ? Duration::zero() : deadline_ - now;
  }

  /** @brief Returns elapsed time, saturated at zero before the start. */
  Duration elapsed(TimePoint now = Clock::now()) const noexcept
  {
    if (now <= start_)
      return Duration::zero();
    return now >= deadline_ ? budget_ : now - start_;
  }

private:
  static Duration computeBudget(TimePoint start, Duration budget) noexcept
  {
    if (budget <= Duration::zero())
      return Duration::zero();

    // Subtracting a positive duration from max() is representable.  Only
    // compute max()-start in the saturation branch, where the result is known
    // to be non-negative and smaller than budget.
    if (start > TimePoint::max() - budget)
      return TimePoint::max() - start;
    return budget;
  }

  TimePoint start_;
  Duration budget_;
  TimePoint deadline_;
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__PLAN_DEADLINE_H_
