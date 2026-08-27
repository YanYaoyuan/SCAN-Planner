// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file planning_context.h @brief Immutable identity and deadline snapshot for one plan. */

#ifndef PLAN_MANAGE__PLANNING_CONTEXT_H_
#define PLAN_MANAGE__PLANNING_CONTEXT_H_

#include <plan_manage/plan_deadline.h>

#include <cstdint>
#include <string>
#include <utility>

namespace scan_planner
{

/** @brief Monotonic revisions that make a candidate stale when they change. */
struct PlanningInputRevision
{
  std::uint64_t task{0};
  std::uint64_t map{0};
  std::uint64_t config{0};
};

inline bool operator==(
    const PlanningInputRevision &lhs,
    const PlanningInputRevision &rhs) noexcept
{
  return lhs.task == rhs.task && lhs.map == rhs.map &&
         lhs.config == rhs.config;
}

inline bool operator!=(
    const PlanningInputRevision &lhs,
    const PlanningInputRevision &rhs) noexcept
{
  return !(lhs == rhs);
}

/**
 * @brief Immutable inputs shared by every stage of one candidate plan.
 *
 * The numeric task revision is authoritative for stale-candidate rejection.
 * Mission and route strings are diagnostic identities and may be empty for the
 * legacy `/initial_path` entry point.
 */
class PlanningContext final
{
public:
  PlanningContext(
      PlanningInputRevision revisions,
      PlanDeadline deadline,
      std::string mission_id = {},
      std::string route_id = {})
  : revisions_(revisions),
    deadline_(deadline),
    mission_id_(std::move(mission_id)),
    route_id_(std::move(route_id))
  {
  }

  const PlanningInputRevision &revisions() const noexcept
  {
    return revisions_;
  }

  const PlanDeadline &deadline() const noexcept
  {
    return deadline_;
  }

  const std::string &missionId() const noexcept
  {
    return mission_id_;
  }

  const std::string &routeId() const noexcept
  {
    return route_id_;
  }

private:
  PlanningInputRevision revisions_;
  PlanDeadline deadline_;
  std::string mission_id_;
  std::string route_id_;
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__PLANNING_CONTEXT_H_
