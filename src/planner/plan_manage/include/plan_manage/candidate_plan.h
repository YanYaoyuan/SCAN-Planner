// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file candidate_plan.h @brief One-shot stale-safe candidate commit boundary. */

#ifndef PLAN_MANAGE__CANDIDATE_PLAN_H_
#define PLAN_MANAGE__CANDIDATE_PLAN_H_

#include <plan_manage/planning_context.h>

#include <optional>
#include <utility>

namespace scan_planner
{

/** @brief Result of checking and committing a candidate plan. */
enum class CandidateCommitResult
{
  kCommitted,
  kAlreadyConsumed,
  kValidationFailed,
  kDeadlineExceeded,
  kTaskChanged,
  kMapChanged,
  kConfigChanged,
};

/** @brief Stable diagnostic name for a commit result. */
inline const char *candidateCommitResultName(CandidateCommitResult result) noexcept
{
  switch (result)
  {
    case CandidateCommitResult::kCommitted:
      return "committed";
    case CandidateCommitResult::kAlreadyConsumed:
      return "already_consumed";
    case CandidateCommitResult::kValidationFailed:
      return "validation_failed";
    case CandidateCommitResult::kDeadlineExceeded:
      return "deadline_exceeded";
    case CandidateCommitResult::kTaskChanged:
      return "task_changed";
    case CandidateCommitResult::kMapChanged:
      return "map_changed";
    case CandidateCommitResult::kConfigChanged:
      return "config_changed";
  }
  return "unknown";
}

/** @brief Returns the first stale-input reason, or no value when revisions match. */
inline std::optional<CandidateCommitResult> candidateRevisionMismatch(
    const PlanningInputRevision &planned,
    const PlanningInputRevision &current) noexcept
{
  if (planned.task != current.task)
    return CandidateCommitResult::kTaskChanged;
  if (planned.map != current.map)
    return CandidateCommitResult::kMapChanged;
  if (planned.config != current.config)
    return CandidateCommitResult::kConfigChanged;
  return std::nullopt;
}

/**
 * @brief Owns an uncommitted payload and exposes exactly one commit operation.
 *
 * Candidate construction and validation do not touch the active trajectory.
 * The commit authority is invoked only after validation and deadline checks.
 * It must compare the supplied planned revisions with authoritative live state
 * and install the payload under the same owner lock or serialized executor
 * boundary. This closes the check/install race that would exist if callers
 * passed an earlier copy of the live revisions.
 *
 * The type is intentionally neither copyable nor movable so the same candidate
 * cannot accidentally be committed by two owners.
 */
template<typename Payload>
class CandidatePlan final
{
public:
  CandidatePlan(PlanningContext context, Payload payload)
  : context_(std::move(context)), payload_(std::move(payload))
  {
  }

  CandidatePlan(const CandidatePlan &) = delete;
  CandidatePlan &operator=(const CandidatePlan &) = delete;
  CandidatePlan(CandidatePlan &&) = delete;
  CandidatePlan &operator=(CandidatePlan &&) = delete;

  const PlanningContext &context() const noexcept
  {
    return context_;
  }

  bool consumed() const noexcept
  {
    return !payload_.has_value();
  }

  /** @brief Returns the uncommitted payload for read-only hard validation. */
  const Payload *payload() const noexcept
  {
    return payload_ ? &(*payload_) : nullptr;
  }

  /**
   * @brief Commits the payload if it is safe and current.
   * @param validation_passed True only after all hard validators passed.
   * @param authority Callback that atomically checks live revisions and installs
   * the payload. It must return a CandidateCommitResult and must not throw.
   * @param now Current steady-clock time, injectable for deterministic tests.
   */
  template<typename CommitAuthority>
  CandidateCommitResult tryCommit(
      bool validation_passed,
      CommitAuthority &&authority,
      PlanDeadline::TimePoint now = PlanDeadline::Clock::now())
  {
    if (!payload_)
      return CandidateCommitResult::kAlreadyConsumed;
    if (!validation_passed)
      return CandidateCommitResult::kValidationFailed;
    if (context_.deadline().expired(now))
      return CandidateCommitResult::kDeadlineExceeded;

    // Mark consumed before invoking user code so a re-entrant authority cannot
    // commit this candidate twice. Commit authorities must not throw.
    std::optional<Payload> payload = std::move(payload_);
    payload_.reset();
    return std::forward<CommitAuthority>(authority)(
        context_.revisions(), std::move(*payload));
  }

private:
  PlanningContext context_;
  std::optional<Payload> payload_;
};

}  // namespace scan_planner

#endif  // PLAN_MANAGE__CANDIDATE_PLAN_H_
