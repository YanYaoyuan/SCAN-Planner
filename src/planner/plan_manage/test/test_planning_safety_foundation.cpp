// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_planning_safety_foundation.cpp @brief Planning deadline and atomic commit tests. */

#include <gtest/gtest.h>

#include <plan_manage/candidate_plan.h>

#include <chrono>
#include <string>

namespace scan_planner
{
namespace
{

using namespace std::chrono_literals;

TEST(PlanDeadline, UsesAbsoluteSteadyClockBoundary)
{
  const PlanDeadline::TimePoint start{};
  const PlanDeadline deadline(start, 100ms);

  EXPECT_EQ(deadline.startTime(), start);
  EXPECT_EQ(deadline.budget(), 100ms);
  EXPECT_EQ(deadline.expirationTime(), start + 100ms);
  EXPECT_FALSE(deadline.expired(start + 99ms));
  EXPECT_TRUE(deadline.expired(start + 100ms));
  EXPECT_EQ(deadline.remaining(start + 40ms), 60ms);
  EXPECT_EQ(deadline.remaining(start + 101ms), PlanDeadline::Duration::zero());
  EXPECT_EQ(deadline.elapsed(start - 1ms), PlanDeadline::Duration::zero());
}

TEST(PlanDeadline, NonPositiveBudgetFailsClosed)
{
  const PlanDeadline::TimePoint start{};
  EXPECT_TRUE(PlanDeadline(start, PlanDeadline::Duration::zero()).expired(start));
  EXPECT_TRUE(PlanDeadline(start, -1ms).expired(start));
}

TEST(PlanDeadline, SaturatesInsteadOfOverflowing)
{
  const auto start = PlanDeadline::TimePoint::max() - 5ms;
  const PlanDeadline deadline(start, 10ms);
  EXPECT_EQ(deadline.expirationTime(), PlanDeadline::TimePoint::max());
  EXPECT_EQ(deadline.budget(), 5ms);
}

TEST(PlanningContext, PreservesImmutableIdentityAndRevisionSnapshot)
{
  const PlanDeadline::TimePoint start{};
  const PlanningInputRevision revisions{4, 8, 15};
  const PlanningContext context(
      revisions, PlanDeadline(start, 100ms), "mission-a", "route-b");

  EXPECT_EQ(context.revisions(), revisions);
  EXPECT_EQ(context.missionId(), "mission-a");
  EXPECT_EQ(context.routeId(), "route-b");
  EXPECT_FALSE(context.deadline().expired(start + 99ms));
}

TEST(CandidatePlan, CommitsExactlyOnceWhenCurrentAndValidated)
{
  const PlanDeadline::TimePoint start{};
  const PlanningInputRevision revisions{1, 2, 3};
  CandidatePlan<std::string> candidate(
      PlanningContext(revisions, PlanDeadline(start, 100ms)), "candidate");
  std::string active = "old";
  const auto authority = [&active, &revisions](
      const PlanningInputRevision &planned, std::string &&payload) {
      if (const auto mismatch = candidateRevisionMismatch(planned, revisions))
        return *mismatch;
      active = std::move(payload);
      return CandidateCommitResult::kCommitted;
    };

  ASSERT_NE(candidate.payload(), nullptr);
  EXPECT_EQ(*candidate.payload(), "candidate");

  const auto result = candidate.tryCommit(
      true, authority, start + 50ms);

  EXPECT_EQ(result, CandidateCommitResult::kCommitted);
  EXPECT_EQ(active, "candidate");
  EXPECT_TRUE(candidate.consumed());
  EXPECT_EQ(
      candidate.tryCommit(
          true, authority, start + 60ms),
      CandidateCommitResult::kAlreadyConsumed);
}

TEST(CandidatePlan, ValidationAndDeadlineRejectionsPreserveCandidate)
{
  const PlanDeadline::TimePoint start{};
  const PlanningInputRevision planned{10, 20, 30};
  int commit_count = 0;
  const auto authority = [&commit_count](const PlanningInputRevision &, int &&) {
      ++commit_count;
      return CandidateCommitResult::kCommitted;
    };

  CandidatePlan<int> invalid(
      PlanningContext(planned, PlanDeadline(start, 100ms)), 1);
  EXPECT_EQ(
      invalid.tryCommit(false, authority, start + 1ms),
      CandidateCommitResult::kValidationFailed);
  EXPECT_FALSE(invalid.consumed());

  CandidatePlan<int> expired(
      PlanningContext(planned, PlanDeadline(start, 100ms)), 1);
  EXPECT_EQ(
      expired.tryCommit(true, authority, start + 100ms),
      CandidateCommitResult::kDeadlineExceeded);

  EXPECT_EQ(commit_count, 0);
  EXPECT_FALSE(expired.consumed());
}

TEST(CandidatePlan, RevisionAuthorityRejectsAndConsumesStaleAttempt)
{
  const PlanDeadline::TimePoint start{};
  const PlanningInputRevision planned{10, 20, 30};
  PlanningInputRevision current = planned;
  int commit_count = 0;
  const auto authority = [&commit_count, &current](
      const PlanningInputRevision &candidate_revisions, int &&) {
      if (const auto mismatch =
              candidateRevisionMismatch(candidate_revisions, current))
        return *mismatch;
      ++commit_count;
      return CandidateCommitResult::kCommitted;
    };

  current = {11, 20, 30};

  CandidatePlan<int> task_changed(
      PlanningContext(planned, PlanDeadline(start, 100ms)), 1);
  EXPECT_EQ(
      task_changed.tryCommit(true, authority, start + 1ms),
      CandidateCommitResult::kTaskChanged);

  current = {10, 21, 30};

  CandidatePlan<int> map_changed(
      PlanningContext(planned, PlanDeadline(start, 100ms)), 1);
  EXPECT_EQ(
      map_changed.tryCommit(true, authority, start + 1ms),
      CandidateCommitResult::kMapChanged);

  current = {10, 20, 31};

  CandidatePlan<int> config_changed(
      PlanningContext(planned, PlanDeadline(start, 100ms)), 1);
  EXPECT_EQ(
      config_changed.tryCommit(true, authority, start + 1ms),
      CandidateCommitResult::kConfigChanged);

  EXPECT_EQ(commit_count, 0);
  EXPECT_TRUE(task_changed.consumed());
  EXPECT_TRUE(map_changed.consumed());
  EXPECT_TRUE(config_changed.consumed());
}

}  // namespace
}  // namespace scan_planner
