// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file test_a_star_deadline.cpp @brief A-star deadline contract tests. */

#include <gtest/gtest.h>

#include <path_searching/dyn_a_star.h>

#include <chrono>

namespace
{

TEST(AStarDeadline, ExpiredAttemptFailsBeforeMapAccess)
{
  AStar search;
  search.setDeadline(std::chrono::steady_clock::now());

  EXPECT_TRUE(search.deadlineExceeded());
  EXPECT_EQ(
      search.AstarSearch(
          0.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()),
      ASTAR_RET::DEADLINE_EXCEEDED);
}

TEST(AStarDeadline, CanBeClearedForStandaloneUse)
{
  AStar search;
  search.setDeadline(std::chrono::steady_clock::now());
  search.clearDeadline();

  EXPECT_FALSE(search.deadlineExceeded());
  EXPECT_EQ(
      search.AstarSearch(
          0.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()),
      ASTAR_RET::INIT_ERR);
}

}  // namespace
