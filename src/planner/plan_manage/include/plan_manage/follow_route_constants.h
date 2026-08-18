// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file follow_route_constants.h @brief Local mirror of the FollowRoute.action contract constants. */

#ifndef _FOLLOW_ROUTE_CONSTANTS_H_
#define _FOLLOW_ROUTE_CONSTANTS_H_

#include <cstddef>
#include <cstdint>

namespace scan_planner
{
namespace follow_route
{

// Values mirror omni_robot_interfaces/action/FollowRoute.action. The Humble
// rosidl generator binds .action section constants to the ADJACENT generated
// type (feedback-section constants on Result, result-section constants on
// Feedback), so consumers mirror the constants locally instead of relying on
// the shifted generated names. The canonical values are pinned against the
// IDL source by the omni_robot_interfaces CI
// (ci/check_contract_constants.py).

// FollowRoute feedback state (feedback section).
// STATE_PAUSED is owned by the Mission Manager's MissionStatus; the planner
// never reports it (a paused mission keeps its planner goal EXECUTING with
// static progress while the gateway lease forces zero velocity).
constexpr std::uint8_t STATE_PLANNING = 1;
constexpr std::uint8_t STATE_EXECUTING = 2;
constexpr std::uint8_t STATE_PAUSED = 3;

// FollowRoute result reason_code (result section).
constexpr std::uint32_t REASON_OK = 0;
constexpr std::uint32_t REASON_USER_CANCELED = 1;
constexpr std::uint32_t REASON_ABORTED = 2;
constexpr std::uint32_t REASON_GOAL_REJECTED = 3;
constexpr std::uint32_t REASON_MAP_MISMATCH = 4;
constexpr std::uint32_t REASON_LOCALIZATION_LOST = 5;
constexpr std::uint32_t REASON_HEARTBEAT_LOST = 6;
constexpr std::uint32_t REASON_TIMEOUT = 7;

// Goal speed_scale validation: 0 means "planner default", otherwise
// SPEED_SCALE_MIN..SPEED_SCALE_MAX.
constexpr double SPEED_SCALE_MIN = 0.05;
constexpr double SPEED_SCALE_MAX = 1.0;

// mission_id dedup key per planner epoch; bounded so a long-lived node does
// not grow the set unboundedly.
constexpr std::size_t MAX_FINISHED_MISSION_IDS = 4096;

}  // namespace follow_route
}  // namespace scan_planner

#endif  // _FOLLOW_ROUTE_CONSTANTS_H_