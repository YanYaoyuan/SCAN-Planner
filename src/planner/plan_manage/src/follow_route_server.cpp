// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file follow_route_server.cpp @brief FollowRoute action server for reference route missions. */

#include <plan_manage/follow_route_server.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace scan_planner
{
namespace
{

constexpr double kMonitorPeriodSec = 0.1;
constexpr double kCancelStopTimeoutSec = 15.0;
constexpr double kDefaultStuckTimeoutSec = 60.0;

// FSM_EXEC_STATE values (private enum of SCANReplanFSM, exposed as an int in
// FollowRouteView::exec_state).
constexpr int kStateInit = 0;
constexpr int kStateWaitTarget = 1;

}  // namespace

FollowRouteServer::FollowRouteServer(
    const rclcpp::Node::SharedPtr &node, SCANReplanFSM *fsm)
: node_(node),
  fsm_(fsm),
  stuck_timeout_sec_(kDefaultStuckTimeoutSec)
{
  server_ = rclcpp_action::create_server<FollowRoute>(
      node_, "/omni/navigation/follow_route",
      std::bind(&FollowRouteServer::handleGoal, this, std::placeholders::_1),
      std::bind(&FollowRouteServer::handleAccepted, this, std::placeholders::_1),
      std::bind(&FollowRouteServer::handleCancel, this, std::placeholders::_1));

  stuck_timeout_sec_ = node_->declare_parameter(
      "fsm.follow_route_stuck_timeout_sec", kDefaultStuckTimeoutSec);

  monitor_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(kMonitorPeriodSec),
      std::bind(&FollowRouteServer::monitorCallback, this));

  RCLCPP_INFO(
      node_->get_logger(),
      "FollowRoute action server ready on /omni/navigation/follow_route");
}

rclcpp_action::GoalResponse FollowRouteServer::handleGoal(
    const GoalHandle::SharedPtr &goal_handle)
{
  const FollowRoute::Goal &goal = *goal_handle->get_goal();

  // One route at a time; concurrent goals are rejected.
  if (mission_.active)
    return rclcpp_action::GoalResponse::REJECT;
  if (goal.mission_id.empty())
    return rclcpp_action::GoalResponse::REJECT;
  // Per-planner-epoch idempotency: a mission that already reached a terminal
  // result is rejected as a replay.
  if (finished_mission_ids_.count(goal.mission_id) > 0)
    return rclcpp_action::GoalResponse::REJECT;
  // The node must run in reference route mode to follow a route.
  if (!fsm_->isReferencePathMode())
    return rclcpp_action::GoalResponse::REJECT;
  // A usable localization is required before the route can start.
  if (!fsm_->hasValidOdom())
    return rclcpp_action::GoalResponse::REJECT;
  if (goal.path.poses.size() < 2)
    return rclcpp_action::GoalResponse::REJECT;

  const double speed_scale = static_cast<double>(goal.speed_scale);
  if (speed_scale != 0.0 &&
      (!std::isfinite(speed_scale) ||
       speed_scale < follow_route::SPEED_SCALE_MIN ||
       speed_scale > follow_route::SPEED_SCALE_MAX))
    return rclcpp_action::GoalResponse::REJECT;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void FollowRouteServer::handleAccepted(const GoalHandle::SharedPtr &goal_handle)
{
  const FollowRoute::Goal &goal = *goal_handle->get_goal();

  mission_ = ActiveMission{};
  mission_.active = true;
  mission_.goal_handle = goal_handle;
  mission_.mission_id = goal.mission_id;
  mission_.route_id = goal.route_id;

  // A cancel that raced the accept is finalized immediately; the route never
  // starts and the robot never moves.
  mission_.cancel_requested = goal_handle->is_canceling();
  mission_.cancel_requested_sec =
      mission_.cancel_requested ? node_->now().seconds() : -1.0;
  if (mission_.cancel_requested)
  {
    finish(goal_handle, false, follow_route::REASON_USER_CANCELED,
           "Canceled before the route started", 0.0);
    return;
  }

  const double speed_scale = static_cast<double>(goal.speed_scale);
  fsm_->setFollowRouteSpeedScale(speed_scale == 0.0 ? 1.0 : speed_scale);

  if (!fsm_->startFollowRoute(goal.path))
  {
    // The goal was accepted, but the route could not be accepted (invalid
    // geometry, or global trajectory planning failed). The action protocol
    // requires a result for every accepted goal.
    RCLCPP_ERROR(
        node_->get_logger(),
        "FollowRoute mission '%s' rejected: route could not be accepted",
        goal.mission_id.c_str());
    finish(goal_handle, false, follow_route::REASON_GOAL_REJECTED,
           "Route could not be accepted: invalid geometry or global trajectory "
           "planning failed",
           0.0);
    return;
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "FollowRoute mission '%s' (route '%s', speed_scale %.2f) accepted; "
      "following reference route",
      goal.mission_id.c_str(), goal.route_id.c_str(),
      speed_scale == 0.0 ? 1.0 : speed_scale);
}

rclcpp_action::CancelResponse FollowRouteServer::handleCancel(
    const GoalHandle::SharedPtr &goal_handle)
{
  if (mission_.active && mission_.goal_handle == goal_handle &&
      !mission_.result_sent)
  {
    mission_.cancel_requested = true;
    mission_.cancel_requested_sec = node_->now().seconds();
    // Controlled stop through the normal trajectory pipeline (NOT an
    // emergency stop). The monitor finalizes once the robot has stopped.
    fsm_->cancelFollowRoute();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowRouteServer::monitorCallback()
{
  if (!mission_.active || mission_.result_sent)
    return;

  const GoalHandle::SharedPtr goal_handle = mission_.goal_handle;
  if (!goal_handle)
    return;

  const SCANReplanFSM::FollowRouteView view = fsm_->followRouteView();

  // --- periodic feedback ---
  FollowRoute::Feedback feedback;
  feedback.mission_id = mission_.mission_id;
  feedback.state = view.motion_active ? follow_route::STATE_EXECUTING
                                      : follow_route::STATE_PLANNING;
  const double progress =
      view.route_active
          ? std::clamp(view.progress_ratio, 0.0, 1.0)
          : 0.0;
  mission_.last_progress = std::max(mission_.last_progress, progress);
  feedback.progress = static_cast<float>(mission_.last_progress);
  feedback.current_pose = fsm_->currentOdomPose();
  if (mission_.cancel_requested)
    feedback.status_text = "canceled; controlled stop in progress";
  else if (view.in_emergency_stop)
    feedback.status_text = "emergency stop; holding and recovering";
  else if (view.route_active)
    feedback.status_text = "following route";
  else
    feedback.status_text = "planning";
  goal_handle->publish_feedback(feedback);

  // --- terminal detection ---
  const rclcpp::Time now = node_->now();
  const double now_sec = now.seconds();

  if (view.route_outcome == SCANReplanFSM::RouteOutcome::SUCCEEDED)
  {
    finish(goal_handle, true, follow_route::REASON_OK, "Route completed", 1.0);
    return;
  }

  if (mission_.cancel_requested)
  {
    if (!view.have_odom ||
        view.route_outcome == SCANReplanFSM::RouteOutcome::LOCALIZATION_LOST)
    {
      finish(goal_handle, false, follow_route::REASON_LOCALIZATION_LOST,
             "Localization lost while canceling", mission_.last_progress);
      return;
    }
    if (view.exec_state == kStateWaitTarget || view.exec_state == kStateInit)
    {
      finish(goal_handle, false, follow_route::REASON_USER_CANCELED,
             "Canceled by user; controlled stop complete",
             mission_.last_progress);
      return;
    }
    if (now_sec - mission_.cancel_requested_sec > kCancelStopTimeoutSec)
    {
      // The robot did not stop in time. The mission manager releases the
      // motion lease when it receives this result, so velocity authority is
      // revoked externally.
      finish(goal_handle, false, follow_route::REASON_USER_CANCELED,
             "Canceled by user; controlled stop timed out",
             mission_.last_progress);
      return;
    }
  }

  if (view.route_outcome == SCANReplanFSM::RouteOutcome::ABORTED)
  {
    finish(goal_handle, false, follow_route::REASON_ABORTED,
           "Planner aborted the route (replanning failed)",
           mission_.last_progress);
    return;
  }

  if (view.route_outcome == SCANReplanFSM::RouteOutcome::LOCALIZATION_LOST)
  {
    finish(goal_handle, false, follow_route::REASON_LOCALIZATION_LOST,
           "body_pose timed out; route interrupted", mission_.last_progress);
    return;
  }

  if (view.in_emergency_stop)
  {
    if (mission_.emergency_stop_start_sec < 0.0)
      mission_.emergency_stop_start_sec = now_sec;
    else if (now_sec - mission_.emergency_stop_start_sec > stuck_timeout_sec_)
    {
      finish(goal_handle, false, follow_route::REASON_ABORTED,
             "Stuck in emergency stop beyond stuck timeout",
             mission_.last_progress);
      return;
    }
  }
  else
  {
    mission_.emergency_stop_start_sec = -1.0;
  }
}

void FollowRouteServer::finish(
    const GoalHandle::SharedPtr &goal_handle, bool success,
    std::uint32_t reason_code, const std::string &reason_text,
    double final_progress)
{
  if (mission_.result_sent || !goal_handle)
    return;
  mission_.result_sent = true;
  mission_.active = false;
  mission_.goal_handle.reset();

  // Per-planner-epoch dedup: a mission_id that reached a terminal result is
  // rejected on replay.
  if (finished_mission_ids_.size() >= follow_route::MAX_FINISHED_MISSION_IDS)
    finished_mission_ids_.clear();
  finished_mission_ids_.insert(mission_.mission_id);

  fsm_->resetFollowRouteCancel();

  FollowRoute::Result result;
  result.success = success;
  result.reason_code = reason_code;
  result.reason_text = reason_text;
  result.final_progress =
      static_cast<float>(std::clamp(final_progress, 0.0, 1.0));

  RCLCPP_INFO(
      node_->get_logger(),
      "FollowRoute mission '%s' finished: success=%d reason=%u (%s) "
      "progress=%.3f",
      mission_.mission_id.c_str(), success ? 1 : 0, reason_code,
      reason_text.c_str(), result.final_progress);

  if (success)
    goal_handle->succeeded(result);
  else if (reason_code == follow_route::REASON_USER_CANCELED)
    goal_handle->canceled(result);
  else
    goal_handle->aborted(result);
}

}  // namespace scan_planner