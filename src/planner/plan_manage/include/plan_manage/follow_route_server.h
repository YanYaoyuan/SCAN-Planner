// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file follow_route_server.h @brief FollowRoute action server for reference route missions. */

#ifndef _FOLLOW_ROUTE_SERVER_H_
#define _FOLLOW_ROUTE_SERVER_H_

#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <omni_robot_interfaces/action/follow_route.hpp>

#include <plan_manage/follow_route_constants.h>
#include <plan_manage/scan_replan_fsm.h>

namespace scan_planner
{

  /** @brief Serves /omni/navigation/follow_route over the mode-3 route
   *  pipeline.
   *
   *  One goal at a time: concurrent goals are rejected. The mission_id is
   *  the per-planner-epoch dedup key: a goal whose mission_id already
   *  reached a terminal result is rejected (idempotent replay). Cancel is a
   *  controlled stop (decelerate through the normal pipeline, NOT an
   *  emergency stop) and always finalizes with REASON_USER_CANCELED.
   */
  class FollowRouteServer
  {
  public:
    using FollowRoute = omni_robot_interfaces::action::FollowRoute;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowRoute>;

    /** @brief Creates the action server and monitor timer.
     *  @param node ROS node (shared ownership; required by the action
     *  server factory).
     *  @param fsm FSM driving the route pipeline (non-owning). */
    FollowRouteServer(const rclcpp::Node::SharedPtr &node, SCANReplanFSM *fsm);

  private:
    struct ActiveMission
    {
      bool active = false;
      bool cancel_requested = false;
      bool result_sent = false;
      std::shared_ptr<GoalHandle> goal_handle;
      std::string mission_id;
      std::string route_id;
      double last_progress = 0.0;
      double cancel_requested_sec = -1.0;
      double emergency_stop_start_sec = -1.0;
    };

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        const std::shared_ptr<const FollowRoute::Goal> &goal);
    void handleAccepted(const std::shared_ptr<GoalHandle> &goal_handle);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> &goal_handle);
    void monitorCallback();
    void finish(const std::shared_ptr<GoalHandle> &goal_handle, bool success,
                std::uint32_t reason_code, const std::string &reason_text,
                double final_progress);

    rclcpp::Node::SharedPtr node_;
    SCANReplanFSM *fsm_;
    rclcpp_action::Server<FollowRoute>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    double stuck_timeout_sec_;
    ActiveMission mission_;
    std::unordered_set<std::string> finished_mission_ids_;
  };

}  // namespace scan_planner

#endif  // _FOLLOW_ROUTE_SERVER_H_