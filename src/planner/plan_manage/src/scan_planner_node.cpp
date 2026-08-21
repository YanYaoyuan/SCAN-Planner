// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file scan_planner_node.cpp @brief ROS 2 entry point for SCAN-Planner. */

#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <plan_manage/follow_route_server.h>
#include <plan_manage/scan_replan_fsm.h>

/** @brief Runs the SCAN planner ROS node. @param argc Argument count. @param argv Argument vector. @return Process exit status. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("omni_scan_planner");

  try
  {
    scan_planner::SCANReplanFSM planner;
    planner.init(node.get());

    // The FollowRoute action server drives the same reference-route pipeline
    // as /initial_path; it is only created in route-following mode (3).
    std::unique_ptr<scan_planner::FollowRouteServer> follow_route_server;
    if (planner.isReferencePathMode())
    {
      follow_route_server =
          std::make_unique<scan_planner::FollowRouteServer>(node, &planner);
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
