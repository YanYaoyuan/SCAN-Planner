// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file planner_manager.h @brief Global and local trajectory planner manager API. */

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace scan_planner
{

  /** @brief Coordinates global references, B-spline optimization, and local trajectory state. */
  class SCANPlannerManager
  {
    // SECTION stable
  public:
    /** @brief Constructs an uninitialized manager. */
    SCANPlannerManager();
    /** @brief Releases owned planning modules. */
    ~SCANPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Generates a collision-free local B-spline.
     * @param start_pt Start position.
     * @param start_vel Start velocity.
     * @param start_acc Start acceleration.
     * @param end_pt Local target position.
     * @param end_vel Local target velocity.
     * @param flag_polyInit Force polynomial/reference initialization.
     * @param flag_randomPolyTraj Enable randomized fallback initialization.
     * @param reference_seed Optional ordered reference-route samples.
     * @return True when a safe, dynamically feasible trajectory is produced.
     */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit,
                       bool flag_randomPolyTraj,
                       const std::vector<Eigen::Vector3d> &reference_seed = {});
    /** @brief Generates a stationary emergency trajectory. @param stop_pos Hold position. @return True on success. */
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    /** @brief Generates a point-to-point global polynomial. @param start_pos Start position. @param start_vel Start velocity. @param start_acc Start acceleration. @param end_pos Goal position. @param end_vel Goal velocity. @param end_acc Goal acceleration. @return True on success. */
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    /** @brief Generates a global polynomial through ordered waypoints. @param start_pos Start position. @param start_vel Start velocity. @param start_acc Start acceleration. @param waypoints Ordered route points. @param end_vel Final velocity. @param end_acc Final acceleration. @return True on success. */
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /** @brief Loads parameters and creates planning modules. @param node Owning ROS node. @param vis Optional shared visualization helper. */
    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    rclcpp::Node *node_{nullptr};
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continuous_failures_count_{0};

    /** @brief Stores a newly planned local trajectory. @param position_traj Position spline. @param time_now Publication start time. */
    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);
    /** @brief Performs final dynamic-limit validation. @param position_traj Position spline. @return True when within configured limits. */
    bool checkDynamicFeasibility(UniformBspline position_traj);

    /** @brief Reparameterizes a B-spline after time scaling. @param[in,out] bspline Position spline. @param start_end_derivative Boundary derivatives. @param ratio Time scaling. @param[out] ctrl_pts New control points. @param[out] dt New knot interval. @param[out] time_inc Added duration. */
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    /** @brief Iteratively refines a dynamically infeasible trajectory. @param[in,out] traj Position spline. @param start_end_derivative Boundary derivatives. @param ratio Initial time scaling. @param[in,out] ts Knot interval. @param[out] optimal_control_points Refined points. @return True on success. */
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<SCANPlannerManager> Ptr;

    // !SECTION
  };
} // namespace scan_planner

#endif
