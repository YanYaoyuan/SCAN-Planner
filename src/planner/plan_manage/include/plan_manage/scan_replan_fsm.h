// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file scan_replan_fsm.h @brief Replanning state-machine API. */

#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <plan_manage/local_trajectory_tracker.h>
#include <plan_manage/pure_pursuit_recovery_sweep.h>
#include <plan_manage/reference_path_tracker.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <scan_planner_msgs/msg/planner_heartbeat.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  /** @brief Drives navigation targets, local replanning, execution, and emergency states. */
  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    double reference_path_height_offset_;
    double reference_path_min_point_spacing_;
    double reference_path_closed_tolerance_;
    double reference_path_min_length_;
    double reference_progress_max_advance_;
    double reference_progress_movement_scale_;
    double reference_progress_movement_deadband_;
    double reference_progress_initial_credit_;
    double reference_departure_distance_;
    double reference_completion_min_ratio_;
    double reference_finish_distance_;
    double reference_finish_remaining_length_;
    double odom_timeout_;
    double local_progress_sample_dt_;
    double local_progress_max_advance_;
    double local_progress_movement_scale_;
    double local_progress_movement_deadband_;
    double local_progress_initial_credit_;
    double local_progress_collision_backtrack_;
    double local_progress_max_cross_track_;
    double local_recovery_check_min_cross_track_;
    double local_recovery_lookahead_;
    double local_recovery_yaw_lookahead_;
    double local_recovery_sample_distance_;
    RecoverySweepControlLimits local_recovery_control_limits_;
    double local_finish_distance_;
    double local_finish_remaining_length_;
    double local_finish_speed_;
    std::string self_inflation_frame_id_;
    std::string expected_odom_frame_;
    std::string goal_frame_id_;
    double goal_transform_timeout_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool reference_path_active_{false};
    bool reference_path_closed_{false};
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    bool odom_timeout_active_{false};
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;
    rclcpp::Time last_odom_receive_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    std::vector<Eigen::Vector3d> local_reference_seed_;
    ReferencePathTracker reference_path_tracker_;
    LocalTrajectoryTracker local_trajectory_tracker_;
    int local_progress_traj_id_{-1};
    int current_wp_;
    double reference_path_total_length_{0.0};
    double reference_local_target_arc_length_{0.0};

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, heartbeat_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav_msgs::msg::Path::ConstSharedPtr pending_reference_path_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::PlannerHeartbeat>::SharedPtr planner_heartbeat_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    /** @brief Invokes local rebound planning. @param flag_use_poly_init Force fresh initialization. @param flag_randomPolyTraj Enable randomized fallback. @return True on success. */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    /** @brief Publishes a stationary emergency trajectory. @param stop_pos Stop position. @return True on success. */
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    /** @brief Replans from current odometry and trajectory derivatives. @return True on success. */
    bool planFromCurrentTraj();
    /** @brief Selects start position from odometry and derivatives from the active trajectory. */
    void setStartStateFromOdomOrCurrentTraj();

    /** @brief Changes FSM state and resets consecutive-call accounting. @param new_state Destination state. @param pos_call Caller label for diagnostics. */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    /** @brief Updates consecutive-state call count. @return Call count and current state. */
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    /** @brief Prints throttled FSM diagnostics. */
    void printFSMExecState();

    /** @brief Starts the configured preset waypoint sequence. */
    void planGlobalTrajbyGivenWps();
    /** @brief Builds a global trajectory through waypoints. @param waypoints Ordered points. @return True on success. */
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    /** @brief Plans to the next preset waypoint. @return True on success. */
    bool planNextWaypoint();
    /** @brief Tests whether preset waypoint sequencing is active. @return True in mode 2. */
    bool isWaypointSequenceMode() const;
    /** @brief Validates and deduplicates a supplied reference route. @param input Raw points. @param[out] output Cleaned points. @return True when valid. */
    bool prepareReferenceWaypoints(const std::vector<Eigen::Vector3d> &input,
                                   std::vector<Eigen::Vector3d> &output);
    /** @brief Projects odometry monotonically onto the immutable reference polyline. @return Updated arc-length progress in metres. */
    double updateReferencePathProgress();
    /** @brief Computes unconsumed reference-route length. @return Remaining arc length in meters. */
    double referencePathRemainingLength() const;
    /** @brief Checks route-progress and endpoint completion conditions. @return True when complete. */
    bool referencePathComplete();
    /** @brief Moves an occupied point goal backward to free space. @return True when a usable goal exists. */
    bool adjustGlobalTargetIfOccupied();
    /** @brief Transforms a goal into planner coordinates. @param input Input pose. @param[out] output Transformed pose. @return True on success. */
    bool transformPoseToGoalFrame(const geometry_msgs::msg::PoseStamped &input,
                                  geometry_msgs::msg::PoseStamped &output) const;
    /** @brief Chooses the forward local target and optional route seed. */
    void getLocalTarget();
    /** @brief Applies failure-count fail-safe handling. */
    void finishProcess();
    /** @brief Clears the controller's active trajectory. @param reason Diagnostic reason. */
    void publishTrajectoryClear(const std::string &reason);
    /** @brief Publishes the robot inflation model marker. */
    void publishSelfInflationMarker();
    /** @brief Returns current odometry yaw. @return Yaw in radians. */
    double getOdomYaw() const;
    /** @brief Estimates planar segment yaw. @param from Segment start. @param to Segment end. @return Yaw in radians. */
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    /** @brief 兼容旧反馈；空间进度模式下仅更新时间戳，不再平移轨迹时间。 */
    void updateLocalTrajTimeFreeze();
    /** @brief 为当前局部 B-spline 建立空间进度采样。 */
    bool resetLocalTrajectoryProgress();
    /** @brief 用实际里程计更新局部轨迹进度并返回对应 B-spline 时间。 */
    double currentLocalTrajectoryTime();
    /** @brief 返回带空间回退余量的碰撞扫描起始时间。 */
    double localCollisionScanStartTime(double progress_time) const;
    /** @brief 根据末端距离、剩余弧长和实测速度判断局部轨迹是否完成。 */
    bool localTrajectoryComplete() const;
    /** @brief 周期发布带轨迹身份的规划器存活心跳。 */
    void publishPlannerHeartbeat();

    /** @brief Main FSM timer callback. */
    void execFSMCallback();
    /** @brief Active-trajectory collision-check timer callback. */
    void checkCollisionCallback();
    /** @brief Receives an RViz point goal. @param msg Goal pose. */
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    /** @brief Receives legacy waypoint paths. @param msg Waypoint path. */
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    /** @brief Receives a global reference route. @param msg Reference path. */
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    /** @brief Receives validated body odometry. @param msg Body odometry. */
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    /** @brief Receives controller execution-freeze state. @param msg Freeze flag. */
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    /** @brief Checks the current local trajectory against the map. @return True when a collision is detected. */
    bool checkCollision();

  public:
    /** @brief Constructs an uninitialized FSM. */
    SCANReplanFSM(/* args */)
    {
    }
    /** @brief Destroys the FSM. */
    ~SCANReplanFSM()
    {
    }

    /** @brief Loads parameters and creates ROS/planning modules. @param node Owning ROS node. */
    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
