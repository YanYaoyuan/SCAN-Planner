// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file scan_replan_fsm.h @brief Replanning state-machine API. */

#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <atomic>
#include <cstdint>
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

  public:
    /** @brief Terminal outcome of the last accepted reference route. */
    enum class RouteOutcome
    {
      NONE = 0,
      SUCCEEDED = 1,
      ABORTED = 2,
      LOCALIZATION_LOST = 3,
    };

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
    bool require_tf_ready_{false};
    bool tf_ready_{true};

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
    bool follow_route_cancel_pending_{false};
    double follow_route_speed_scale_{1.0};
    RouteOutcome last_reference_route_outcome_{RouteOutcome::NONE};
    std::atomic<std::uint64_t> task_revision_{0};
    std::string active_mission_id_;
    std::string active_route_id_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, heartbeat_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tf_ready_sub_;
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
    /** @brief Advances the authoritative task generation after replacement or invalidation. */
    void advanceTaskRevision(const char *reason);
    /** @brief Reads the authoritative revisions used by the commit boundary. */
    PlanningInputRevision currentPlanningRevisions() const noexcept;
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
    /** @brief Revokes planning authority whenever omni_tf_manager is not ready. */
    void tfReadyCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
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

    /** @brief Immutable FSM snapshot consumed by the FollowRoute action
     *  server. */
    struct FollowRouteView
    {
      bool have_odom = false;
      bool route_active = false;
      bool in_emergency_stop = false;
      bool motion_active = false;
      double progress_ratio = 0.0;
      double planar_speed = 0.0;
      int exec_state = 0;
      RouteOutcome route_outcome = RouteOutcome::NONE;
    };

    /** @brief Tests whether the node runs in reference route mode.
     *  @return True when navi_mode is REFERENCE_PATH. */
    bool isReferencePathMode() const;

    /** @brief Tests whether a valid body_pose has been received and is not
     *  stale. @return True when odometry is usable for planning. */
    bool hasValidOdom() const;

    /** @brief Accepts a reference route through the mode-3 pipeline.
     *  @param path Route waypoints in the goal frame.
     *  @param mission_id Optional action mission identity for diagnostics.
     *  @param route_id Optional action route identity for diagnostics.
     *  @return True when the route became active. */
    bool startFollowRoute(
        const nav_msgs::msg::Path &path,
        const std::string &mission_id = {},
        const std::string &route_id = {});

    /** @brief Requests a controlled stop for the active route (decelerate
     *  through the normal pipeline; NOT an emergency stop). No-op when no
     *  route is active. */
    void cancelFollowRoute();

    /** @brief Clears the pending cancel flag. Idempotent. */
    void resetFollowRouteCancel();

    /** @brief Applies the FollowRoute speed scale to the current route.
     *  @param scale 1.0 for the planner default. */
    void setFollowRouteSpeedScale(double scale);

    /** @brief Returns the latest body pose in the odometry frame. */
    geometry_msgs::msg::PoseStamped currentOdomPose() const;

    /** @brief Returns the FSM snapshot consumed by the FollowRoute action
     *  server. */
    FollowRouteView followRouteView() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
