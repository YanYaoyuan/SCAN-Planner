// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file scan_replan_fsm.cpp @brief Implements replanning modes and safety-state transitions. */

#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/pure_pursuit_recovery_sweep.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
  /**
   * @brief Declares and reads a ROS parameter.
   * @tparam T Parameter value type.
   * @param node Parameter-owning node.
   * @param name Fully qualified parameter name.
   * @param default_value Value used when undeclared.
   * @return Loaded parameter value.
   */
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();
    last_odom_source_time_ = node_->now();
    last_odom_callback_time_ = std::chrono::steady_clock::now();
    use_sim_time_ = load_parameter<bool>(node_, "use_sim_time", false);

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    failure_retry_period_ =
        load_parameter<double>(node_, "fsm.failure_retry_period", 0.2);
    min_replan_period_ =
        load_parameter<double>(node_, "fsm.min_replan_period", 0.2);
    max_replan_period_ =
        load_parameter<double>(node_, "fsm.max_replan_period", 2.0);
    if (!std::isfinite(failure_retry_period_) || failure_retry_period_ < 0.0)
      failure_retry_period_ = 0.2;
    if (!std::isfinite(min_replan_period_) || min_replan_period_ < 0.0)
      min_replan_period_ = 0.2;
    if (!std::isfinite(max_replan_period_) || max_replan_period_ <= 0.0)
      max_replan_period_ = 2.0;
    if (max_replan_period_ < min_replan_period_)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "fsm.max_replan_period is smaller than fsm.min_replan_period; "
          "using %.3f s",
          min_replan_period_);
      max_replan_period_ = min_replan_period_;
    }
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    reference_path_height_offset_ =
        load_parameter<double>(node_, "fsm.reference_path_height_offset", 0.0);
    reference_path_min_point_spacing_ =
        std::max(0.01, load_parameter<double>(node_, "fsm.reference_path_min_point_spacing", 0.05));
    reference_path_closed_tolerance_ =
        std::max(reference_path_min_point_spacing_,
                 load_parameter<double>(node_, "fsm.reference_path_closed_tolerance", 0.30));
    reference_path_min_length_ =
        std::max(reference_path_closed_tolerance_,
                 load_parameter<double>(node_, "fsm.reference_path_min_length", 1.0));
    reference_path_same_tolerance_ = std::max(
        0.0,
        load_parameter<double>(
            node_, "fsm.reference_path_same_tolerance", 0.02));
    reference_progress_max_advance_ =
        std::max(reference_path_min_point_spacing_,
                 load_parameter<double>(node_, "fsm.reference_progress_max_advance", 2.0));
    reference_progress_movement_scale_ =
        std::max(1.0,
                 load_parameter<double>(node_, "fsm.reference_progress_movement_scale", 1.5));
    reference_progress_movement_deadband_ =
        std::max(0.0,
                 load_parameter<double>(node_, "fsm.reference_progress_movement_deadband", 0.02));
    reference_progress_initial_credit_ =
        std::max(0.0,
                 load_parameter<double>(node_, "fsm.reference_progress_initial_credit", 0.10));
    reference_departure_distance_ =
        std::max(reference_path_closed_tolerance_,
                 load_parameter<double>(node_, "fsm.reference_departure_distance", 0.60));
    reference_completion_min_ratio_ =
        std::clamp(
            load_parameter<double>(node_, "fsm.reference_completion_min_ratio", 0.95),
            0.5,
            1.0);
    reference_finish_distance_ =
        std::max(reference_path_min_point_spacing_,
                 load_parameter<double>(node_, "fsm.reference_finish_distance", 0.30));
    reference_finish_remaining_length_ =
        std::max(reference_path_min_point_spacing_,
                 load_parameter<double>(node_, "fsm.reference_finish_remaining_length", 0.30));
    reference_path_tracker_.setConfig(
        ReferencePathTracker::Config{
            reference_progress_max_advance_,
            reference_progress_movement_scale_,
            reference_progress_movement_deadband_,
            reference_progress_initial_credit_,
            reference_departure_distance_});
    odom_timeout_ = std::max(0.05, load_parameter<double>(node_, "fsm.odom_timeout", 0.3));
    local_progress_sample_dt_ = std::clamp(
        load_parameter<double>(node_, "fsm.local_progress_sample_dt", 0.05),
        0.01,
        0.20);
    local_progress_max_advance_ = std::max(
        0.10,
        load_parameter<double>(node_, "fsm.local_progress_max_advance", 0.75));
    local_progress_movement_scale_ = std::max(
        1.0,
        load_parameter<double>(node_, "fsm.local_progress_movement_scale", 1.5));
    local_progress_movement_deadband_ = std::max(
        0.0,
        load_parameter<double>(node_, "fsm.local_progress_movement_deadband", 0.01));
    local_progress_initial_credit_ = std::max(
        0.0,
        load_parameter<double>(node_, "fsm.local_progress_initial_credit", 0.15));
    local_progress_collision_backtrack_ = std::max(
        0.0,
        load_parameter<double>(node_, "fsm.local_progress_collision_backtrack", 0.10));
    local_progress_max_cross_track_ = std::max(
        0.10,
        load_parameter<double>(node_, "fsm.local_progress_max_cross_track", 0.40));
    local_recovery_check_min_cross_track_ = std::clamp(
        load_parameter<double>(node_, "fsm.local_recovery_check_min_cross_track", 0.05),
        0.0,
        local_progress_max_cross_track_);
    local_recovery_lookahead_ = std::max(
        0.10,
        load_parameter<double>(node_, "fsm.local_recovery_lookahead", 0.45));
    local_recovery_yaw_lookahead_ = std::max(
        0.10,
        load_parameter<double>(node_, "fsm.local_recovery_yaw_lookahead", 0.55));
    local_recovery_sample_distance_ = std::clamp(
        load_parameter<double>(node_, "fsm.local_recovery_sample_distance", 0.05),
        0.02,
        0.10);
    if (local_recovery_sample_distance_ < 0.05)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "fsm.local_recovery_sample_distance=%.3f is below the real-robot "
          "recommendation 0.05m; extreme cross-track recovery may exceed the "
          "bounded sweep budget and fail closed",
          local_recovery_sample_distance_);
    }

    // 这些默认值与 controllers.yaml 的 PathTrackingController 和
    // PathCommandGovernor 一一对应。两个 ROS 节点无法共享有状态对象，FSM
    // 因此读取同一组几何/底盘边界，构造实际可发布命令的保守碰撞包络。
    local_recovery_control_limits_.heading_feedback_gain = std::max(
        0.0,
        load_parameter<double>(
            node_, "fsm.local_recovery_heading_feedback_gain", 0.35));
    local_recovery_control_limits_.align_yaw_gain = std::max(
        0.0,
        load_parameter<double>(
            node_, "fsm.local_recovery_align_yaw_gain", 0.80));
    local_recovery_control_limits_.align_enter_error = std::max(
        0.05,
        load_parameter<double>(
            node_, "fsm.local_recovery_align_enter_error", 0.80));
    local_recovery_control_limits_.align_exit_error = std::clamp(
        load_parameter<double>(
            node_, "fsm.local_recovery_align_exit_error", 0.45),
        0.0,
        local_recovery_control_limits_.align_enter_error - 1.0e-4);
    local_recovery_control_limits_.lateral_error_deadband = std::max(
        0.0,
        load_parameter<double>(
            node_, "fsm.local_recovery_lateral_error_deadband", 0.04));
    local_recovery_control_limits_.heading_error_deadband = std::max(
        0.0,
        load_parameter<double>(
            node_, "fsm.local_recovery_heading_error_deadband", 0.035));
    local_recovery_control_limits_.curvature_deadband = std::max(
        0.0,
        load_parameter<double>(
            node_, "fsm.local_recovery_curvature_deadband", 0.04));
    local_recovery_control_limits_.minimum_lookahead = std::max(
        0.01,
        load_parameter<double>(
            node_, "fsm.local_recovery_minimum_lookahead", 0.05));
    local_recovery_control_limits_.min_forward_speed = std::clamp(
        load_parameter<double>(
            node_, "fsm.local_recovery_min_forward_speed", 0.05),
        0.001,
        1.0);
    local_recovery_control_limits_.max_yaw_rate = std::clamp(
        load_parameter<double>(
            node_, "fsm.local_recovery_max_yaw_rate", 0.50),
        0.01,
        2.0);
    local_recovery_control_limits_.yaw_deadband = std::clamp(
        load_parameter<double>(
            node_, "fsm.local_recovery_yaw_deadband", 0.025),
        0.0,
        local_recovery_control_limits_.max_yaw_rate);
    local_recovery_control_limits_.yaw_start_threshold = std::clamp(
        load_parameter<double>(
            node_, "fsm.local_recovery_yaw_start_threshold", 0.06),
        local_recovery_control_limits_.yaw_deadband,
        local_recovery_control_limits_.max_yaw_rate);
    local_recovery_control_limits_.min_yaw_rate = std::clamp(
        load_parameter<double>(
            node_, "fsm.local_recovery_min_yaw_rate", 0.10),
        0.0,
        local_recovery_control_limits_.max_yaw_rate);
    local_finish_distance_ = std::max(
        0.05,
        load_parameter<double>(node_, "fsm.local_finish_distance", 0.15));
    local_finish_remaining_length_ = std::max(
        0.05,
        load_parameter<double>(node_, "fsm.local_finish_remaining_length", 0.18));
    local_finish_speed_ = std::clamp(
        load_parameter<double>(node_, "fsm.local_finish_speed", 0.06),
        0.0,
        0.50);
    publish_local_path_ =
        load_parameter<bool>(node_, "local_path.publish", false);
    local_path_sample_dt_ = std::clamp(
        load_parameter<double>(node_, "local_path.sample_dt", 0.10),
        0.01,
        1.0);
    local_path_max_samples_ = std::clamp(
        load_parameter<int>(node_, "local_path.max_samples", 10000),
        2,
        10000);
    local_trajectory_tracker_.setConfig(
        LocalTrajectoryTracker::Config{
            local_progress_max_advance_,
            local_progress_movement_scale_,
            local_progress_movement_deadband_,
            local_progress_initial_credit_});
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    expected_odom_frame_ =
        load_parameter<std::string>(node_, "fsm.expected_odom_frame", self_inflation_frame_id_);
    goal_frame_id_ = load_parameter<std::string>(node_, "fsm.goal_frame_id", self_inflation_frame_id_);
    goal_transform_timeout_ = load_parameter<double>(node_, "fsm.goal_transform_timeout", 0.2);
    if (goal_frame_id_.empty())
      goal_frame_id_ = self_inflation_frame_id_;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", rclcpp::QoS(1).reliable().transient_local());
    if (publish_local_path_)
    {
      local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
          "planning/local_path",
          rclcpp::QoS(1).reliable().transient_local());
    }
    planner_heartbeat_pub_ =
        node_->create_publisher<scan_planner_msgs::msg::PlannerHeartbeat>(
            "planning/planner_heartbeat",
            rclcpp::QoS(1).reliable().durability_volatile());
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    heartbeat_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&SCANReplanFSM::publishPlannerHeartbeat, this));

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    if (!transformPoseToGoalFrame(*msg, goal))
      return;

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = goal.header;
    path->poses.push_back(goal);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    geometry_msgs::msg::PoseStamped goal;
    if (!transformPoseToGoalFrame(msg->poses[0], goal))
      return;
    if (goal.pose.position.z < -0.1)
      return;

    end_pt_ << goal.pose.position.x, goal.pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    // Commit the replacement goal only after the complete polynomial route
    // has been generated. A malformed update must not alter the endpoint of
    // the route that is currently being executed.
    end_pt_ = waypoints.back();

    // Do not truncate a supplied reference route merely because its final
    // point is currently occupied. Mode 3 handles occupancy on each local
    // target and must preserve the caller's full route and completion point.

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::prepareReferenceWaypoints(
      const std::vector<Eigen::Vector3d> &input,
      std::vector<Eigen::Vector3d> &output,
      bool &path_closed,
      double &path_length)
  {
    output.clear();
    path_closed = false;
    path_length = 0.0;

    if (input.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference path contains no points");
      return false;
    }

    path_closed =
        input.size() >= 2 &&
        (input.front().head<2>() - input.back().head<2>()).norm() <=
            reference_path_closed_tolerance_;

    output.reserve(input.size());
    for (const auto &point : input)
    {
      if (!point.allFinite())
      {
        RCLCPP_ERROR(node_->get_logger(), "Reference path contains a non-finite point");
        output.clear();
        return false;
      }
      if (output.empty() ||
          (point - output.back()).norm() >= reference_path_min_point_spacing_)
      {
        output.push_back(point);
      }
    }

    // planGlobalTrajWaypoints() prepends odom_pos_. Keeping an identical first
    // path point would create a zero-duration polynomial segment.
    while (output.size() > 1 &&
           (output.front() - odom_pos_).norm() < reference_path_min_point_spacing_)
    {
      output.erase(output.begin());
    }

    if (output.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference path is empty after duplicate removal");
      return false;
    }

    Eigen::Vector3d previous = odom_pos_;
    for (const auto &point : output)
    {
      path_length += (point - previous).norm();
      previous = point;
    }

    if (path_length < reference_path_min_length_)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Reference path is too short after cleaning: %.3f m (minimum %.3f m)",
          path_length,
          reference_path_min_length_);
      output.clear();
      return false;
    }

    if (path_closed && output.size() < 3)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Closed reference path has too few distinct points after cleaning");
      output.clear();
      return false;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Prepared %s reference path: %zu points, %.2f m",
        path_closed ? "closed" : "open",
        output.size(),
        path_length);
    return true;
  }

  double SCANReplanFSM::updateReferencePathProgress()
  {
    if (!reference_path_active_ || !reference_path_tracker_.active())
      return 0.0;
    return reference_path_tracker_.update(odom_pos_);
  }

  double SCANReplanFSM::referencePathRemainingLength() const
  {
    if (!reference_path_active_ || !reference_path_tracker_.active())
      return 0.0;
    return reference_path_tracker_.remainingLength();
  }

  bool SCANReplanFSM::referencePathComplete()
  {
    if (!reference_path_active_)
      return false;

    updateReferencePathProgress();
    const double remaining = referencePathRemainingLength();
    const double distance_to_end =
        (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
    const double total_length = reference_path_tracker_.totalLength();
    const double progress_ratio =
        total_length > 1.0e-9
            ? reference_path_tracker_.progress() / total_length
            : 0.0;
    const bool departure_armed =
        !reference_path_closed_ || reference_path_tracker_.departedStart();
    const bool complete =
        departure_armed &&
        progress_ratio >= reference_completion_min_ratio_ &&
        remaining <= reference_finish_remaining_length_ &&
        distance_to_end <= reference_finish_distance_;

    if (complete)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "%s reference path completed: progress %.1f%%, remaining %.3f m, "
          "end distance %.3f m",
          reference_path_closed_ ? "Closed" : "Open",
          100.0 * progress_ratio,
          remaining,
          distance_to_end);
    }
    return complete;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  bool SCANReplanFSM::transformPoseToGoalFrame(const geometry_msgs::msg::PoseStamped &input,
                                               geometry_msgs::msg::PoseStamped &output) const
  {
    output = input;
    if (goal_frame_id_.empty())
      return true;

    if (output.header.frame_id.empty())
    {
      output.header.frame_id = goal_frame_id_;
      RCLCPP_WARN(node_->get_logger(), "Goal has empty frame_id; assuming '%s'",
                  goal_frame_id_.c_str());
      return true;
    }

    if (output.header.frame_id == goal_frame_id_)
      return true;

    try
    {
      output = tf_buffer_->transform(
          input, goal_frame_id_, tf2::durationFromSec(std::max(0.0, goal_transform_timeout_)));
      RCLCPP_INFO(node_->get_logger(), "Transformed goal from '%s' to '%s'",
                  input.header.frame_id.c_str(), goal_frame_id_.c_str());
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Reject goal in frame '%s': cannot transform to planner frame '%s': %s",
          input.header.frame_id.c_str(), goal_frame_id_.c_str(), ex.what());
      return false;
    }
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    if (!have_odom_ || !planner_manager_->grid_map_->inputReady())
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Reference path is waiting for valid body_pose and the first occupancy update");
      pending_reference_path_ = msg;
      return;
    }

    std::vector<Eigen::Vector3d> raw_waypoints;
    raw_waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      geometry_msgs::msg::PoseStamped transformed_pose;
      geometry_msgs::msg::PoseStamped input_pose = pose_stamped;
      if (input_pose.header.frame_id.empty())
        input_pose.header.frame_id = msg->header.frame_id;
      if (!transformPoseToGoalFrame(input_pose, transformed_pose))
      {
        // TF may legitimately arrive after a transient-local route. Keep the
        // sample pending so the FSM timer can retry without requiring the
        // upstream publisher to send the same route again.
        pending_reference_path_ = msg;
        return;
      }

      Eigen::Vector3d wp;
      wp(0) = transformed_pose.pose.position.x;
      wp(1) = transformed_pose.pose.position.y;
      wp(2) = transformed_pose.pose.position.z + reference_path_height_offset_;
      raw_waypoints.push_back(wp);
    }

    if (reference_path_active_ &&
        raw_waypoints.size() == last_reference_input_.size())
    {
      bool same_path = true;
      for (size_t index = 0; index < raw_waypoints.size(); ++index)
      {
        if ((raw_waypoints[index] - last_reference_input_[index]).norm() >
            reference_path_same_tolerance_)
        {
          same_path = false;
          break;
        }
      }
      if (same_path)
      {
        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Ignoring repeated reference path while the current route is active");
        return;
      }
    }

    std::vector<Eigen::Vector3d> waypoints;
    bool candidate_path_closed = false;
    double candidate_path_length = 0.0;
    if (!prepareReferenceWaypoints(
            raw_waypoints,
            waypoints,
            candidate_path_closed,
            candidate_path_length))
    {
      return;
    }

    std::vector<Eigen::Vector3d> tracker_points;
    tracker_points.reserve(waypoints.size() + 1);
    tracker_points.push_back(odom_pos_);
    tracker_points.insert(
        tracker_points.end(), waypoints.begin(), waypoints.end());
    ReferencePathTracker candidate_tracker;
    candidate_tracker.setConfig(
        ReferencePathTracker::Config{
            reference_progress_max_advance_,
            reference_progress_movement_scale_,
            reference_progress_movement_deadband_,
            reference_progress_initial_credit_,
            reference_departure_distance_});
    try
    {
      candidate_tracker.reset(
          tracker_points, candidate_path_closed, odom_pos_);
      candidate_path_length = candidate_tracker.totalLength();
    }
    catch (const std::invalid_argument &error)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Unable to initialize reference progress tracker: %s",
          error.what());
      return;
    }

    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      trigger_ = true;
      last_accepted_reference_path_ = msg;
      last_reference_input_ = raw_waypoints;
      reference_path_tracker_ = std::move(candidate_tracker);
      reference_path_closed_ = candidate_path_closed;
      reference_path_total_length_ = candidate_path_length;
      reference_local_target_arc_length_ = 0.0;
      reference_path_active_ = true;
      planner_manager_->global_data_.last_progress_time_ = 0.0;
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(
          node_->get_logger(),
          "%s reference path accepted; completion requires forward progress through the full route",
          reference_path_closed_ ? "Closed" : "Open");
    }
    else
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Unable to generate replacement global trajectory; keeping the current route");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    if (!msg)
      return;
    const auto &position = msg->pose.pose.position;
    const auto &orientation = msg->pose.pose.orientation;
    const auto &linear = msg->twist.twist.linear;
    const double quaternion_norm2 =
        orientation.w * orientation.w + orientation.x * orientation.x +
        orientation.y * orientation.y + orientation.z * orientation.z;
    const bool finite =
        std::isfinite(position.x) && std::isfinite(position.y) &&
        std::isfinite(position.z) && std::isfinite(linear.x) &&
        std::isfinite(linear.y) && std::isfinite(linear.z) &&
        std::isfinite(quaternion_norm2) && quaternion_norm2 > 1.0e-12;
    if (!finite)
    {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Rejecting body_pose with non-finite values or invalid quaternion");
      return;
    }
    if (msg->header.frame_id.empty() ||
        (!expected_odom_frame_.empty() &&
         msg->header.frame_id != expected_odom_frame_))
    {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Rejecting body_pose in frame '%s'; expected '%s'",
          msg->header.frame_id.c_str(), expected_odom_frame_.c_str());
      return;
    }
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    odom_orient_.normalize();

    have_odom_ = true;
    if (local_progress_traj_id_ == planner_manager_->local_data_.traj_id_ &&
        local_trajectory_tracker_.active())
    {
      planner_manager_->local_data_.progress_arc_length_ =
          local_trajectory_tracker_.update(odom_pos_);
      planner_manager_->local_data_.progress_time_ =
          local_trajectory_tracker_.progressTime();
    }
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        reference_path_active_ &&
        reference_path_tracker_.active())
    {
      reference_path_tracker_.update(odom_pos_);
    }
    const rclcpp::Time odom_source_time(msg->header.stamp);
    if (odom_source_time.nanoseconds() > 0)
    {
      last_odom_source_time_ = odom_source_time;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "body_pose has a zero source timestamp; freshness falls back to callback time");
      last_odom_source_time_ = node_->now();
    }
    last_odom_callback_time_ = std::chrono::steady_clock::now();
    odom_timeout_active_ = false;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && pending_reference_path_)
    {
      auto pending_path = pending_reference_path_;
      pending_reference_path_.reset();
      pathCallback(pending_path);
    }
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    // 真实进度由 odom 在局部轨迹上的空间投影决定。ALIGN 时机器人不移动，
    // 投影进度会自然冻结，因此绝不能再平移 start_time_。start_time_ 也是
    // planner heartbeat 的轨迹身份之一，修改它会让控制器拒绝合法心跳。
    last_freeze_update_time_ = node_->now();
  }

  bool SCANReplanFSM::resetLocalTrajectoryProgress()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    local_trajectory_tracker_.clear();
    local_progress_traj_id_ = -1;
    info->progress_time_ = 0.0;
    info->progress_arc_length_ = 0.0;

    if (!std::isfinite(info->duration_) || info->duration_ <= 1.0e-6)
      return false;

    const int sample_count = std::max(
        2,
        static_cast<int>(std::ceil(
            info->duration_ / local_progress_sample_dt_)) + 1);
    if (sample_count > 10000)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Local trajectory %d requires too many progress samples: %d",
          info->traj_id_, sample_count);
      return false;
    }

    std::vector<Eigen::Vector3d> points;
    std::vector<double> times;
    points.reserve(sample_count);
    times.reserve(sample_count);
    double planar_length = 0.0;
    for (int index = 0; index < sample_count; ++index)
    {
      const double time = info->duration_ * static_cast<double>(index) /
          static_cast<double>(sample_count - 1);
      const Eigen::Vector3d point =
          info->position_traj_.evaluateDeBoorT(time);
      if (!point.allFinite())
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Local trajectory %d produced a non-finite progress sample",
            info->traj_id_);
        return false;
      }
      if (!points.empty())
        planar_length += (point.head<2>() - points.back().head<2>()).norm();
      points.push_back(point);
      times.push_back(time);
    }

    local_progress_traj_id_ = info->traj_id_;
    if (planar_length <= 1.0e-8)
    {
      // 急停轨迹可能是合法的原地保持 B-spline，没有可投影的平面弧长。
      info->progress_time_ = info->duration_;
      return true;
    }

    try
    {
      local_trajectory_tracker_.reset(
          points,
          times,
          have_odom_ ? odom_pos_ : points.front());
    }
    catch (const std::invalid_argument &error)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Unable to initialize local trajectory progress: %s",
          error.what());
      local_progress_traj_id_ = -1;
      return false;
    }
    return true;
  }

  double SCANReplanFSM::currentLocalTrajectoryTime()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    if (local_progress_traj_id_ != info->traj_id_)
      return 0.0;

    if (local_trajectory_tracker_.active() && have_odom_)
    {
      info->progress_arc_length_ =
          local_trajectory_tracker_.update(odom_pos_);
      info->progress_time_ = local_trajectory_tracker_.progressTime();
    }
    return std::clamp(info->progress_time_, 0.0, info->duration_);
  }

  double SCANReplanFSM::localCollisionScanStartTime(double progress_time) const
  {
    if (!local_trajectory_tracker_.active())
      return std::max(0.0, progress_time);
    const double scan_start_arc = std::max(
        0.0,
        local_trajectory_tracker_.progress() -
            local_progress_collision_backtrack_);
    return local_trajectory_tracker_.timeAt(scan_start_arc);
  }

  bool SCANReplanFSM::localTrajectoryComplete() const
  {
    const LocalTrajData *info = &planner_manager_->local_data_;
    if (!have_odom_ || info->duration_ <= 1.0e-6)
      return false;
    const Eigen::Vector3d endpoint =
        info->position_traj_.evaluateDeBoorT(info->duration_);
    const double endpoint_distance =
        (endpoint.head<2>() - odom_pos_.head<2>()).norm();
    const double remaining = local_trajectory_tracker_.active()
        ? local_trajectory_tracker_.remainingLength()
        : 0.0;
    // 距离与空间进度满足时，控制器可能仍在执行正常制动。必须等实测
    // 平面速度降到阈值后再切状态/清轨迹，避免把舒适减速变成立即急停。
    const double planar_speed = odom_vel_.head<2>().norm();
    return endpoint_distance <= local_finish_distance_ &&
        remaining <= local_finish_remaining_length_ &&
        planar_speed <= local_finish_speed_;
  }

  void SCANReplanFSM::publishPlannerHeartbeat()
  {
    if (!planner_heartbeat_pub_ || !planner_manager_)
      return;

    const LocalTrajData &info = planner_manager_->local_data_;
    const bool state_allows_motion =
        exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ;
    const bool trajectory_valid =
        info.traj_id_ > 0 && std::isfinite(info.duration_) &&
        info.duration_ > 1.0e-6 &&
        std::isfinite(info.progress_time_) && info.progress_time_ >= 0.0 &&
        std::isfinite(info.progress_arc_length_) &&
        info.progress_arc_length_ >= 0.0 &&
        local_progress_traj_id_ == info.traj_id_;

    scan_planner_msgs::msg::PlannerHeartbeat heartbeat;
    heartbeat.header.stamp = node_->now();
    heartbeat.header.frame_id = self_inflation_frame_id_;
    heartbeat.motion_allowed = state_allows_motion && trajectory_valid;
    heartbeat.traj_id = heartbeat.motion_allowed ? info.traj_id_ : -1;
    if (heartbeat.motion_allowed)
    {
      heartbeat.trajectory_start_time = info.start_time_;
      heartbeat.progress_time = std::clamp(
          info.progress_time_, 0.0, info.duration_);
      heartbeat.progress_arc_length = std::max(
          0.0, info.progress_arc_length_);
    }
    else
    {
      heartbeat.trajectory_start_time = heartbeat.header.stamp;
      heartbeat.progress_time = 0.0;
      heartbeat.progress_arc_length = 0.0;
    }
    planner_heartbeat_pub_->publish(heartbeat);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    if (new_state == EMERGENCY_STOP && pre_s != int(EMERGENCY_STOP))
    {
      // 先撤销旧轨迹身份和运动授权。EMERGENCY_STOP 中即使发布原地保持
      // B-spline 也不恢复授权，只有成功生成新的可执行轨迹后才会重新开放。
      local_trajectory_tracker_.clear();
      local_progress_traj_id_ = -1;
    }
    if (new_state == WAIT_TARGET && pre_s != int(WAIT_TARGET))
      publishTrajectoryClear(pos_call);
    else if (pre_s != int(new_state))
    {
      // 状态切换必须立即同步运动授权。尤其进入 GEN_NEW_TRAJ 后优化器可能
      // 在单线程 executor 中运行数百毫秒，不能等 20Hz 定时器才撤销旧授权。
      publishPlannerHeartbeat();
    }
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    if (resetIfRosTimeJumpedBackward())
      return;

    updateLocalTrajTimeFreeze();

    const double odom_source_age =
        (node_->now() - last_odom_source_time_).seconds();
    const double odom_processing_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_odom_callback_time_).count();
    if (have_odom_ &&
        (odom_source_age > odom_timeout_ ||
         odom_processing_age > odom_timeout_))
    {
      if (!odom_timeout_active_)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "body_pose timed out (source age %.3fs, processing age %.3fs, "
            "limit %.2fs); clearing target and trajectory",
            odom_source_age, odom_processing_age, odom_timeout_);
        publishTrajectoryClear("body_pose timeout");
      }
      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
          reference_path_active_ && last_accepted_reference_path_)
      {
        pending_reference_path_ = last_accepted_reference_path_;
      }
      odom_timeout_active_ = true;
      have_odom_ = false;
      trigger_ = false;
      have_target_ = false;
      have_new_target_ = false;
      active_waypoints_.clear();
      reference_path_active_ = false;
      reference_path_tracker_.clear();
      local_reference_seed_.clear();
      local_trajectory_tracker_.clear();
      local_progress_traj_id_ = -1;
      planner_manager_->grid_map_->resetBuffer();
      planner_manager_->grid_map_->resetInputReadiness();
      preset_started_ = false;
      exec_state_ = FSM_EXEC_STATE::INIT;
      return;
    }

    const bool map_input_ready = planner_manager_->grid_map_->inputReady();
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && have_odom_ &&
        reference_path_active_ && have_target_ && !map_input_ready)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Occupancy input timed out; clearing trajectory until a fresh synchronized cloud arrives");
      if (last_accepted_reference_path_)
        pending_reference_path_ = last_accepted_reference_path_;
      publishTrajectoryClear("occupancy input timeout");
      trigger_ = false;
      have_target_ = false;
      have_new_target_ = false;
      reference_path_active_ = false;
      reference_path_tracker_.clear();
      local_reference_seed_.clear();
      local_trajectory_tracker_.clear();
      local_progress_traj_id_ = -1;
      planner_manager_->grid_map_->resetBuffer();
      planner_manager_->grid_map_->resetInputReadiness();
      exec_state_ = FSM_EXEC_STATE::INIT;
      return;
    }

    // The first synchronized cloud is fused by a wall timer after its pose
    // callback. Retry a cached mode-3 route here as soon as the occupancy map
    // becomes ready; otherwise a paused bag or one-shot odometry source could
    // leave the route waiting forever for another odometry message.
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        pending_reference_path_ && have_odom_ && map_input_ready)
    {
      auto pending_path = pending_reference_path_;
      pending_reference_path_.reset();
      pathCallback(pending_path);
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (shouldDelayReplanRetry())
        return;

      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        recordReplanSuccess();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        recordReplanFailure();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (shouldDelayReplanRetry())
        return;

      if (planFromCurrentTraj())
      {
        recordReplanSuccess();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        recordReplanFailure();
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      if (local_progress_traj_id_ != info->traj_id_)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Active trajectory lost its spatial-progress identity; entering emergency stop");
        changeFSMExecState(EMERGENCY_STOP, "PROGRESS");
        return;
      }
      const double t_cur = currentLocalTrajectoryTime();

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        recordReplanFailure();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (localTrajectoryComplete())
      {
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_path_active_)
        {
          if (referencePathComplete())
          {
            reference_path_active_ = false;
            last_accepted_reference_path_.reset();
            reference_path_tracker_.clear();
            have_target_ = false;
            changeFSMExecState(WAIT_TARGET, "REFERENCE_DONE");
          }
          else
          {
            // A local B-spline reaching its end is not the same thing as
            // completing the full reference route.
            changeFSMExecState(GEN_NEW_TRAJ, "REFERENCE_CONTINUE");
          }
          return;
        }

        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          recordReplanFailure();
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      const double seconds_since_plan = have_successful_plan_time_
          ? std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_successful_plan_time_)
                .count()
          : max_replan_period_;
      if (seconds_since_plan < min_replan_period_)
      {
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_ &&
               seconds_since_plan < max_replan_period_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          reference_path_active_ = false;
          last_accepted_reference_path_.reset();
          reference_path_tracker_.clear();
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      replan_retry_pending_ = false;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::shouldDelayReplanRetry()
  {
    if (!replan_retry_pending_)
      return false;
    if (std::chrono::steady_clock::now() < next_replan_retry_time_)
      return true;
    replan_retry_pending_ = false;
    return false;
  }

  void SCANReplanFSM::recordReplanFailure()
  {
    ++replan_fail_count_;
    if (failure_retry_period_ <= 0.0)
    {
      replan_retry_pending_ = false;
      return;
    }
    next_replan_retry_time_ =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(failure_retry_period_));
    replan_retry_pending_ = true;
  }

  void SCANReplanFSM::recordReplanSuccess()
  {
    replan_fail_count_ = 0;
    replan_retry_pending_ = false;
    last_successful_plan_time_ = std::chrono::steady_clock::now();
    have_successful_plan_time_ = true;
  }

  bool SCANReplanFSM::resetIfRosTimeJumpedBackward()
  {
    if (!use_sim_time_ || !node_->get_clock()->ros_time_is_active())
      return false;

    const rclcpp::Time current_ros_time = node_->now();
    if (!have_last_ros_time_)
    {
      if (current_ros_time.nanoseconds() > 0)
      {
        last_ros_time_ = current_ros_time;
        have_last_ros_time_ = true;
      }
      return false;
    }

    constexpr int64_t kBackwardJumpThresholdNanoseconds = 500000000;
    const int64_t backward_jump_nanoseconds =
        last_ros_time_.nanoseconds() - current_ros_time.nanoseconds();
    if (current_ros_time.nanoseconds() > 0)
      last_ros_time_ = current_ros_time;
    else
      have_last_ros_time_ = false;
    if (backward_jump_nanoseconds <= kBackwardJumpThresholdNanoseconds)
      return false;

    RCLCPP_WARN(
        node_->get_logger(),
        "ROS time jumped backward by %.3f s; resetting rosbag planning state",
        static_cast<double>(backward_jump_nanoseconds) * 1.0e-9);

    trigger_ = false;
    have_target_ = false;
    have_new_target_ = false;
    have_odom_ = false;
    preset_started_ = false;
    reference_path_active_ = false;
    reference_path_closed_ = false;
    go2_execution_frozen_ = false;
    odom_timeout_active_ = false;
    need_hover_stop_ = false;
    flag_escape_emergency_ = true;
    exec_state_ = FSM_EXEC_STATE::INIT;
    continuously_called_times_ = 0;
    replan_fail_count_ = 0;
    replan_retry_pending_ = false;
    have_successful_plan_time_ = false;
    current_wp_ = 0;
    active_waypoints_.clear();
    local_reference_seed_.clear();
    last_reference_input_.clear();
    pending_reference_path_.reset();
    last_accepted_reference_path_.reset();
    reference_path_tracker_.clear();
    local_trajectory_tracker_.clear();
    local_progress_traj_id_ = -1;
    last_freeze_update_time_ = current_ros_time;
    last_odom_source_time_ = current_ros_time;
    last_odom_callback_time_ = std::chrono::steady_clock::now();

    if (planner_manager_->grid_map_)
    {
      planner_manager_->grid_map_->resetBuffer();
      planner_manager_->grid_map_->resetInputReadiness();
    }

    LocalTrajData &local_data = planner_manager_->local_data_;
    local_data.start_time_ = current_ros_time;
    local_data.duration_ = 0.0;
    local_data.progress_time_ = 0.0;
    local_data.progress_arc_length_ = 0.0;

    GlobalTrajData &global_data = planner_manager_->global_data_;
    global_data.global_start_time_ = current_ros_time;
    global_data.global_duration_ = 0.0;
    global_data.local_traj_.clear();
    global_data.local_start_time_ = -1.0;
    global_data.local_end_time_ = -1.0;
    global_data.time_increase_ = 0.0;
    global_data.last_time_inc_ = 0.0;
    global_data.last_progress_time_ = 0.0;

    publishTrajectoryClear("ROS time jumped backward");
    return true;
  }

  void SCANReplanFSM::publishTrajectoryClear(const std::string &reason)
  {
    if (!bspline_pub_)
      return;

    scan_planner_msgs::msg::Bspline bspline;
    bspline.header.stamp = node_->now();
    bspline.header.frame_id = self_inflation_frame_id_;
    bspline.order = 0;
    bspline.traj_id = -1;
    bspline.start_time = bspline.header.stamp;
    bspline_pub_->publish(bspline);
    publishLocalPathClear();
    local_trajectory_tracker_.clear();
    local_progress_traj_id_ = -1;
    planner_manager_->local_data_.progress_time_ = 0.0;
    planner_manager_->local_data_.progress_arc_length_ = 0.0;
    publishPlannerHeartbeat();
    RCLCPP_INFO(node_->get_logger(),
                "Published empty B-spline to clear controller trajectory: %s",
                reason.c_str());
  }

  void SCANReplanFSM::publishLocalPath(UniformBspline position_traj)
  {
    if (!publish_local_path_ || !local_path_pub_)
      return;

    const double duration = position_traj.getTimeSum();
    if (!std::isfinite(duration) || duration < 0.0)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Skip local path publication: invalid trajectory duration");
      return;
    }

    UniformBspline velocity_traj = position_traj.getDerivative();
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = self_inflation_frame_id_;

    double last_yaw = getOdomYaw();
    auto append_sample = [&](double sample_time) {
      const Eigen::Vector3d position =
          position_traj.evaluateDeBoorT(sample_time);
      const Eigen::Vector3d velocity =
          velocity_traj.evaluateDeBoorT(sample_time);
      if (!position.allFinite() || !velocity.allFinite())
        return;

      if (velocity.head<2>().squaredNorm() > 1.0e-8)
        last_yaw = std::atan2(velocity.y(), velocity.x());

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = position.x();
      pose.pose.position.y = position.y();
      pose.pose.position.z = position.z();
      pose.pose.orientation.z = std::sin(last_yaw * 0.5);
      pose.pose.orientation.w = std::cos(last_yaw * 0.5);
      path.poses.push_back(pose);
    };

    int sample_count = 0;
    for (double sample_time = 0.0;
         sample_time < duration &&
         sample_count < local_path_max_samples_ - 1;
         sample_time += local_path_sample_dt_, ++sample_count)
    {
      append_sample(sample_time);
    }
    append_sample(duration);

    if (!path.poses.empty())
      local_path_pub_->publish(path);
  }

  void SCANReplanFSM::publishLocalPathClear()
  {
    if (!publish_local_path_ || !local_path_pub_)
      return;

    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = self_inflation_frame_id_;
    local_path_pub_->publish(path);
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    if (local_progress_traj_id_ != info->traj_id_)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Cannot replan from a trajectory without matching spatial progress");
      return false;
    }
    // 重规划起点必须服从机器狗实测状态。原实现取规划轨迹导数；当规划速度
    // 0.75m/s 而真机仅走 0.2m/s 时，会把并不存在的高速状态带进下一条轨迹。
    // body_pose 没有可靠线加速度字段，因此从零加速度开始重新优化。
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        to_goal.norm() > 1e-3 &&
        start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // A reference path is the global route contract. Replacing it with a new
    // current-position-to-end polynomial makes mode 3 cut directly to the
    // final point after its first local replan.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || !reference_path_active_)
    {
      if (!planner_manager_->planGlobalTraj(
              start_pt_,
              start_vel_,
              start_acc_,
              end_pt_,
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero()))
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
        return false;
      }

      if (!adjustGlobalTargetIfOccupied())
        return false;
    }

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        to_goal.norm() > 1e-3 &&
        start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 只允许活动执行/重规划状态检查当前轨迹。INIT、GEN、WAIT、急停窗口
    // 都不得由安全定时器复活旧轨迹或抢占 FSM 状态。
    if (!have_odom_ ||
        (exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ) ||
        local_progress_traj_id_ != info->traj_id_ ||
        info->duration_ <= 1.0e-6)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    const double t_cur = currentLocalTrajectoryTime();

    if (local_trajectory_tracker_.active())
    {
      const double progress_s = local_trajectory_tracker_.progress();
      const Eigen::Vector3d projection =
          local_trajectory_tracker_.pointAt(progress_s);
      const double cross_track_error =
          (projection.head<2>() - odom_pos_.head<2>()).norm();
      if (cross_track_error > local_progress_max_cross_track_)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Local trajectory cross-track error %.3fm exceeds %.3fm; stopping before replanning",
            cross_track_error,
            local_progress_max_cross_track_);
        changeFSMExecState(EMERGENCY_STOP, "CROSS_TRACK");
        return;
      }

      // 与 ClosedLoopController 使用同一个空间前视点，并用更远的 yaw 前视
      // 点估计路径切线。这样碰撞模型同时覆盖 ALIGN 原地转向、TRACK 的
      // kappa=2*y/L^2+heading_yaw/v，以及 SDK 线角速度死区造成的极值轨迹。
      const double recovery_target_s = std::min(
          local_trajectory_tracker_.totalLength(),
          progress_s + local_recovery_lookahead_);
      const double recovery_yaw_target_s = std::min(
          local_trajectory_tracker_.totalLength(),
          progress_s + local_recovery_yaw_lookahead_);
      const Eigen::Vector3d recovery_target =
          local_trajectory_tracker_.pointAt(recovery_target_s);
      const double tangent_back_s = std::max(0.0, progress_s - 0.15);
      Eigen::Vector2d path_direction =
          local_trajectory_tracker_.pointAt(recovery_yaw_target_s).head<2>() -
          local_trajectory_tracker_.pointAt(tangent_back_s).head<2>();
      const Eigen::Vector2d to_recovery_target =
          recovery_target.head<2>() - odom_pos_.head<2>();
      if (path_direction.squaredNorm() < 1.0e-4)
        path_direction = to_recovery_target;

      const double odom_yaw = getOdomYaw();
      const double desired_yaw = path_direction.squaredNorm() < 1.0e-4
          ? odom_yaw
          : std::atan2(path_direction.y(), path_direction.x());
      RecoverySweepControlLimits recovery_control =
          local_recovery_control_limits_;
      recovery_control.heading_error = normalizeRecoveryAngle(
          desired_yaw - odom_yaw);
      const std::vector<RecoverySweepPose> recovery_sweep =
          samplePurePursuitRecoverySweep(
              odom_pos_,
              odom_yaw,
              recovery_target,
              local_recovery_sample_distance_,
              recovery_control);
      if (recovery_sweep.empty())
      {
        // 空数组表示输入非有限、采样维度溢出或超过硬样本预算。安全检查
        // 不能在这种情况下静默放行，必须撤销旧轨迹授权并原地停车。
        RCLCPP_ERROR(
            node_->get_logger(),
            "Unable to build bounded Pure Pursuit swept region; stopping safely");
        changeFSMExecState(EMERGENCY_STOP, "RECOVERY_SWEEP_INVALID");
        return;
      }
      for (const RecoverySweepPose &sample : recovery_sweep)
      {
        if (map->getInflateOccupancy(sample.position, sample.yaw))
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "Pure Pursuit %s swept region is occupied; stopping before replanning",
              cross_track_error > local_recovery_check_min_cross_track_
                  ? "off-path recovery"
                  : "tracking");
          changeFSMExecState(EMERGENCY_STOP, "RECOVERY_COLLISION");
          return;
        }
      }
    }

    const double scan_start_t = localCollisionScanStartTime(t_cur);
    const double t_2_3 = info->duration_ * 2 / 3;
    const double scan_end_t = t_cur < t_2_3 ? t_2_3 : info->duration_;
    for (double t = scan_start_t; ; t += time_step)
    {
      const double sample_t = std::min(t, scan_end_t);
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(sample_t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(
          std::min(sample_t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        const double collision_time_ahead =
            std::max(0.0, sample_t - t_cur);
        if (collision_time_ahead < emergency_time_)
        {
          // 近场/已落入回退余量的障碍必须先撤销旧轨迹授权。不能先同步跑
          // 优化器再决定停车，否则规划耗时内机器狗仍沿碰撞轨迹前进。
          RCLCPP_WARN(
              node_->get_logger(),
              "Obstacle discovered; emergency stop in %.3fs of trajectory time",
              collision_time_ahead);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          return;
        }

        if (shouldDelayReplanRetry())
          return;

        if (planFromCurrentTraj()) // 仅远场障碍允许边执行边重规划。
        {
          recordReplanSuccess();
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }

        // 远场重规划失败，进入 REPLAN；若优化耗时超过 heartbeat timeout，
        // 控制器会独立清零，绝不继续无限执行最后一条轨迹。
        recordReplanFailure();
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        return;
      }
      // 明确采样分区端点；原来的 t<scan_end_t 会漏掉精确终点占用。
      if (sample_t >= scan_end_t - 1.0e-9)
        break;
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(
            start_pt_,
            start_vel_,
            start_acc_,
            local_target_pt_,
            local_target_vel_,
            (have_new_target_ || flag_use_poly_init),
            flag_randomPolyTraj,
            local_reference_seed_);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;
      if (!resetLocalTrajectoryProgress())
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Rejecting newly planned trajectory %d: unable to initialize spatial progress",
            info->traj_id_);
        publishTrajectoryClear("local progress initialization failed");
        return false;
      }

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.header.stamp = info->start_time_;
      bspline.header.frame_id = self_inflation_frame_id_;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);
      publishLocalPath(info->position_traj_);
      publishPlannerHeartbeat();

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;
    if (!resetLocalTrajectoryProgress())
    {
      publishTrajectoryClear("emergency trajectory progress initialization failed");
      return false;
    }

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.header.stamp = info->start_time_;
    bspline.header.frame_id = self_inflation_frame_id_;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);
    publishLocalPath(info->position_traj_);
    publishPlannerHeartbeat();

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    local_reference_seed_.clear();
    const double t_step = std::max(
        0.05,
        planning_horizon_ / 20.0 /
            std::max(0.05, planner_manager_->pp_.max_vel_));
    double dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    double target_arc_length = 0.0;
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_path_active_)
    {
      const double progress = updateReferencePathProgress();
      const double total_length = reference_path_tracker_.totalLength();
      const double remaining = reference_path_tracker_.remainingLength();
      target_arc_length =
          std::min(total_length, progress + planning_horizon_);

      // A short closed route can fit entirely inside the planning horizon.
      // Its terminal point is also the current start, which would otherwise
      // produce a zero-length local target before the robot leaves the start.
      const bool terminal_is_current_start =
          reference_path_closed_ &&
          !reference_path_tracker_.departedStart() &&
          target_arc_length >= total_length - 1.0e-6 &&
          (reference_path_tracker_.pointAt(total_length).head<2>() -
           odom_pos_.head<2>()).norm() <= reference_finish_distance_ &&
          remaining > reference_finish_remaining_length_;
      if (terminal_is_current_start)
      {
        const double guarded_advance = std::min(
            planning_horizon_,
            std::max(reference_path_min_point_spacing_, 0.5 * remaining));
        target_arc_length = std::min(
            total_length - reference_path_min_point_spacing_,
            progress + guarded_advance);
      }

      local_target_pt_ =
          reference_path_tracker_.pointAt(target_arc_length);
      constexpr double minimum_local_target_distance = 0.25;
      if ((local_target_pt_.head<2>() - start_pt_.head<2>()).norm() <
              minimum_local_target_distance &&
          remaining > reference_finish_remaining_length_)
      {
        const double arc_step =
            std::max(reference_path_min_point_spacing_, 0.05);
        bool separated_target_found = false;
        for (double offset = arc_step;
             target_arc_length - offset >
                 progress + reference_path_min_point_spacing_;
             offset += arc_step)
        {
          const double candidate_arc = target_arc_length - offset;
          const Eigen::Vector3d candidate =
              reference_path_tracker_.pointAt(candidate_arc);
          if ((candidate.head<2>() - start_pt_.head<2>()).norm() >=
              minimum_local_target_distance)
          {
            target_arc_length = candidate_arc;
            local_target_pt_ = candidate;
            separated_target_found = true;
            break;
          }
        }
        if (!separated_target_found)
        {
          for (double candidate_arc = target_arc_length + arc_step;
               candidate_arc <=
                   reference_path_tracker_.totalLength() + 1.0e-9;
               candidate_arc += arc_step)
          {
            const Eigen::Vector3d candidate =
                reference_path_tracker_.pointAt(candidate_arc);
            if ((candidate.head<2>() - start_pt_.head<2>()).norm() >=
                minimum_local_target_distance)
            {
              target_arc_length = candidate_arc;
              local_target_pt_ = candidate;
              break;
            }
          }
        }
      }
      reference_local_target_arc_length_ = target_arc_length;
    }
    else
    {
      double t;
      double dist_min = 9999.0;
      for (t = planner_manager_->global_data_.last_progress_time_;
           t < planner_manager_->global_data_.global_duration_;
           t += t_step)
      {
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
        double dist = (pos_t - start_pt_).norm();

        if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 &&
            dist > planning_horizon_)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                       dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
          local_target_pt_ = pos_t;
          target_t = t;
          planner_manager_->global_data_.last_progress_time_ = t;
          break;
        }
        if (dist < dist_min)
        {
          dist_min = dist;
          dist_min_t = t;
        }
        if (dist >= planning_horizon_)
        {
          local_target_pt_ = pos_t;
          target_t = t;
          planner_manager_->global_data_.last_progress_time_ = dist_min_t;
          break;
        }
      }
      if (t > planner_manager_->global_data_.global_duration_) // Last global point
      {
        local_target_pt_ = end_pt_;
        target_t = planner_manager_->global_data_.global_duration_;
      }
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        reference_path_active_)
    {
      const double progress = reference_path_tracker_.progress();
      const double total_length = reference_path_tracker_.totalLength();
      const double arc_step =
          std::max(reference_path_min_point_spacing_,
                   planning_horizon_ / 20.0);

      if (targetOccupancy(local_target_pt_) != 0)
      {
        bool found_free_target = false;
        double adjusted_arc = target_arc_length;
        for (double offset = 0.0;
             offset <= total_length + 1.0e-9;
             offset += arc_step)
        {
          const double forward_arc = target_arc_length + offset;
          if (forward_arc <= total_length)
          {
            const Eigen::Vector3d point =
                reference_path_tracker_.pointAt(forward_arc);
            if (targetOccupancy(point) == 0)
            {
              local_target_pt_ = point;
              adjusted_arc = forward_arc;
              found_free_target = true;
              break;
            }
          }

          const double backward_arc = target_arc_length - offset;
          if (backward_arc >= progress)
          {
            const Eigen::Vector3d point =
                reference_path_tracker_.pointAt(backward_arc);
            if (targetOccupancy(point) == 0)
            {
              local_target_pt_ = point;
              adjusted_arc = backward_arc;
              found_free_target = true;
              break;
            }
          }
        }

        if (found_free_target)
        {
          target_arc_length = adjusted_arc;
          reference_local_target_arc_length_ = adjusted_arc;
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Reference local target was adjusted to a nearby collision-free point");
        }
        else
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Reference local target is occupied and no forward-route target is free");
        }
      }

      local_reference_seed_.clear();
      local_reference_seed_.push_back(start_pt_);
      const Eigen::Vector3d seed_start =
          reference_path_tracker_.pointAt(progress);
      if ((seed_start - local_reference_seed_.back()).norm() >=
          reference_path_min_point_spacing_)
      {
        local_reference_seed_.push_back(seed_start);
      }

      for (double arc = progress + arc_step;
           arc < target_arc_length - 1.0e-6;
           arc += arc_step)
      {
        const Eigen::Vector3d point =
            reference_path_tracker_.pointAt(arc);
        if ((point - local_reference_seed_.back()).norm() >=
            reference_path_min_point_spacing_)
        {
          local_reference_seed_.push_back(point);
        }
      }

      if ((local_target_pt_ - local_reference_seed_.back()).norm() >=
          reference_path_min_point_spacing_)
      {
        local_reference_seed_.push_back(local_target_pt_);
      }
      else
      {
        local_reference_seed_.back() = local_target_pt_;
      }
    }
    else if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0;
           dt <= planner_manager_->global_data_.global_duration_;
           dt += t_step)
      {
        const double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          const Eigen::Vector3d point =
              planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(point) == 0)
          {
            local_target_pt_ = point;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        const double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          const Eigen::Vector3d point =
              planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(point) == 0)
          {
            local_target_pt_ = point;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Local target is in collision and no nearby free target was found");
      }
    }

    const double stopping_distance =
        planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_ /
        (2.0 * planner_manager_->pp_.max_acc_);
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        reference_path_active_)
    {
      const double target_remaining =
          std::max(
              0.0,
              reference_path_tracker_.totalLength() -
                  reference_local_target_arc_length_);
      if (target_remaining < stopping_distance)
      {
        local_target_vel_.setZero();
      }
      else
      {
        local_target_vel_ =
            reference_path_tracker_.tangentAt(
                reference_local_target_arc_length_) *
            planner_manager_->pp_.max_vel_;
      }
    }
    else if ((end_pt_ - local_target_pt_).norm() < stopping_distance)
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
    }
  }

} // namespace scan_planner
