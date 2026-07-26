// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file scan_replan_fsm.cpp @brief Implements replanning modes and safety-state transitions. */

#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <stdexcept>
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
    last_odom_receive_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
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
    reference_progress_max_advance_ =
        std::max(reference_path_min_point_spacing_,
                 load_parameter<double>(node_, "fsm.reference_progress_max_advance", 2.0));
    reference_finish_distance_ =
        std::max(reference_path_min_point_spacing_,
                 load_parameter<double>(node_, "fsm.reference_finish_distance", 0.30));
    reference_finish_remaining_length_ =
        std::max(reference_path_min_point_spacing_,
                 load_parameter<double>(node_, "fsm.reference_finish_remaining_length", 0.30));
    odom_timeout_ = std::max(0.05, load_parameter<double>(node_, "fsm.odom_timeout", 0.3));
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
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", rclcpp::QoS(20).reliable().transient_local());
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

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

    end_pt_ = waypoints.back();

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
      std::vector<Eigen::Vector3d> &output)
  {
    output.clear();
    reference_path_active_ = false;
    reference_path_closed_ = false;
    reference_path_total_length_ = 0.0;

    if (input.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference path contains no points");
      return false;
    }

    reference_path_closed_ =
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
      reference_path_total_length_ += (point - previous).norm();
      previous = point;
    }

    if (reference_path_total_length_ < reference_path_min_length_)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Reference path is too short after cleaning: %.3f m (minimum %.3f m)",
          reference_path_total_length_,
          reference_path_min_length_);
      output.clear();
      return false;
    }

    if (reference_path_closed_ && output.size() < 3)
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
        reference_path_closed_ ? "closed" : "open",
        output.size(),
        reference_path_total_length_);
    return true;
  }

  double SCANReplanFSM::referenceProgressSampleStep() const
  {
    const double max_velocity =
        std::max(0.05, planner_manager_ ? planner_manager_->pp_.max_vel_ : 0.3);
    return std::clamp(
        reference_path_min_point_spacing_ / max_velocity,
        0.05,
        0.5);
  }

  double SCANReplanFSM::updateReferencePathProgress()
  {
    if (!reference_path_active_ || !planner_manager_)
      return 0.0;

    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!std::isfinite(duration) || duration <= 1.0e-6)
      return 0.0;

    const double start_t =
        std::clamp(global_data.last_progress_time_, 0.0, duration);
    const double sample_dt = referenceProgressSampleStep();
    double best_t = start_t;
    double best_distance =
        (global_data.getPosition(start_t).head<2>() - odom_pos_.head<2>()).squaredNorm();
    double travelled = 0.0;
    Eigen::Vector3d previous = global_data.getPosition(start_t);

    double t = start_t;
    while (t < duration - 1.0e-6)
    {
      const double sample_t = std::min(t + sample_dt, duration);
      const Eigen::Vector3d point = global_data.getPosition(sample_t);
      travelled += (point - previous).norm();
      previous = point;

      if (travelled > reference_progress_max_advance_)
        break;

      const double distance =
          (point.head<2>() - odom_pos_.head<2>()).squaredNorm();
      if (distance < best_distance)
      {
        best_distance = distance;
        best_t = sample_t;
      }
      t = sample_t;
    }

    global_data.last_progress_time_ =
        std::max(global_data.last_progress_time_, best_t);
    return global_data.last_progress_time_;
  }

  double SCANReplanFSM::referencePathRemainingLength()
  {
    if (!reference_path_active_ || !planner_manager_)
      return 0.0;

    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!std::isfinite(duration) || duration <= 1.0e-6)
      return std::max(reference_path_total_length_,
                      reference_finish_remaining_length_ + 1.0);
    const double start_t =
        std::clamp(global_data.last_progress_time_, 0.0, duration);
    const double sample_dt = referenceProgressSampleStep();
    double remaining = 0.0;
    Eigen::Vector3d previous = global_data.getPosition(start_t);

    double t = start_t;
    while (t < duration - 1.0e-6)
    {
      const double sample_t = std::min(t + sample_dt, duration);
      const Eigen::Vector3d point = global_data.getPosition(sample_t);
      remaining += (point - previous).norm();
      previous = point;
      t = sample_t;
    }
    return remaining;
  }

  bool SCANReplanFSM::referencePathComplete()
  {
    if (!reference_path_active_)
      return false;

    updateReferencePathProgress();
    const double remaining = referencePathRemainingLength();
    const double distance_to_end =
        (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
    const bool complete =
        remaining <= reference_finish_remaining_length_ &&
        distance_to_end <= reference_finish_distance_;

    if (complete)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "%s reference path completed: remaining %.3f m, end distance %.3f m",
          reference_path_closed_ ? "Closed" : "Open",
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
      RCLCPP_ERROR(
          node_->get_logger(),
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

    if (!have_odom_)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Received reference path before a valid body_pose; caching it");
      pending_reference_path_ = msg;
      return;
    }

    trigger_ = true;

    std::vector<Eigen::Vector3d> raw_waypoints;
    raw_waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      geometry_msgs::msg::PoseStamped transformed_pose;
      geometry_msgs::msg::PoseStamped input_pose = pose_stamped;
      if (input_pose.header.frame_id.empty())
        input_pose.header.frame_id = msg->header.frame_id;
      if (!transformPoseToGoalFrame(input_pose, transformed_pose))
        return;

      Eigen::Vector3d wp;
      wp(0) = transformed_pose.pose.position.x;
      wp(1) = transformed_pose.pose.position.y;
      wp(2) = transformed_pose.pose.position.z + reference_path_height_offset_;
      raw_waypoints.push_back(wp);
    }

    std::vector<Eigen::Vector3d> waypoints;
    if (!prepareReferenceWaypoints(raw_waypoints, waypoints))
    {
      have_target_ = false;
      return;
    }

    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
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
      reference_path_active_ = false;
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
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
    last_odom_receive_time_ = node_->now();
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
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
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
    if (new_state == WAIT_TARGET && pre_s != int(WAIT_TARGET))
      publishTrajectoryClear(pos_call);
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
    updateLocalTrajTimeFreeze();

    if (have_odom_ &&
        (node_->now() - last_odom_receive_time_).seconds() > odom_timeout_)
    {
      if (!odom_timeout_active_)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "body_pose timed out (limit %.2fs); clearing target and trajectory",
            odom_timeout_);
        publishTrajectoryClear("body_pose timeout");
      }
      odom_timeout_active_ = true;
      have_odom_ = false;
      trigger_ = false;
      have_target_ = false;
      have_new_target_ = false;
      active_waypoints_.clear();
      reference_path_active_ = false;
      preset_started_ = false;
      exec_state_ = FSM_EXEC_STATE::INIT;
      return;
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

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

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
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_path_active_)
        {
          if (referencePathComplete())
          {
            reference_path_active_ = false;
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
          replan_fail_count_++;
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
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
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
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
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
    RCLCPP_INFO(node_->get_logger(),
                "Published empty B-spline to clear controller trajectory: %s",
                reason.c_str());
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

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

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

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

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
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

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

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
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_path_active_)
    {
      const double progress_t = updateReferencePathProgress();
      dist_min_t = progress_t;
      double accumulated_length = 0.0;
      Eigen::Vector3d previous =
          planner_manager_->global_data_.getPosition(progress_t);
      bool target_selected = false;
      local_reference_seed_.push_back(start_pt_);
      if ((previous - local_reference_seed_.back()).norm() >=
          reference_path_min_point_spacing_)
      {
        local_reference_seed_.push_back(previous);
      }

      for (double t = progress_t + t_step;
           t <= planner_manager_->global_data_.global_duration_ + 1.0e-6;
           t += t_step)
      {
        const double sample_t =
            std::min(t, planner_manager_->global_data_.global_duration_);
        const Eigen::Vector3d point =
            planner_manager_->global_data_.getPosition(sample_t);
        accumulated_length += (point - previous).norm();
        previous = point;
        if ((point - local_reference_seed_.back()).norm() >=
            reference_path_min_point_spacing_)
        {
          local_reference_seed_.push_back(point);
        }

        if (accumulated_length >= planning_horizon_)
        {
          local_target_pt_ = point;
          target_t = sample_t;
          target_selected = true;
          break;
        }
        if (sample_t >= planner_manager_->global_data_.global_duration_)
          break;
      }

      if (!target_selected)
      {
        local_target_pt_ = end_pt_;
        target_t = planner_manager_->global_data_.global_duration_;
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

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_path_active_)
    {
      // The occupancy check above may move the local target forward or
      // backward along the global trajectory. Rebuild the seed to that exact
      // target so it never contains an overshoot followed by a backward leg.
      local_reference_seed_.clear();
      local_reference_seed_.push_back(start_pt_);
      const double seed_start_t =
          std::clamp(dist_min_t, 0.0, planner_manager_->global_data_.global_duration_);
      const double seed_end_t =
          std::clamp(target_t, seed_start_t, planner_manager_->global_data_.global_duration_);
      const Eigen::Vector3d seed_start =
          planner_manager_->global_data_.getPosition(seed_start_t);
      if ((seed_start - local_reference_seed_.back()).norm() >=
          reference_path_min_point_spacing_)
      {
        local_reference_seed_.push_back(seed_start);
      }

      for (double t = seed_start_t + t_step;
           t < seed_end_t - 1.0e-6;
           t += t_step)
      {
        const Eigen::Vector3d point =
            planner_manager_->global_data_.getPosition(t);
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

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
