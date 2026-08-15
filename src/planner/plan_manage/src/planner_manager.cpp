// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file planner_manager.cpp @brief Implements global and local trajectory planning. */

// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace scan_planner
{
  namespace
  {
    constexpr int kMaxSamplingAttempts = 30;
    constexpr int kMaxRegenerationAttempts = 3;
    // PolynomialTraj::minSnapTraj builds dense 6N x 6N matrices. Keep the
    // global guide polynomial bounded; mode 3 still tracks the complete raw
    // reference polyline in ReferencePathTracker for local target selection.
    constexpr size_t kMaxGlobalPolynomialPoints = 32;

    bool allPointsFinite(const std::vector<Eigen::Vector3d> &points)
    {
      return std::all_of(
          points.begin(), points.end(),
          [](const Eigen::Vector3d &point) { return point.allFinite(); });
    }

    bool initializeAndValidatePolynomialTrajectory(
        PolynomialTraj &trajectory, double &duration)
    {
      const std::vector<double> segment_times = trajectory.getTimes();
      if (segment_times.empty() ||
          !std::all_of(
              segment_times.begin(), segment_times.end(),
              [](const double segment_time) {
                return std::isfinite(segment_time) && segment_time > 1.0e-6;
              }))
      {
        return false;
      }

      trajectory.init();
      duration = trajectory.getTimeSum();
      return std::isfinite(duration) && duration > 1.0e-6 &&
             trajectory.evaluate(0.0).allFinite() &&
             trajectory.evaluate(duration).allFinite();
    }

    bool resamplePolylineByArcLength(
        const std::vector<Eigen::Vector3d> &points,
        const double target_spacing,
        const double min_spacing,
        const double max_spacing,
        std::vector<Eigen::Vector3d> &resampled_points,
        double &actual_spacing)
    {
      resampled_points.clear();
      actual_spacing = 0.0;
      if (points.size() < 2 || !allPointsFinite(points))
        return false;

      std::vector<double> accumulated_length(points.size(), 0.0);
      for (size_t index = 1; index < points.size(); ++index)
      {
        const double segment_length =
            (points[index] - points[index - 1]).norm();
        if (!std::isfinite(segment_length) || segment_length <= 1.0e-6)
          return false;
        accumulated_length[index] =
            accumulated_length[index - 1] + segment_length;
      }

      const double total_length = accumulated_length.back();
      if (!std::isfinite(total_length) || total_length <= 1.0e-6)
        return false;

      constexpr size_t kMaxIntervalCount =
          kMaxGlobalPolynomialPoints - 1;
      const auto boundedIntervalCount = [](const double value) {
        return static_cast<size_t>(std::clamp(
            value,
            1.0,
            static_cast<double>(kMaxGlobalPolynomialPoints - 1)));
      };
      const size_t desired_interval_count = boundedIntervalCount(
          std::round(total_length / target_spacing));
      const size_t minimum_interval_count = boundedIntervalCount(
          std::ceil(total_length / max_spacing));
      const size_t maximum_interval_count = boundedIntervalCount(
          std::floor(total_length / min_spacing));

      size_t interval_count = desired_interval_count;
      if (minimum_interval_count <= maximum_interval_count)
      {
        interval_count = std::clamp(
            desired_interval_count,
            minimum_interval_count,
            maximum_interval_count);
      }
      interval_count = std::min(interval_count, kMaxIntervalCount);

      actual_spacing = total_length / static_cast<double>(interval_count);
      resampled_points.reserve(interval_count + 1);
      resampled_points.push_back(points.front());

      size_t segment_index = 0;
      for (size_t sample_index = 1;
           sample_index < interval_count;
           ++sample_index)
      {
        const double sample_distance =
            actual_spacing * static_cast<double>(sample_index);
        while (segment_index + 1 < accumulated_length.size() - 1 &&
               accumulated_length[segment_index + 1] < sample_distance)
        {
          ++segment_index;
        }

        const double segment_start_distance =
            accumulated_length[segment_index];
        const double segment_end_distance =
            accumulated_length[segment_index + 1];
        const double segment_length =
            segment_end_distance - segment_start_distance;
        if (!std::isfinite(segment_length) || segment_length <= 1.0e-6)
          return false;

        const double ratio = std::clamp(
            (sample_distance - segment_start_distance) / segment_length,
            0.0,
            1.0);
        resampled_points.push_back(
            points[segment_index] * (1.0 - ratio) +
            points[segment_index + 1] * ratio);
      }

      resampled_points.push_back(points.back());
      return allPointsFinite(resampled_points);
    }

    /**
     * @brief Replaces sample heights with linear interpolation along planar arc length.
     * @param[in,out] points Ordered trajectory samples.
     * @param start_z Height assigned at the first sample.
     * @param target_z Height assigned at the final sample.
     */
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    pp_.global_path_resample_spacing_ =
        get_double("global_path.resample_spacing", 0.20);
    pp_.global_path_min_spacing_ =
        get_double("global_path.min_spacing", 0.16);
    pp_.global_path_max_spacing_ =
        get_double("global_path.max_spacing", 0.24);
    if (!std::isfinite(pp_.global_path_resample_spacing_) ||
        !std::isfinite(pp_.global_path_min_spacing_) ||
        !std::isfinite(pp_.global_path_max_spacing_) ||
        pp_.global_path_min_spacing_ <= 0.0 ||
        pp_.global_path_max_spacing_ < pp_.global_path_min_spacing_ ||
        pp_.global_path_resample_spacing_ < pp_.global_path_min_spacing_ ||
        pp_.global_path_resample_spacing_ > pp_.global_path_max_spacing_)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Invalid global path resampling parameters; using spacing=0.20, "
          "min=0.16, max=0.24 m");
      pp_.global_path_resample_spacing_ = 0.20;
      pp_.global_path_min_spacing_ = 0.16;
      pp_.global_path_max_spacing_ = 0.24;
    }

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit,
                                        bool flag_randomPolyTraj,
                                        const std::vector<Eigen::Vector3d> &reference_seed)
  {

    if (!start_pt.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
        !local_target_pt.allFinite() || !local_target_vel.allFinite())
    {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting replan request containing a non-finite state or target");
      continuous_failures_count_++;
      return false;
    }
    if (!std::isfinite(pp_.max_vel_) || pp_.max_vel_ <= 0.0 ||
        !std::isfinite(pp_.max_acc_) || pp_.max_acc_ <= 0.0 ||
        !std::isfinite(pp_.ctrl_pt_dist) || pp_.ctrl_pt_dist <= 0.0)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid planner limits: max_vel=%.3f max_acc=%.3f ctrl_pt_dist=%.3f",
                   pp_.max_vel_, pp_.max_acc_, pp_.ctrl_pt_dist);
      continuous_failures_count_++;
      return false;
    }

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    if (!std::isfinite(ts) || ts <= 1.0e-6)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid initial trajectory sampling interval: %.6f", ts);
      continuous_failures_count_++;
      return false;
    }
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    bool used_reference_seed = false;
    int regeneration_attempts = 0;
    do
    {
      if (++regeneration_attempts > kMaxRegenerationAttempts)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Initial trajectory regeneration exceeded %d attempts",
                     kMaxRegenerationAttempts);
        continuous_failures_count_++;
        return false;
      }
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;
      used_reference_seed = false;

      if (!reference_seed.empty() && flag_polyInit && !flag_randomPolyTraj)
      {
        std::vector<Eigen::Vector3d> seed;
        seed.reserve(reference_seed.size() + 2);
        seed.push_back(start_pt);
        for (const auto &point : reference_seed)
        {
          if (point.allFinite() && (point - seed.back()).norm() > 1.0e-4)
            seed.push_back(point);
        }
        if ((local_target_pt - seed.back()).norm() > 1.0e-4)
          seed.push_back(local_target_pt);
        else
          seed.back() = local_target_pt;

        std::vector<double> cumulative_length(seed.size(), 0.0);
        for (size_t i = 1; i < seed.size(); ++i)
        {
          cumulative_length[i] =
              cumulative_length[i - 1] + (seed[i] - seed[i - 1]).norm();
        }
        const double total_length = cumulative_length.back();
        if (seed.size() < 2 || !std::isfinite(total_length) || total_length < 0.2)
        {
          RCLCPP_ERROR(node_->get_logger(), "Reference seed is too short for local replanning");
          continuous_failures_count_++;
          return false;
        }

        const double desired_spacing =
            std::max(0.02, std::min(pp_.ctrl_pt_dist, total_length / 6.0));
        const int segment_count =
            std::max(6, static_cast<int>(std::ceil(total_length / desired_spacing)));
        point_set.reserve(segment_count + 1);
        size_t segment_index = 0;
        for (int i = 0; i <= segment_count; ++i)
        {
          const double target_length =
              total_length * static_cast<double>(i) / segment_count;
          while (segment_index + 1 < cumulative_length.size() &&
                 cumulative_length[segment_index + 1] < target_length)
          {
            ++segment_index;
          }

          if (segment_index + 1 >= seed.size())
          {
            point_set.push_back(seed.back());
            continue;
          }

          const double segment_length =
              cumulative_length[segment_index + 1] -
              cumulative_length[segment_index];
          const double ratio = segment_length > 1.0e-9
              ? (target_length - cumulative_length[segment_index]) / segment_length
              : 0.0;
          point_set.push_back(
              seed[segment_index] * (1.0 - ratio) +
              seed[segment_index + 1] * ratio);
        }
        point_set.front() = start_pt;
        point_set.back() = local_target_pt;
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        used_reference_seed = true;
        flag_first_call = false;
        flag_force_polynomial = false;
      }
      else if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
        if (!std::isfinite(dist) || !std::isfinite(time) || time <= 1.0e-6)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Invalid polynomial initialization: distance=%.6f time=%.6f",
                       dist, time);
          continuous_failures_count_++;
          return false;
        }

        const Eigen::Vector3d lateral =
            (start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1));
        if (!flag_randomPolyTraj || lateral.squaredNorm() < 1.0e-10)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = lateral.normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t = 0.0;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        int sampling_attempts = 0;
        do
        {
          if (++sampling_attempts > kMaxSamplingAttempts)
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "Polynomial sampling exceeded %d attempts",
                         kMaxSamplingAttempts);
            continuous_failures_count_++;
            return false;
          }
          ts /= 1.5;
          if (!std::isfinite(ts) || ts <= 1.0e-6)
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "Polynomial sampling interval became invalid: %.9f", ts);
            continuous_failures_count_++;
            return false;
          }
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          if (!last_pt.allFinite())
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "Polynomial trajectory starts with a non-finite point");
            continuous_failures_count_++;
            return false;
          }
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if (!pt.allFinite())
            {
              RCLCPP_ERROR(node_->get_logger(),
                           "Polynomial trajectory returned a non-finite point at t=%.6f", t);
              continuous_failures_count_++;
              return false;
            }
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        const Eigen::Vector3d initial_vel = gl_traj.evaluateVel(0);
        const Eigen::Vector3d initial_acc = gl_traj.evaluateAcc(0);
        const Eigen::Vector3d final_acc = gl_traj.evaluateAcc(t);
        if (!initial_vel.allFinite() || !initial_acc.allFinite() || !final_acc.allFinite())
        {
          RCLCPP_ERROR(node_->get_logger(), "Polynomial trajectory derivatives are not finite");
          continuous_failures_count_++;
          return false;
        }
        start_end_derivatives.push_back(initial_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(initial_acc);
        start_end_derivatives.push_back(final_acc);
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        // 局部轨迹的执行进度由 FSM 根据真实 odom 空间投影更新，禁止在
        // 规划器内部重新使用墙钟时间，否则机器人落后时会跳过未执行路径。
        if (!std::isfinite(local_data_.progress_time_) ||
            !std::isfinite(local_data_.duration_) || local_data_.duration_ < 0.0)
        {
          RCLCPP_ERROR(node_->get_logger(), "Invalid previous trajectory time state");
          continuous_failures_count_++;
          return false;
        }
        double t_cur = std::clamp(
            local_data_.progress_time_, 0.0, local_data_.duration_);

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          const Eigen::Vector3d point = local_data_.position_traj_.evaluateDeBoorT(t);
          if (!point.allFinite())
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "Previous trajectory returned a non-finite point at t=%.6f", t);
            continuous_failures_count_++;
            return false;
          }
          segment_point.push_back(point);
          if (t > t_cur)
          {
            const double segment_length =
                (segment_point.back() - segment_point[segment_point.size() - 2]).norm();
            if (!std::isfinite(segment_length))
            {
              RCLCPP_ERROR(node_->get_logger(),
                           "Previous trajectory has a non-finite arc length");
              continuous_failures_count_++;
              return false;
            }
            pseudo_arc_length.push_back(segment_length + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (!std::isfinite(poly_time))
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Previous trajectory produced a non-finite connection time");
          continuous_failures_count_++;
          return false;
        }
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              if (!segment_point.back().allFinite())
              {
                RCLCPP_ERROR(node_->get_logger(),
                             "Connection trajectory returned a non-finite point");
                continuous_failures_count_++;
                return false;
              }
              const double segment_length =
                  (segment_point.back() - segment_point[segment_point.size() - 2]).norm();
              if (!std::isfinite(segment_length))
              {
                RCLCPP_ERROR(node_->get_logger(),
                             "Connection trajectory has a non-finite arc length");
                continuous_failures_count_++;
                return false;
              }
              pseudo_arc_length.push_back(segment_length + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        if (segment_point.size() < 2 ||
            pseudo_arc_length.size() != segment_point.size() ||
            !std::isfinite(pseudo_arc_length.back()) ||
            pseudo_arc_length.back() <= 1.0e-6)
        {
          // 旧轨迹已经到末端或剩余段退化时，下面的 size()-2 会下溢，
          // 零弧长还会让“至少 7 点”循环永不结束。强制回退到多项式初始化。
          RCLCPP_WARN(
              node_->get_logger(),
              "Previous-trajectory seed is degenerate; falling back to polynomial initialization");
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        int sampling_attempts = 0;
        do
        {
          if (++sampling_attempts > kMaxSamplingAttempts)
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "Previous trajectory resampling exceeded %d attempts",
                         kMaxSamplingAttempts);
            continuous_failures_count_++;
            return false;
          }
          cps_dist /= 1.5;
          if (!std::isfinite(cps_dist) || cps_dist <= 1.0e-6)
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "Previous trajectory sampling distance became invalid: %.9f",
                         cps_dist);
            continuous_failures_count_++;
            return false;
          }
          point_set.clear();
          sample_length = 0;
          id = 0;
          while (id + 1 < pseudo_arc_length.size() &&
                 sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              const double denominator =
                  pseudo_arc_length[id + 1] - pseudo_arc_length[id];
              if (denominator <= 1.0e-9)
              {
                ++id;
                continue;
              }
              point_set.push_back(
                  (sample_length - pseudo_arc_length[id]) / denominator * segment_point[id + 1] +
                  (pseudo_arc_length[id + 1] - sample_length) / denominator * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        const Eigen::Vector3d initial_vel =
            local_data_.velocity_traj_.evaluateDeBoorT(t_cur);
        const Eigen::Vector3d initial_acc =
            local_data_.acceleration_traj_.evaluateDeBoorT(t_cur);
        if (!initial_vel.allFinite() || !initial_acc.allFinite())
        {
          RCLCPP_ERROR(node_->get_logger(), "Previous trajectory derivatives are not finite");
          continuous_failures_count_++;
          return false;
        }
        start_end_derivatives.push_back(initial_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(initial_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    if (point_set.size() < 7 || !allPointsFinite(point_set) ||
        start_end_derivatives.size() != 4 || !allPointsFinite(start_end_derivatives) ||
        !std::isfinite(ts) || ts <= 1.0e-6)
    {
      RCLCPP_ERROR(node_->get_logger(), "Initial trajectory samples or derivatives are invalid");
      continuous_failures_count_++;
      return false;
    }

    if (!used_reference_seed)
    {
      applyLinearZReference(point_set, start_pt(2), local_target_pt(2));
      if (!allPointsFinite(point_set))
      {
        RCLCPP_ERROR(node_->get_logger(), "Initial trajectory height reference is not finite");
        continuous_failures_count_++;
        return false;
      }
    }

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    if (!ctrl_pts.allFinite() || ctrl_pts.cols() < 4)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "B-spline parameterization produced invalid control points");
      continuous_failures_count_++;
      return false;
    }

    if (used_reference_seed)
      bspline_optimizer_rebound_->setReboundReference(ctrl_pts);
    else
      bspline_optimizer_rebound_->clearReboundReference();

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success || !ctrl_pts.allFinite())
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty() || !start_pos.allFinite() || !start_vel.allFinite() ||
        !start_acc.allFinite() || !end_vel.allFinite() || !end_acc.allFinite() ||
        !std::isfinite(pp_.max_vel_) || pp_.max_vel_ <= 0.0)
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      if (!waypoints[wp_i].allFinite())
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Global waypoint %zu is not finite",
            wp_i);
        return false;
      }
      if ((waypoints[wp_i] - points.back()).norm() > 1.0e-4)
        points.push_back(waypoints[wp_i]);
    }

    if (points.size() < 2)
      return false;

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    if (!std::isfinite(total_len) || total_len <= 1.0e-6)
      return false;

    vector<Eigen::Vector3d> resampled_points;
    double actual_spacing = 0.0;
    if (!resamplePolylineByArcLength(
            points,
            pp_.global_path_resample_spacing_,
            pp_.global_path_min_spacing_,
            pp_.global_path_max_spacing_,
            resampled_points,
            actual_spacing))
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Unable to resample global waypoint path (input=%zu, limit=%zu)",
          points.size(),
          kMaxGlobalPolynomialPoints);
      return false;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Global path resampled: %zu -> %zu points, target=%.3f m, average=%.3f m",
        points.size(),
        resampled_points.size(),
        pp_.global_path_resample_spacing_,
        actual_spacing);
    if (actual_spacing > pp_.global_path_max_spacing_ + 1.0e-6)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Global guide polynomial capped at %zu points to bound dense "
          "min-snap memory/CPU; average guide spacing is %.3f m",
          kMaxGlobalPolynomialPoints,
          actual_spacing);
    }

    // write position matrix
    int pt_num = resampled_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = resampled_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    if (!pos.allFinite() || !time.allFinite() ||
        (time.array() <= 1.0e-6).any())
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Global waypoint trajectory contains invalid points or segment times");
      return false;
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    double generated_duration = 0.0;
    if (!initializeAndValidatePolynomialTrajectory(gl_traj, generated_duration))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Generated global waypoint trajectory is not finite");
      return false;
    }

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    if (!start_pos.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
        !end_pos.allFinite() || !end_vel.allFinite() || !end_acc.allFinite() ||
        !std::isfinite(pp_.max_vel_) || pp_.max_vel_ <= 0.0)
      return false;

    const double endpoint_distance = (end_pos - start_pos).norm();
    if (!std::isfinite(endpoint_distance) || endpoint_distance <= 1.0e-6)
      return false;

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    if (!pos.allFinite() || !time.allFinite() ||
        (time.array() <= 1.0e-6).any())
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Global trajectory contains invalid points or segment times");
      return false;
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    double generated_duration = 0.0;
    if (!initializeAndValidatePolynomialTrajectory(gl_traj, generated_duration))
    {
      RCLCPP_ERROR(node_->get_logger(), "Generated global trajectory is not finite");
      return false;
    }

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.progress_time_ = 0.0;
    local_data_.progress_arc_length_ = 0.0;
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    if (!std::isfinite(duration) || duration <= 0.0 ||
        !std::isfinite(sample_dt) || sample_dt <= 0.0 ||
        !std::isfinite(vel_limit) || !std::isfinite(acc_limit))
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Dynamic feasibility received invalid trajectory limits");
      return false;
    }

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (!vel.allFinite() || vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (!acc.allFinite() || acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
