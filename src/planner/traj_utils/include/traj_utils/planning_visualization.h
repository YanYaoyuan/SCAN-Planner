// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file planning_visualization.h @brief RViz planning-visualization API. */

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <traj_utils/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace scan_planner
{
  /** @brief Builds and publishes RViz markers for planner diagnostics. */
  class PlanningVisualization
  {
  private:
    using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>;
    using MarkerArrayPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;
    rclcpp::Node *node_{nullptr};
    std::string frame_id_{"world"};

    MarkerPublisher::SharedPtr goal_point_pub;
    MarkerPublisher::SharedPtr global_list_pub;
    MarkerPublisher::SharedPtr init_list_pub;
    MarkerPublisher::SharedPtr optimal_list_pub;
    MarkerPublisher::SharedPtr a_star_list_pub;

  public:
    /** @brief Constructs an uninitialized visualization helper. */
    PlanningVisualization(/* args */) {}
    /** @brief Destroys the helper. */
    ~PlanningVisualization() {}
    /** @brief Creates publishers on a ROS node. @param node Owning ROS node. */
    explicit PlanningVisualization(rclcpp::Node *node);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    /** @brief Publishes points as a marker list. @param pub Destination publisher. @param list Points. @param scale Marker scale. @param color RGBA color. @param id Marker ID. */
    void displayMarkerList(const MarkerPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);
    /** @brief Appends path markers to an array. @param[out] array Marker array. @param list Points. @param scale Marker scale. @param color RGBA color. @param id Marker ID. */
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    /** @brief Appends direction arrows to an array. @param[out] array Marker array. @param list Point pairs. @param scale Arrow scale. @param color RGBA color. @param id Marker ID. */
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    /** @brief Displays one goal point. @param goal_point Goal coordinate. @param color RGBA color. @param scale Marker scale. @param id Marker ID. */
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    /** @brief Displays the global path. @param global_pts Path points. @param scale Marker scale. @param id Marker ID. */
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    /** @brief Displays an initial local path. @param init_pts Path points. @param scale Marker scale. @param id Marker ID. */
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    /** @brief Displays optimized control points. @param optimal_pts Control points. @param id Marker ID. */
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    /** @brief Displays a sampled optimized spline. @param position_traj Position spline. @param id Marker ID. */
    void displayOptimalTraj(UniformBspline position_traj, int id);
    /** @brief Displays A* bypass paths. @param a_star_paths List of paths. @param id Marker ID. */
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    /** @brief Publishes arrow markers. @param pub Destination publisher. @param list Point pairs. @param scale Arrow scale. @param color RGBA color. @param id Marker ID. */
    void displayArrowList(const MarkerArrayPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list,
                          double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, scan_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, scan_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace scan_planner
#endif
