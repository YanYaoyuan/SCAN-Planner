// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file bspline_optimizer.h @brief Collision-aware B-spline optimizer API. */

#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>
#include "bspline_opt/lbfgs.hpp"

// Gradient and elastic band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace scan_planner
{

  /** @brief Stores B-spline control points and obstacle-rebound geometry. */
  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the start of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it every time before using it.
    // std::vector<bool> occupancy;

    /** @brief Resizes all per-control-point storage. @param size_set New control-point count. */
    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

  /**
   * @brief Optimizes a B-spline for smoothness, clearance, feasibility, and route adherence.
   */
  class BsplineOptimizer
  {

  public:
    /** @brief Constructs an optimizer with parameters awaiting initialization. */
    BsplineOptimizer() {}
    /** @brief Destroys the optimizer. */
    ~BsplineOptimizer() {}

    /** @brief Binds the collision map. @param env Shared local occupancy map. */
    void setEnvironment(const GridMap::Ptr &env);
    /** @brief Loads optimization parameters from a ROS node. @param node Parameter-owning node. */
    void setParam(rclcpp::Node *node);
    /**
     * @brief Runs the generic configured B-spline optimization.
     * @param points Initial control points.
     * @param ts Knot interval in seconds.
     * @param cost_function Bit mask selecting objective terms.
     * @param max_num_id Iteration-limit profile identifier.
     * @param max_time_id Runtime-limit profile identifier.
     * @return Optimized control-point matrix.
     */
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /** @brief Replaces current control points. @param points Control-point matrix. */
    void setControlPoints(const Eigen::MatrixXd &points);
    /** @brief Sets the knot interval. @param ts Interval in seconds. */
    void setBsplineInterval(const double &ts);
    /** @brief Selects generic objective terms. @param cost_function Objective bit mask. */
    void setCostFunction(const int &cost_function);
    /** @brief Selects generic termination profiles. @param max_num_id Iteration profile. @param max_time_id Runtime profile. */
    void setTerminateCond(const int &max_num_id, const int &max_time_id);
    /** @brief Enables route adherence for rebound optimization. @param points Reference control points. */
    void setReboundReference(const Eigen::MatrixXd &points);
    /** @brief Disables and clears the rebound reference. */
    void clearReboundReference();

    /** @brief Sets a geometric guide path. @param guide_pt Ordered guide points. */
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    /** @brief Adds waypoint constraints. @param waypts Waypoint coordinates. @param waypt_idx Associated control-point indices. */
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    /** @brief Runs the legacy generic optimization using configured state. */
    void optimize();

    /** @brief Returns current control points. @return Control-point matrix. */
    Eigen::MatrixXd getControlPoints();

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    /**
     * @brief Initializes rebound directions and obstacle bypass paths.
     * @param[in,out] init_points Initial control points; may be adjusted for rebound.
     * @param flag_first_init Whether to reset all optimizer state.
     * @return A* bypass paths generated for collision segments.
     */
    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    /** @brief Runs rebound optimization. @param[out] optimal_points Optimized control points. @param ts Knot interval. @return True when collision-free optimization succeeds. */
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    /** @brief Refines a dynamically infeasible trajectory. @param init_points Initial control points. @param ts Knot interval. @param[out] optimal_points Refined points. @return True on success. */
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    /** @brief Returns the configured B-spline degree. @return Polynomial degree. */
    inline int getOrder(void) { return order_; }

  private:
    GridMap::Ptr grid_map_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting
    double lambda5_;               // reference-route tracking during rebound
    int a;
    //
    double dist0_;             // safe distance
    double max_vel_, max_acc_; // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    ControlPoints cps_;
    Eigen::MatrixXd rebound_reference_;
    bool use_rebound_reference_{false};

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    /** @brief Adapter for the generic optimizer. @param x Variables. @param[out] grad Gradient. @param func_data Optimizer instance. @return Combined cost. */
    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    /** @brief Combines selected generic costs. @param x Variables. @param[out] grad Gradient. @param[out] cost Cost. */
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    /** @brief Computes smoothness cost. @param q Control points. @param[out] cost Cost. @param[out] gradient Gradient. @param falg_use_jerk True for jerk, false for acceleration. */
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    /** @brief Computes dynamic-limit cost. @param q Control points. @param[out] cost Cost. @param[out] gradient Gradient. */
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    /** @brief Computes obstacle rebound cost. @param q Control points. @param[out] cost Cost. @param[out] gradient Gradient. @param iter_num Optimizer iteration. @param smoothness_cost Current smoothness cost. */
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    /** @brief Computes reference-route adherence cost. @param q Control points. @param[out] cost Cost. @param[out] gradient Gradient. */
    void calcReboundReferenceCost(const Eigen::MatrixXd &q, double &cost,
                                  Eigen::MatrixXd &gradient);
    /** @brief Computes curve-fitting cost. @param q Control points. @param[out] cost Cost. @param[out] gradient Gradient. */
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    /** @brief Detects residual collisions and updates rebound directions. @return True when optimization must restart. */
    bool check_collision_and_rebound(void);
    /** @brief Estimates planar segment heading. @param from Segment start. @param to Segment end. @return Yaw in radians. */
    double estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    /** @brief Estimates heading at one control point. @param q Control points. @param id Control-point index. @return Yaw in radians. */
    double estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const;

    /** @brief L-BFGS early-stop callback. @param func_data Optimizer instance. @param x Variables. @param g Gradient. @param fx Cost. @param xnorm Variable norm. @param gnorm Gradient norm. @param step Line-search step. @param n Variable count. @param k Iteration. @param ls Line-search iteration. @return Nonzero to stop. */
    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    /** @brief L-BFGS rebound-cost callback. @param func_data Optimizer instance. @param x Variables. @param[out] grad Gradient. @param n Variable count. @return Cost. */
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    /** @brief L-BFGS refinement-cost callback. @param func_data Optimizer instance. @param x Variables. @param[out] grad Gradient. @param n Variable count. @return Cost. */
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    /** @brief Runs collision-rebound L-BFGS. @return True on success. */
    bool rebound_optimize();
    /** @brief Runs feasibility-refinement L-BFGS. @return True on success. */
    bool refine_optimize();
    /** @brief Combines rebound objective terms. @param x Variables. @param[out] grad Gradient. @param[out] f_combine Total cost. @param n Variable count. */
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    /** @brief Combines refinement objective terms. @param x Variables. @param[out] grad Gradient. @param[out] f_combine Total cost. @param n Variable count. */
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benchmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner
#endif
