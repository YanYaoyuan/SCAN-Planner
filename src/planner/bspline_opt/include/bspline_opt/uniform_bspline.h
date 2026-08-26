// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file uniform_bspline.h @brief Uniform B-spline evaluation and feasibility API. */

#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace scan_planner
{
  /**
   * @brief Represents a uniform B-spline and its time derivatives.
   *
   * Control points are stored by column; each column is one point and each row
   * is one coordinate dimension.
   */
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    /** @brief Builds control points for the first derivative spline. @return Derivative control points. */
    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, feasibility_tolerance_; // physical limits and feasibility tolerance

  public:
    /** @brief Constructs an empty spline. */
    UniformBspline()
      : p_(0), n_(-1), m_(0), interval_(0.0), limit_vel_(0.0),
        limit_acc_(0.0), feasibility_tolerance_(0.0) {}
    /**
     * @brief Constructs a uniform B-spline.
     * @param points Control-point matrix with one control point per column.
     * @param order Polynomial degree.
     * @param interval Uniform knot interval in seconds.
     */
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    /** @brief Destroys the spline. */
    ~UniformBspline();

    /** @brief Returns a copy of the control-point matrix. @return Control points. */
    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    /**
     * @brief Reinitializes this object as a uniform B-spline.
     * @param points Control-point matrix.
     * @param order Polynomial degree.
     * @param interval Uniform knot interval in seconds.
     */
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    /** @brief Replaces the knot vector. @param knot New nondecreasing knot vector. */
    void setKnot(const Eigen::VectorXd &knot);
    /** @brief Returns the knot vector. @return Knot vector. */
    Eigen::VectorXd getKnot();
    /** @brief Returns the control-point matrix. @return Control points. */
    Eigen::MatrixXd getControlPoint();
    /** @brief Returns the uniform knot interval. @return Interval in seconds. */
    double getInterval();
    /**
     * @brief Gets the valid De Boor parameter range.
     * @param[out] um Beginning of the valid interval.
     * @param[out] um_p End of the valid interval.
     * @return True when a valid span is available.
     */
    bool getTimeSpan(double &um, double &um_p);

    /** @brief Evaluates the spline using De Boor's algorithm. @param u Knot-domain parameter. @return Spline value. */
    Eigen::VectorXd evaluateDeBoor(const double &u) const;                                               // use u \in [up, u_mp]
    /** @brief Evaluates at trajectory time. @param t Time from trajectory start in seconds. @return Spline value. */
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) const { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    /** @brief Constructs the first time derivative. @return Derivative spline. */
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    /**
     * @brief Fits a cubic B-spline to samples and endpoint derivatives.
     * @param ts Sample interval in seconds.
     * @param point_set Ordered trajectory samples.
     * @param start_end_derivative Start/end velocity followed by start/end acceleration.
     * @param[out] ctrl_pts Fitted B-spline control points.
     */
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */

    /**
     * @brief Configures dynamic feasibility limits.
     * @param vel Maximum velocity.
     * @param acc Maximum acceleration.
     * @param tolerance Multiplicative feasibility tolerance.
     */
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    /** @brief Checks velocity and acceleration limits. @param[out] ratio Required time scaling. @param show Whether to print violations. @return True if feasible. */
    bool checkFeasibility(double &ratio, bool show = false);
    /** @brief Lengthens trajectory time. @param ratio Time scaling factor. */
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    /** @brief Returns trajectory duration. @return Duration in seconds. */
    double getTimeSum();
    /** @brief Numerically integrates path length. @param res Sampling interval in seconds. @return Path length. */
    double getLength(const double &res = 0.01);
    /** @brief Integrates squared jerk over time. @return Jerk metric. */
    double getJerk();
    /** @brief Computes velocity statistics. @param[out] mean_v Mean speed. @param[out] max_v Maximum speed. */
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    /** @brief Computes acceleration statistics. @param[out] mean_a Mean acceleration norm. @param[out] max_a Maximum acceleration norm. */
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace scan_planner
#endif
