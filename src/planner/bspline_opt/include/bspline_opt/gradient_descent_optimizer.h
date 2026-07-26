// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file gradient_descent_optimizer.h @brief Generic gradient-descent optimizer API. */

#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <iostream>
#include <limits>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

/**
 * @brief Minimizes a differentiable objective with bounded iterations and evaluations.
 */
class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Objective callback evaluated by the optimizer.
   * @param x Current optimization variables.
   * @param grad Output gradient at @p x.
   * @param force_return Set true by the callback to terminate immediately.
   * @param data User-owned callback context.
   * @return Objective value at @p x.
   */
  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);

  /** @brief Termination reason returned by optimize(). */
  enum RESULT
  {
    FIND_MIN,
    FAILED,
    RETURN_BY_ORDER,
    REACH_MAX_ITERATION
  };

  /**
   * @brief Constructs an optimizer for a fixed-size variable vector.
   * @param v_num Number of scalar optimization variables.
   * @param objf Objective callback.
   * @param f_data Opaque context passed to @p objf.
   */
  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  {
    variable_num_ = v_num;
    objfun_ = objf;
    f_data_ = f_data;
  };

  /** @brief Sets the iteration limit. @param limit Maximum optimizer iterations. */
  void set_maxiter(int limit) { iter_limit_ = limit; }
  /** @brief Sets the objective-evaluation limit. @param limit Maximum callback invocations. */
  void set_maxeval(int limit) { invoke_limit_ = limit; }
  /** @brief Sets the relative variable-change tolerance. @param xtol_rel Relative tolerance. */
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; }
  /** @brief Sets the absolute variable-change tolerance. @param xtol_abs Absolute tolerance. */
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; }
  /** @brief Sets the convergence gradient threshold. @param min_grad Minimum gradient norm. */
  void set_min_grad(double min_grad) { min_grad_ = min_grad; }

  /**
   * @brief Runs optimization from the supplied initial vector.
   * @param[in,out] x_init_optimal Initial variables on entry and best variables on return.
   * @param[out] opt_f Objective value associated with the returned variables.
   * @return Reason the optimization loop terminated.
   */
  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  int variable_num_{0};
  int iter_limit_{std::numeric_limits<int>::max()};
  int invoke_limit_{std::numeric_limits<int>::max()};
  double xtol_rel_;
  double xtol_abs_;
  double min_grad_;
  double time_limit_;
  void *f_data_;
  objfunDef objfun_;
};

#endif
