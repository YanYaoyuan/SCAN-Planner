// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file raycast.h @brief Voxel-grid ray traversal API. */

#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

/** @brief Returns the sign of an integer. @param x Input scalar. @return -1, 0, or 1. */
int signum(int x);

/** @brief Computes a positive floating-point modulus. @param value Dividend. @param modulus Divisor. @return Wrapped value. */
double mod(double value, double modulus);

/** @brief Computes ray time to the next integer boundary. @param s Initial coordinate. @param ds Coordinate direction. @return Nonnegative boundary time. */
double intbound(double s, double ds);

/** @brief Traverses voxels intersected by a bounded segment. @param start Segment start. @param end Segment end. @param min Minimum bound. @param max Maximum bound. @param[out] output_points_cnt Number of written points. @param[out] output Caller-owned output array. */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

/** @brief Traverses voxels intersected by a bounded segment. @param start Segment start. @param end Segment end. @param min Minimum bound. @param max Maximum bound. @param[out] output Traversed voxel coordinates. */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/** @brief Stateful voxel iterator implementing grid traversal for one ray. */
class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  /** @brief Constructs an empty ray iterator. */
  RayCaster(/* args */) {
  }
  /** @brief Destroys the iterator. */
  ~RayCaster() {
  }

  /** @brief Initializes traversal for a segment. @param start Segment start. @param end Segment end. @return True when the input defines a valid ray. */
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /** @brief Advances to the next intersected voxel. @param[out] ray_pt Voxel coordinate. @return True while another voxel exists. */
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_
