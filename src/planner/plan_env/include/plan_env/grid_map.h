// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file grid_map.h @brief Sliding occupancy-map and collision-query API. */

#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <tuple>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;
/** @brief Hashes fixed-size Eigen matrices for unordered containers. @tparam T Eigen matrix type. */
template <typename T>
struct matrix_hash {
  /** @brief Computes a combined scalar hash. @param matrix Matrix key. @return Hash value. */
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

/** @brief Immutable and runtime-configured parameters used by GridMap. */
struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  Eigen::Vector3i map_origin_idx_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_z_up, obstacles_inflation_z_down;
  double double_cylinder_radius_, double_cylinder_offset_;
  bool map_sliding_en_;
  double map_sliding_thresh_;
  int map_sliding_thresh_vox_;
  string frame_id_, sliding_map_frame_id_;

  /* depth camera intrinsics */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* visualization and computation time display */
  double vis_height_, ground_height_;
  bool show_occ_time_;

  /* mapping sensor input */
  string sensor_type_, sensor_frame_id_;
  double lidar_sync_tolerance_;
  bool cloud_is_world_;
  bool need_extrinsic_;
  Eigen::Matrix4d lidar_extrinsic_;
  Eigen::Matrix4d depth_extrinsic_;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion

/** @brief Mutable sensor-fusion buffers and map-update state. */
struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<int> occupancy_buffer_inflate_cnt_;
  vector<Eigen::Vector3i> inflate_offsets_;

  // raycast origin and sensor pose data

  Eigen::Vector3d ray_pos_;
  Eigen::Quaterniond ray_q_;
  Eigen::Vector3d sliding_map_frame_pos_;

  // depth image data

  cv::Mat depth_image_;
  int image_cnt_;
  // flags of map state

  bool occ_need_update_;
  bool use_cloud_update_;
  bool has_first_depth_;
  bool has_ray_pose_, has_cloud_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @brief Maintains a sliding 3D occupancy grid and inflated robot collision layer. */
class GridMap {
public:
  /** @brief Constructs an uninitialized map. */
  GridMap() {}
  /** @brief Destroys the map and ROS interfaces. */
  ~GridMap() {}

  enum { INVALID_IDX = -10000 };

  /** @brief Resets the complete occupancy buffer. */
  void resetBuffer();
  /** @brief Resets voxels inside a world-coordinate box. @param min Minimum corner. @param max Maximum corner. */
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  /** @brief Converts a world position to a global voxel index. @param pos World position. @param[out] id Voxel index. */
  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  /** @brief Converts a global voxel index to its center position. @param id Voxel index. @param[out] pos World position. */
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  /** @brief Converts a global voxel index to circular-buffer address. @param id Voxel index. @return Buffer address. */
  inline int toAddress(const Eigen::Vector3i& id);
  /** @brief Converts index components to circular-buffer address. @param x X index. @param y Y index. @param z Z index. @return Buffer address. */
  inline int toAddress(int& x, int& y, int& z);
  /** @brief Tests whether a position lies in the active map. @param pos World position. @return True when inside. */
  inline bool isInMap(const Eigen::Vector3d& pos);
  /** @brief Tests whether an index lies in the active map. @param idx Global index. @return True when inside. */
  inline bool isInMap(const Eigen::Vector3i& idx);

  /** @brief Assigns a binary occupancy state. @param pos World position. @param occ 1 for occupied, 0 for free. */
  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  /** @brief Marks a position occupied. @param pos World position. */
  inline void setOccupied(Eigen::Vector3d pos);
  /** @brief Queries raw occupancy by position. @param pos World position. @return -1 outside, 0 free, 1 occupied. */
  inline int getOccupancy(Eigen::Vector3d pos);
  /** @brief Queries raw occupancy by index. @param id Global voxel index. @return -1 outside, 0 free, 1 occupied. */
  inline int getOccupancy(Eigen::Vector3i id);
  /** @brief Queries inflated occupancy for the double-cylinder body model. @param pos Body-center position. @param yaw Body yaw. @return -1 outside, 0 free, 1 occupied. */
  inline int getInflateOccupancy(Eigen::Vector3d pos, double yaw);

  /** @brief Clamps an index to map bounds. @param[in,out] id Index to clamp. */
  inline void boundIndex(Eigen::Vector3i& id);
  /** @brief Tests whether an indexed cell is unobserved. @param id Voxel index. @return True when unknown. */
  inline bool isUnknown(const Eigen::Vector3i& id);
  /** @brief Tests whether a position is unobserved. @param pos World position. @return True when unknown. */
  inline bool isUnknown(const Eigen::Vector3d& pos);
  /** @brief Tests for observed, inflation-free space. @param id Voxel index. @return True when known free. */
  inline bool isKnownFree(const Eigen::Vector3i& id);
  /** @brief Tests inflated occupancy. @param id Voxel index. @return True when known occupied. */
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  /** @brief Loads parameters and creates ROS interfaces. @param node Owning ROS node. */
  void initMap(rclcpp::Node* node);

  /** @brief Publishes raw occupied voxels. */
  void publishMap();
  /** @brief Publishes inflated occupied voxels. @param all_info Whether to publish the complete buffer. */
  void publishMapInflate(bool all_info = false);

  /** @brief Publishes unknown voxels. */
  void publishUnknown();
  /** @brief Publishes the filtered depth image. */
  void publishDepth();
  /** @brief Publishes the depth-derived point cloud. */
  void publishDepthCloud();
  /** @brief Publishes the active sliding-map bounds marker. */
  void publishSlidingMapBBox();
  /** @brief Publishes the sliding-map frame pose. */
  void publishSlidingMapFrame();

  /** @brief Reports whether depth data have been integrated. @return True after first valid depth observation. */
  bool hasDepthObservation();
  /** @brief Reports whether sensor odometry is valid. @return True after a valid pose. */
  bool odomValid();
  /** @brief Returns the active map region. @param[out] ori Minimum corner. @param[out] size Region size. */
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  /** @brief Returns voxel resolution. @return Resolution in meters. */
  inline double getResolution();
  /** @brief Returns map origin. @return Origin in world coordinates. */
  Eigen::Vector3d getOrigin();
  /** @brief Returns total voxel count. @return Buffer cell count. */
  int getVoxelNum();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  /** @brief Receives synchronized depth and pose. @param img Depth image. @param pose Sensor odometry. */
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  /** @brief Updates the latest sensor pose. @param pose Sensor odometry. */
  void sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  /** @brief Integrates synchronized lidar cloud and pose. @param cloud Point cloud. @param pose Sensor odometry. */
  void lidarCloudPoseCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
      const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  /** @brief Updates the sliding-map center frame. @param pose Frame odometry. */
  void slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  /** @brief Receives a world-frame cloud without synchronized pose. @param img Point cloud message. */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& img);

  /** @brief Timer callback that fuses pending observations. */
  void updateOccupancyCallback();
  /** @brief Timer callback that publishes visualization topics. */
  void visCallback();

  /** @brief Projects the current depth image into 3D sensor rays. */
  void projectDepthImage();
  /** @brief Integrates projected rays into log-odds occupancy. */
  void raycastProcess();

  /** @brief Generates inflation offsets around one voxel. @param pt Center voxel. @param inf_step_xy Horizontal radius in cells. @param inf_step_z_up Upward radius. @param inf_step_z_down Downward radius. @param[out] pts Inflated indices. */
  inline void inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts);
  /** @brief Queries a selected inflation buffer. @param pos World position. @param buffer Inflation buffer. @return Occupancy state. */
  inline int getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer);
  /** @brief Wraps a global index into one local dimension. @param id Global index. @param dim Dimension. @return Local circular index. */
  inline int getLocalIndex(int id, int dim) const;
  /** @brief Converts a local index to linear address. @param id_l Local index. @return Address. */
  inline int toAddressLocal(const Eigen::Vector3i& id_l) const;
  /** @brief Converts local components to linear address. @param x X index. @param y Y index. @param z Z index. @return Address. */
  inline int toAddressLocal(int x, int y, int z) const;
  /** @brief Accumulates one ray observation in cache buffers. @param pos World position. @param occ Endpoint occupancy flag. @return Updated address or invalid marker. */
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  /** @brief Clips a ray endpoint to map bounds. @param pt Requested endpoint. @param ray_pos Ray origin. @return Closest in-map point. */
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& ray_pos);
  /** @brief Slides the circular map around a new center. @param center New center position. */
  void updateSlidingMap(const Eigen::Vector3d& center);
  /** @brief Recomputes metric boundaries from index boundaries. */
  void updateMapBoundaryFromIndex();
  /** @brief Clears all occupancy and inflation data. */
  void resetAllMapData();
  /** @brief Clears one buffer address. @param addr Linear address. */
  void resetCellByAddress(int addr);
  /** @brief Clears one address while respecting the slide mask. @param addr Linear address. @param clear_mask Cells already cleared. */
  void resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask);
  /** @brief Converts a circular-buffer address to global index. @param addr Linear address. @param[out] id_g Global index. */
  void hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const;
  /** @brief Applies log-odds occupancy and inflation transition. @param id Global index. @param new_log_odds New log-odds value. */
  void applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds);
  /** @brief Rebuilds precomputed inflation offsets. */
  void rebuildInflationOffsets();
  /** @brief Applies an inflation-count delta. @param id Changed voxel. @param delta Count increment. @param ignore_mask Optional ignored cells. */
  void updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask = nullptr);
  /** @brief Updates one inflation layer. @param id Changed voxel. @param delta Count increment. @param offsets Inflation offsets. @param cnt_buffer Reference counts. @param flag_buffer Occupancy flags. @param ignore_mask Optional ignored cells. */
  void updateInflationLayer(const Eigen::Vector3i& id, int delta,
                            const vector<Eigen::Vector3i>& offsets,
                            std::vector<int>& cnt_buffer,
                            std::vector<char>& flag_buffer,
                            const std::vector<char>* ignore_mask);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>>
      SynchronizerCloudPose;

  rclcpp::Node* node_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> depth_pose_sub_;
  SynchronizerImagePose sync_image_pose_;

  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> lidar_pose_sub_;
  SynchronizerCloudPose sync_cloud_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sliding_map_frame_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sliding_map_bbox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr extrinsic_pose_pub_;
  rclcpp::TimerBase::SharedPtr occ_timer_, vis_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0) local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

inline int GridMap::toAddressLocal(const Eigen::Vector3i& id_l) const {
  return id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id_l(1) * mp_.map_voxel_num_(2) + id_l(2);
}

inline int GridMap::toAddressLocal(int x, int y, int z) const {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d front = pos + mp_.double_cylinder_offset_ * heading;
  Eigen::Vector3d rear = pos - mp_.double_cylinder_offset_ * heading;

  int front_occ = getInflateOccupancyFromBuffer(front, md_.occupancy_buffer_inflate_);
  if (front_occ != 0) return front_occ;

  return getInflateOccupancyFromBuffer(rear, md_.occupancy_buffer_inflate_);
}

inline int GridMap::getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(buffer[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts) {

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  pts.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      if (std::sqrt(x * x + y * y) > inf_step_xy)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z) {
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
