// Copyright (c) 2026 Omni AI
// SPDX-License-Identifier: Apache-2.0
/** @file dyn_a_star.h @brief Dynamic A* local-grid search API. */

#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <chrono>
#include <cstddef>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

/** @brief Status returned by an A* search request. */
enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR,
	DEADLINE_EXCEEDED,
	RESOURCE_LIMIT
};

/** @brief One reusable node in the A* voxel pool. */
struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

/** @brief Orders A* nodes by increasing estimated total cost. */
class NodeComparator
{
public:
	/** @brief Compares two nodes for the priority queue. @param node1 First node. @param node2 Second node. @return True when @p node1 has lower queue priority. */
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

/** @brief Performs bounded three-dimensional A* search in the local occupancy map. */
class AStar
{
private:
	GridMap::Ptr grid_map_;

	/** @brief Converts coordinates into unchecked pool indices. @param x X coordinate. @param y Y coordinate. @param z Z coordinate. @param[out] id_x X index. @param[out] id_y Y index. @param[out] id_z Z index. */
	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	/** @brief Computes diagonal-grid heuristic. @param node1 Source node. @param node2 Goal node. @return Estimated cost. */
	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	/** @brief Computes Manhattan heuristic. @param node1 Source node. @param node2 Goal node. @return Estimated cost. */
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	/** @brief Computes Euclidean heuristic. @param node1 Source node. @param node2 Goal node. @return Estimated cost. */
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	/** @brief Computes the configured tie-broken heuristic. @param node1 Source node. @param node2 Goal node. @return Estimated cost. */
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	/** @brief Converts and clamps endpoints to the search pool. @param start_pt Start coordinate. @param end_pt Goal coordinate. @param[out] start_idx Start index. @param[out] end_idx Goal index. @return True on success. */
	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	/** @brief Converts a pool index to map coordinates. @param index Pool index. @return Voxel-center coordinate. */
	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	/** @brief Converts a coordinate to a validated pool index. @param pt Coordinate. @param[out] idx Pool index. @return False when outside the pool. */
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	/** @brief Queries inflated occupancy. @param pos Coordinate. @param yaw Robot heading in radians. @return Occupancy state. */
	inline int checkOccupancy(const Eigen::Vector3d &pos, const double yaw) { return grid_map_->getInflateOccupancy(pos, yaw); }

	/** @brief Reconstructs a path from the goal node under the shared deadline. */
	bool retrievePath(GridNodePtr current, std::vector<GridNodePtr> &path);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_{Eigen::Vector3i::Zero()};
	Eigen::Vector3i POOL_SIZE_{Eigen::Vector3i::Zero()};
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_{nullptr};
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};
	using Clock = std::chrono::steady_clock;
	bool deadline_active_{false};
	Clock::time_point deadline_{Clock::time_point::max()};
	std::size_t max_expansions_{100000};
	std::chrono::milliseconds local_time_limit_{200};

public:
	typedef std::shared_ptr<AStar> Ptr;

	/** @brief Constructs an uninitialized search object. */
	AStar(){};
	/** @brief Releases the allocated node pool. */
	~AStar();

	/** @brief Allocates the node pool and binds an occupancy map. @param occ_map Occupancy map. @param pool_size Number of nodes along each axis. */
	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	/** @brief Sets the absolute deadline shared with the owning planning attempt. */
	void setDeadline(Clock::time_point deadline) noexcept;
	/** @brief Disables the external deadline for standalone searches. */
	void clearDeadline() noexcept;
	/** @brief Returns true at or after the configured external deadline. */
	bool deadlineExceeded() const noexcept;
	/** @brief Sets the hard node-expansion limit; zero is clamped to one. */
	void setMaxExpansions(std::size_t max_expansions) noexcept;

	/** @brief Searches between two coordinates. @param step_size Search-grid resolution. @param start_pt Start coordinate. @param end_pt Goal coordinate. @return Search status. */
	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	/** @brief Returns the last successful path. @return Coordinates ordered from start to goal. */
	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
		             "Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
