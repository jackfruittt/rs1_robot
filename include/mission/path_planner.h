/**
 * @file path_planner.h
 * @brief Path planning and waypoint management for autonomous drone navigation
 * @author Jackson Russell
 * OTHER AUTHORS ADD HERE AND BELOW
 * @date August-2025
 */

#ifndef DRONE_SWARM_PATH_PLANNER_H_
#define DRONE_SWARM_PATH_PLANNER_H_

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

// TODO: ADD NAV2 PATH HANDLING INCLUDES FOR AUTONOMOUS PATH PLANNING
// #include "nav_msgs/msg/path.hpp"

namespace drone_swarm
{

/**
 * @class PathPlanner
 * @brief Manages waypoint sequences and path planning for drone missions
 * 
 * Provides sequential waypoint navigation with distance calculations
 * and path management. Supports dynamic waypoint updates and mission
 * replanning for autonomous drone operations.
 */
class PathPlanner
{
public:
  /**
   * @brief Default constructor
   * Initialises empty waypoint list with index at zero
   */
  PathPlanner();
  
  /**
   * @brief Set new waypoint sequence for mission
   * @param waypoints Vector of stamped poses defining the path
   * 
   * Replaces current waypoint list and resets index to beginning.
   * Waypoints should be ordered for sequential navigation.
   */
  void setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints);
  
  // TODO: ADD NAV2 PATH CONVERSION METHOD FOR AUTONOMOUS PATH PLANNING
  // /**
  //  * @brief Set waypoints directly from nav2 path
  //  * @param path Planned path from nav2 RRT planner
  //  * @param subsample_distance Minimum distance between waypoints (optional)
  //  * 
  //  * Converts nav_msgs::Path to waypoint sequence with optional subsampling
  //  * to reduce waypoint density for smoother drone navigation.
  //  */
  // void setWaypointsFromPath(const nav_msgs::msg::Path& path, double subsample_distance = 1.0);
  
  /**
   * @brief Get next waypoint and advance index
   * @return Next waypoint in sequence, or empty pose if none available
   * 
   * Advances internal index after returning waypoint. Check hasNextWaypoint()
   * before calling to avoid empty returns.
   */
  geometry_msgs::msg::PoseStamped getNextWaypoint();
  
  /**
   * @brief Get current waypoint without advancing index
   * @return Current waypoint in sequence, or empty pose if none available
   * 
   * Does not modify internal state. Safe to call repeatedly.
   */
  geometry_msgs::msg::PoseStamped getCurrentWaypoint() const;
  
  /**
   * @brief Check if more waypoints are available
   * @return True if waypoints remain in sequence, false if at end
   */
  bool hasNextWaypoint() const;
  
  /**
   * @brief Reset waypoint index to beginning
   * 
   * Allows restarting mission from first waypoint without
   * changing the waypoint list.
   */
  void reset();
  
  /**
   * @brief Calculate distance to current waypoint
   * @param current_pose Current drone position
   * @return Euclidean distance to current waypoint in metres
   * 
   * Returns 0.0 if no waypoints available. Uses 3D distance calculation
   * including altitude differences.
   */
  double getDistanceToWaypoint(const geometry_msgs::msg::PoseStamped& current_pose) const;

private:
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;  ///< Sequence of waypoints for navigation
  size_t current_waypoint_index_;                          ///< Current position in waypoint sequence
};

}  // namespace drone_swarm

#endif  // DRONE_SWARM_PATH_PLANNER_H_
