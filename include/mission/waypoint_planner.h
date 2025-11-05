/**
 * @file waypoint_planner.h
 * @brief Waypoint management for autonomous drone navigation
 * @author Jackson Russell
 * OTHER AUTHORS ADD HERE AND BELOW
 * @date August-2025
 */

#ifndef DRONE_SWARM_WAYPOINT_PLANNER_H_
#define DRONE_SWARM_WAYPOINT_PLANNER_H_

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace drone_swarm
{

/**
 * @class WaypointPlanner
 * @brief Manages waypoint sequences for drone missions
 * 
 * Provides sequential waypoint navigation with distance calculations
 * and waypoint management. Supports dynamic waypoint updates and mission
 * replanning for autonomous drone operations.
 */
class WaypointPlanner
{
public:
  /**
   * @brief Default constructor
   * Initialises empty waypoint list with index at zero
   */
  WaypointPlanner();
  
  /**
   * @brief Set new waypoint sequence for mission
   * @param waypoints Vector of stamped poses defining the path
   * 
   * Replaces current waypoint list and resets index to beginning.
   * Waypoints should be ordered for sequential navigation.
   */
  void setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints);
  
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
  
  /**
   * @brief Get all waypoints in the current path
   * @return Vector of all waypoints
   * 
   * Used for state preservation during scenario reactions
   */
  std::vector<geometry_msgs::msg::PoseStamped> getAllWaypoints() const;
  
  /**
   * @brief Get current waypoint index
   * @return Current index in waypoint sequence
   * 
   * Used for state preservation during scenario reactions
   */
  size_t getCurrentWaypointIndex() const;
  
  /**
   * @brief Set current waypoint index
   * @param index Index to set as current waypoint
   * 
   * Used for state restoration after scenario reactions
   * Clamped to valid range [0, waypoints.size()]
   */
  void setCurrentWaypointIndex(size_t index);

private:
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;  ///< Sequence of waypoints for navigation
  size_t current_waypoint_index_;                           ///< Current position in waypoint sequence
};

}  // namespace drone_swarm

#endif  // DRONE_SWARM_WAYPOINT_PLANNER_H_
