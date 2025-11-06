/**
 * @file mission_executor.h
 * @brief Mission execution class for simple and complex autonomous operations.
 * @author Jackson Russell
 * OTHER AUTHORS ADD HERE AND BELOW
 * @author Matthew Chua
 * @date August-2025
 */

#ifndef DRONE_SWARM_MISSION_EXECUTOR_H
#define DRONE_SWARM_MISSION_EXECUTOR_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

#include "mission_state.h"
#include "mission/waypoint_planner.h"
#include "drone_control.h"

namespace drone_swarm
{

/**
 * @class MissionExecutor
 * @brief Advanced mission execution for complex autonomous operations
 * 
 * Placeholder for future implementation of advanced mission types including
 * formation flying, collaborative tasks, and dynamic mission replanning.
 * Currently provides basic structure for mission state management.
 * 
 * @note Most functionality is commented out for future development
 */
class MissionExecutor
{
public:
  /**
   * @brief Default constructor
   * Initialises mission executor with default parameters
   */
  MissionExecutor();
  
  /**
   * @brief Set ROS 2 logger for debugging output
   * @param logger ROS 2 logger instance
   */
  void setLogger(rclcpp::Logger logger);
  
  /**
   * @brief Set command velocity publisher
   * @param pub ROS 2 publisher for geometry_msgs::msg::Twist
   */
  void setCmdVelPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub);
  
  /**
   * @brief Execute wildfire response mission
   * @param fire_location Location of the fire
   * @param depot_location Location of retardant depot
   * @param path_planner Path planner for waypoint management
   * @return True if mission completed successfully
   * 
   * Sequence: Navigate to depot -> Land -> Collect retardant -> Takeoff ->
   *           Navigate to fire -> Deploy retardant -> Return to previous mission
   */
  bool executeWildfireReaction(
      const geometry_msgs::msg::Point& fire_location,
      const geometry_msgs::msg::Point& depot_location,
      const geometry_msgs::msg::Point& helipad_location,
      WaypointPlanner* path_planner);
  
  /**
   * @brief Execute stranded hiker rescue mission
   * @param hiker_location Location of stranded hiker
   * @param depot_location Location of medkit depot
   * @param path_planner Path planner for waypoint management
   * @return True if mission completed successfully
   * 
   * Sequence: Navigate to depot -> Land -> Collect medkit -> Takeoff ->
   *           Navigate to hiker -> Deploy medkit -> Return to previous mission
   */
  bool executeHikerRescue(
      const geometry_msgs::msg::Point& hiker_location,
      const geometry_msgs::msg::Point& depot_location,
      WaypointPlanner* path_planner);
  
  /**
   * @brief Execute debris obstruction notification
   * @param debris_location Location of debris
   * @return True if notification completed
   * 
   * Simple notification - logs debris location for path planning avoidance
   */
  bool executeDebrisReaction(const geometry_msgs::msg::Point& debris_location);
  
  /**
   * @brief Save current mission state for later restoration
   * @param path_planner Path planner with current waypoints
   * 
   * Stores waypoint progress and mission state before scenario reaction
   */
  void saveMissionState(WaypointPlanner* path_planner);
  
  /**
   * @brief Restore previously saved mission state
   * @param path_planner Path planner to restore waypoints to
   * @return True if state was restored successfully
   * 
   * Restores waypoint progress after scenario reaction completes
   */
  bool restoreMissionState(WaypointPlanner* path_planner);
  
  /**
   * @brief Check if mission has been cancelled or aborted
   * @return True if mission should be terminated
   */
  bool isMissionCancelled() const;
  
  /**
   * @brief Cancel current mission operations
   * Sets internal flag to stop mission execution
   */
  void cancelMission();
  
  /**
   * @brief Reset mission executor for new mission
   * Clears all internal state and prepares for new operations
   */
  void reset();

private:
  // ROS 2 integration
  rclcpp::Logger logger_;                                                ///< ROS 2 logger for debugging
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  ///< Command publisher
  
  // Mission state
  bool mission_cancelled_;                                               ///< Flag indicating if mission is cancelled
  std::unique_ptr<DroneControl> flight_controller_;                      ///< Flight control system
  
  // State preservation for scenario reactions
  struct SavedMissionState {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    size_t current_waypoint_index;
    bool valid;
  };
  SavedMissionState saved_state_;       ///< Saved mission state for restoration
};

}  // namespace drone_swarm

#endif  // DRONE_SWARM_MISSION_EXECUTOR_H