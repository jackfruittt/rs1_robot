/**
 * @file mission_state.h
 * @brief Mission state enumeration for drone swarm mission management
 * @author Jackson Russell
 * @author Matthew Chua
 * OTHER AUTHORS ADD HERE AND BELOW
 * @date August-2025
 */

#ifndef MISSION_STATE_H_
#define MISSION_STATE_H_

namespace drone_swarm
{

/**
 * @enum MissionState
 * @brief Enumeration of possible mission states for drone operations
 * 
 * Defines the core state machine for autonomous drone flight operations.
 * Scenario reactions (wildfire, hiker rescue, debris) are now handled
 * via services rather than states, allowing clean preservation and
 * restoration of mission progress.
 */
enum class MissionState
{
  IDLE,                         ///< Drone on ground, ready for mission start
  TAKEOFF,                      ///< Ascending to mission altitude
  WAYPOINT_NAVIGATION,          ///< Actively navigating between waypoints
  RESPONSE_NAVIGATION,          ///< Navigating waypoints for scenario response mission
  HOVERING,                     ///< Maintaining position, awaiting next command
  LANDING,                      ///< Descending to ground
  MANUAL_CONTROL,               ///< External manual control mode
  EMERGENCY                     ///< Emergency state requiring immediate attention
};

}  // namespace drone_swarm

#endif  // MISSION_STATE_H_