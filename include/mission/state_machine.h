/**
 * @file state_machine.h
 * @brief Mission state machine for drone flight control
 * @author Jackson Russell
 * OTHER AUTHORS ADD HERE AND BELOW
 * @date August-2025
 */

#ifndef DRONE_SWARM_STATE_MACHINE_H_
#define DRONE_SWARM_STATE_MACHINE_H_

#include <chrono>
#include <string>
#include "mission_state.h"

namespace drone_swarm
{

/**
 * @class StateMachine
 * @brief Manages mission state transitions for autonomous drone operations
 * 
 * Provides safe state transitions with validation and timing information
 * for mission planning and execution. Ensures proper state sequences
 * and prevents invalid transitions that could compromise flight safety.
 */
class StateMachine
{
public:
  /**
   * @brief Default constructor
   * Initialises state machine in IDLE state with current timestamp
   */
  StateMachine();
  
  /**
   * @brief Set new mission state
   * @param new_state Target state to transition to
   * 
   * Updates current state and records entry time for transition tracking.
   * Does not validate transition - use canTransition() for safety checks.
   */
  void setState(MissionState new_state);
  
  /**
   * @brief Get current mission state
   * @return Current mission state enumeration
   */
  MissionState getCurrentState() const;
  
  /**
   * @brief Get human-readable state string
   * @return String representation of current state
   * 
   * Useful for logging and debugging mission state information.
   */
  std::string getStateString() const;
  
  /**
   * @brief Check if transition to target state is valid
   * @param target_state Desired state to transition to
   * @return True if transition is allowed, false otherwise
   * 
   * Validates state transitions according to mission safety rules.
   * Prevents invalid transitions that could compromise flight safety.
   */
  bool canTransition(MissionState target_state) const;

private:
  MissionState current_state_;                                    ///< Current mission state
  std::chrono::steady_clock::time_point state_entry_time_;       ///< Time of last state transition
};

}  // namespace drone_swarm

#endif  // DRONE_SWARM_STATE_MACHINE_H_
