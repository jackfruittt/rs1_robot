#include "mission/state_machine.h"

namespace drone_swarm
{

  StateMachine::StateMachine() : current_state_(MissionState::IDLE) {
    // Initialise state machine in idle state with current timestamp
    state_entry_time_ = std::chrono::steady_clock::now();
  }

  void StateMachine::setState(MissionState new_state) {
    // Update state only if it's different, reset entry time for new state
    if (current_state_ != new_state) {
      current_state_ = new_state;
      state_entry_time_ = std::chrono::steady_clock::now();
    }
  }

  MissionState StateMachine::getCurrentState() const {
    return current_state_;
  }

  std::string StateMachine::getStateString() const {
    // Convert mission state enum to human-readable string
    switch (current_state_) {
      case MissionState::IDLE: return "IDLE";
      case MissionState::TAKEOFF: return "TAKEOFF";
      case MissionState::WAYPOINT_NAVIGATION: return "WAYPOINT_NAVIGATION";
      case MissionState::HOVERING: return "HOVERING";
      case MissionState::LANDING: return "LANDING";
      case MissionState::MANUAL_CONTROL: return "MANUAL_CONTROL";
      case MissionState::EMERGENCY: return "EMERGENCY";
      case MissionState::WILDFIRE_REACTION: return "WILDFIRE_REACTION";
<<<<<<< Updated upstream
      case MissionState::ORBIT_INCIDENT: return "ORBIT_INCIDENT";
=======
      case MissionState::OBSERVE: return "OBSERVE";
>>>>>>> Stashed changes
      case MissionState::STRANDED_HIKER_REACTION: return "STRANDED_HIKER_REACTION";
      case MissionState::DEBRIS_OBSTRUCTION_REACTION: return "DEBRIS_OBSTRUCTION_REACTION";
      default: return "UNKNOWN";
    }
  }

  bool StateMachine::canTransition(MissionState target_state) const {
    // Define allowed transitions
    // TO DO: ADD THE OTHER STATES TO THIS SWITCH
    switch (current_state_) {
      case MissionState::IDLE:
        return  target_state == MissionState::TAKEOFF || 
                target_state == MissionState::MANUAL_CONTROL;
      case MissionState::TAKEOFF:
        return  target_state == MissionState::WAYPOINT_NAVIGATION || 
                target_state == MissionState::HOVERING ||
                target_state == MissionState::EMERGENCY;
      case MissionState::WAYPOINT_NAVIGATION:
        return  target_state == MissionState::HOVERING || 
                target_state == MissionState::LANDING ||
                target_state == MissionState::EMERGENCY || 
                target_state == MissionState::WILDFIRE_REACTION ||
<<<<<<< Updated upstream
                target_state == MissionState::ORBIT_INCIDENT || 
=======
                target_state == MissionState::OBSERVE || 
>>>>>>> Stashed changes
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION;
      case MissionState::HOVERING:
        return  target_state == MissionState::WAYPOINT_NAVIGATION || 
                target_state == MissionState::LANDING ||
                target_state == MissionState::EMERGENCY || 
                target_state == MissionState::WILDFIRE_REACTION ||
<<<<<<< Updated upstream
                target_state == MissionState::ORBIT_INCIDENT || 
=======
                target_state == MissionState::OBSERVE || 
>>>>>>> Stashed changes
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION;
      case MissionState::LANDING:
        return  target_state == MissionState::IDLE || 
                target_state == MissionState::EMERGENCY;
      case MissionState::MANUAL_CONTROL:
        return  target_state == MissionState::IDLE || 
                target_state == MissionState::EMERGENCY;
      case MissionState::EMERGENCY:
        return  target_state == MissionState::IDLE;
<<<<<<< Updated upstream
      case MissionState::ORBIT_INCIDENT:
=======
      case MissionState::OBSERVE:
>>>>>>> Stashed changes
        return  target_state == MissionState::IDLE ||
                target_state == MissionState::TAKEOFF ||
                target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::HOVERING ||
                target_state == MissionState::LANDING ||
                target_state == MissionState::MANUAL_CONTROL ||
                target_state == MissionState::WILDFIRE_REACTION ||
<<<<<<< Updated upstream
                target_state == MissionState::ORBIT_INCIDENT ||
=======
                target_state == MissionState::OBSERVE ||
>>>>>>> Stashed changes
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION ||
                target_state == MissionState::EMERGENCY;
      case MissionState::STRANDED_HIKER_REACTION:
        return  target_state == MissionState::IDLE ||
                target_state == MissionState::TAKEOFF ||
                target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::HOVERING ||
                target_state == MissionState::LANDING ||
                target_state == MissionState::MANUAL_CONTROL ||
                target_state == MissionState::WILDFIRE_REACTION ||
<<<<<<< Updated upstream
                target_state == MissionState::ORBIT_INCIDENT ||
=======
                target_state == MissionState::OBSERVE ||
>>>>>>> Stashed changes
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION ||
                target_state == MissionState::EMERGENCY;
      case MissionState::DEBRIS_OBSTRUCTION_REACTION:
        return  target_state == MissionState::IDLE ||
                target_state == MissionState::TAKEOFF ||
                target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::HOVERING ||
                target_state == MissionState::LANDING ||
                target_state == MissionState::MANUAL_CONTROL ||
                target_state == MissionState::WILDFIRE_REACTION ||
<<<<<<< Updated upstream
                target_state == MissionState::ORBIT_INCIDENT ||
=======
                target_state == MissionState::OBSERVE ||
>>>>>>> Stashed changes
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION ||
                target_state == MissionState::EMERGENCY;
      case MissionState::WILDFIRE_REACTION:
        return  target_state == MissionState::IDLE ||
                target_state == MissionState::TAKEOFF ||
                target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::HOVERING ||
                target_state == MissionState::LANDING ||
                target_state == MissionState::MANUAL_CONTROL ||
                target_state == MissionState::WILDFIRE_REACTION ||
<<<<<<< Updated upstream
                target_state == MissionState::ORBIT_INCIDENT ||
=======
                target_state == MissionState::OBSERVE ||
>>>>>>> Stashed changes
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION ||
                target_state == MissionState::EMERGENCY;
    }
    return false;
  }
}  // namespace drone_swarm
