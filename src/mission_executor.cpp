#include "mission/mission_executor.h"
#include "drone_control.h"

namespace drone_swarm
{

MissionExecutor::MissionExecutor() 
  : logger_(rclcpp::get_logger("mission_executor")), mission_cancelled_(false) {
  // Initialise flight controller for mission execution
  flight_controller_ = std::make_unique<DroneControl>();
  
  // Initialise saved state
  saved_state_.valid = false;
  saved_state_.current_waypoint_index = 0;
}

void MissionExecutor::setLogger(rclcpp::Logger logger) {
  logger_ = logger;
  if (flight_controller_) {
    flight_controller_->setLogger(logger);
  }
}

void MissionExecutor::setCmdVelPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub) {
  cmd_vel_pub_ = pub;
  if (flight_controller_) {
    flight_controller_->setCmdVelPublisher(pub);
  }
}

bool MissionExecutor::isMissionCancelled() const {
  return mission_cancelled_;
}

void MissionExecutor::cancelMission() {
  mission_cancelled_ = true;
  RCLCPP_INFO(logger_, "Mission cancelled by executor");
}

void MissionExecutor::reset() {
  mission_cancelled_ = false;
  if (flight_controller_) {
    flight_controller_->resetControllers();
  }
  saved_state_.valid = false;
  saved_state_.waypoints.clear();
  saved_state_.current_waypoint_index = 0;
  RCLCPP_INFO(logger_, "Mission executor reset for new mission");
}

void MissionExecutor::saveMissionState(WaypointPlanner* path_planner) {
  if (!path_planner) {
    RCLCPP_WARN(logger_, "Cannot save mission state - null path planner");
    saved_state_.valid = false;
    return;
  }
  
  saved_state_.waypoints = path_planner->getAllWaypoints();
  saved_state_.current_waypoint_index = path_planner->getCurrentWaypointIndex();
  saved_state_.valid = true;
  
  RCLCPP_INFO(logger_, "Mission state saved: %zu waypoints, current index: %zu",
              saved_state_.waypoints.size(), saved_state_.current_waypoint_index);
}

bool MissionExecutor::restoreMissionState(WaypointPlanner* path_planner) {
  if (!saved_state_.valid) {
    RCLCPP_WARN(logger_, "No valid saved state to restore");
    return false;
  }
  
  if (!path_planner) {
    RCLCPP_ERROR(logger_, "Cannot restore mission state - null path planner");
    return false;
  }
  
  path_planner->setWaypoints(saved_state_.waypoints);
  path_planner->setCurrentWaypointIndex(saved_state_.current_waypoint_index);
  
  RCLCPP_INFO(logger_, "Mission state restored: %zu waypoints, resuming from index: %zu",
              saved_state_.waypoints.size(), saved_state_.current_waypoint_index);
  
  return true;
}

bool MissionExecutor::executeWildfireReaction(
    const geometry_msgs::msg::Point& fire_location,
    const geometry_msgs::msg::Point& depot_location,
    const geometry_msgs::msg::Point& helipad_location,
    WaypointPlanner* path_planner) {
  
  if (!path_planner) {
    RCLCPP_ERROR(logger_, "WaypointPlanner is null in executeWildfireReaction");
    return false;
  }
  
  RCLCPP_INFO(logger_, "=== WILDFIRE REACTION MISSION STARTED ===");
  RCLCPP_INFO(logger_, "  Fire location: [%.2f, %.2f, %.2f]",
              fire_location.x, fire_location.y, fire_location.z);
  RCLCPP_INFO(logger_, "  Depot location: [%.2f, %.2f, %.2f]",
              depot_location.x, depot_location.y, depot_location.z);
  RCLCPP_INFO(logger_, "  Helipad location: [%.2f, %.2f, %.2f]",
              helipad_location.x, helipad_location.y, helipad_location.z);
  
  if (mission_cancelled_) {
    RCLCPP_WARN(logger_, "Wildfire reaction cancelled before execution");
    return false;
  }
  
  // Create waypoint sequence for wildfire response mission
  std::vector<geometry_msgs::msg::PoseStamped> mission_waypoints;
  
  // Waypoint 1: Navigate to depot (to collect fire retardant)
  // Note: Even if drone is already at depot, going to this waypoint represents
  // the "collection" action. In a full implementation, this would trigger
  // landing, waiting for payload loading, and takeoff.
  geometry_msgs::msg::PoseStamped wp_depot;
  wp_depot.header.frame_id = "map";
  wp_depot.header.stamp = rclcpp::Time(0);
  wp_depot.pose.position = depot_location;
  wp_depot.pose.orientation.w = 1.0;  // Identity quaternion
  mission_waypoints.push_back(wp_depot);
  
  // Waypoint 2: Navigate to fire location (to deploy retardant)
  // Offset by 1 meter to avoid re-detecting the fire tag
  geometry_msgs::msg::PoseStamped wp_fire;
  wp_fire.header.frame_id = "map";
  wp_fire.header.stamp = rclcpp::Time(0);
  wp_fire.pose.position.x = fire_location.x + 1.0;  // 1m offset in x
  wp_fire.pose.position.y = fire_location.y;
  wp_fire.pose.position.z = fire_location.z;
  wp_fire.pose.orientation.w = 1.0;
  mission_waypoints.push_back(wp_fire);
  
  // Waypoint 3: Return to helipad (mission complete)
  geometry_msgs::msg::PoseStamped wp_helipad;
  wp_helipad.header.frame_id = "map";
  wp_helipad.header.stamp = rclcpp::Time(0);
  wp_helipad.pose.position = helipad_location;
  wp_helipad.pose.orientation.w = 1.0;
  mission_waypoints.push_back(wp_helipad);
  
  // Set the mission waypoints - this will be executed by mission_node's waypoint navigation
  path_planner->setWaypoints(mission_waypoints);
  
  RCLCPP_INFO(logger_, "Wildfire reaction waypoints configured:");
  RCLCPP_INFO(logger_, "  1. Depot [%.2f, %.2f, %.2f] - Collect fire retardant",
              depot_location.x, depot_location.y, depot_location.z);
  RCLCPP_INFO(logger_, "  2. Fire [%.2f, %.2f, %.2f] - Deploy retardant",
              fire_location.x, fire_location.y, fire_location.z);
  RCLCPP_INFO(logger_, "  3. Helipad [%.2f, %.2f, %.2f] - Return to base",
              helipad_location.x, helipad_location.y, helipad_location.z);
  RCLCPP_INFO(logger_, "=== Mission waypoints set, execution delegated to waypoint navigation ===");
  
  // Note: The actual flight execution happens in mission_node.cpp's executeMission()
  // through the WAYPOINT_NAVIGATION state. This method just sets up the waypoints.
  // Landing at depot and takeoff after collection would need to be handled
  // by additional mission phases or state flags in mission_node.
  
  return true;
}

bool MissionExecutor::executeHikerRescue(
    const geometry_msgs::msg::Point& hiker_location,
    const geometry_msgs::msg::Point& depot_location,
    WaypointPlanner* path_planner) {
  
  if (!path_planner) {
    RCLCPP_ERROR(logger_, "WaypointPlanner is null in executeHikerRescue");
    return false;
  }
  
  RCLCPP_INFO(logger_, "=== HIKER RESCUE MISSION STARTED ===");
  RCLCPP_INFO(logger_, "  Hiker location: [%.2f, %.2f, %.2f]",
              hiker_location.x, hiker_location.y, hiker_location.z);
  RCLCPP_INFO(logger_, "  Medkit depot location: [%.2f, %.2f, %.2f]",
              depot_location.x, depot_location.y, depot_location.z);
  
  if (mission_cancelled_) {
    RCLCPP_WARN(logger_, "Hiker rescue cancelled before execution");
    return false;
  }
  
  // Create waypoint sequence for hiker rescue mission
  std::vector<geometry_msgs::msg::PoseStamped> mission_waypoints;
  
  // Waypoint 1: Navigate to medkit depot (to collect medical supplies)
  geometry_msgs::msg::PoseStamped wp_depot;
  wp_depot.header.frame_id = "map";
  wp_depot.header.stamp = rclcpp::Time(0);
  wp_depot.pose.position = depot_location;
  wp_depot.pose.orientation.w = 1.0;
  mission_waypoints.push_back(wp_depot);
  
  // Waypoint 2: Navigate to hiker location (to deliver medkit)
  geometry_msgs::msg::PoseStamped wp_hiker;
  wp_hiker.header.frame_id = "map";
  wp_hiker.header.stamp = rclcpp::Time(0);
  wp_hiker.pose.position = hiker_location;
  wp_hiker.pose.orientation.w = 1.0;
  mission_waypoints.push_back(wp_hiker);
  
  // Set the mission waypoints
  path_planner->setWaypoints(mission_waypoints);
  
  RCLCPP_INFO(logger_, "Hiker rescue waypoints configured:");
  RCLCPP_INFO(logger_, "  1. Depot [%.2f, %.2f, %.2f] - Collect medkit",
              depot_location.x, depot_location.y, depot_location.z);
  RCLCPP_INFO(logger_, "  2. Hiker [%.2f, %.2f, %.2f] - Deliver medkit",
              hiker_location.x, hiker_location.y, hiker_location.z);
  RCLCPP_INFO(logger_, "=== Mission waypoints set, execution delegated to waypoint navigation ===");
  
  return true;
}

bool MissionExecutor::executeDebrisReaction(
    const geometry_msgs::msg::Point& debris_location,
    WaypointPlanner* path_planner) {
  
  if (!path_planner) {
    RCLCPP_ERROR(logger_, "WaypointPlanner is null in executeDebrisReaction");
    return false;
  }
  
  RCLCPP_INFO(logger_, "=== DEBRIS INSPECTION MISSION STARTED ===");
  RCLCPP_INFO(logger_, "  Debris location: [%.2f, %.2f, %.2f]",
              debris_location.x, debris_location.y, debris_location.z);
  
  // Generate circular orbit waypoints for debris inspection
  // 5m radius orbit with 8 waypoints (45° intervals) for comprehensive view
  const double ORBIT_RADIUS = 5.0;  // meters
  const int ORBIT_POINTS = 8;       // waypoints around circle
  
  std::vector<geometry_msgs::msg::PoseStamped> orbit_waypoints;
  orbit_waypoints.reserve(ORBIT_POINTS);
  
  constexpr double PI = 3.14159265358979323846;
  const double angle_step = 2.0 * PI / static_cast<double>(ORBIT_POINTS);
  
  for (int i = 0; i < ORBIT_POINTS; ++i) {
    const double theta = angle_step * static_cast<double>(i);
    
    // Calculate position on circle around debris
    const double px = debris_location.x + ORBIT_RADIUS * std::cos(theta);
    const double py = debris_location.y + ORBIT_RADIUS * std::sin(theta);
    
    // Calculate yaw to face the debris (centre point)
    double yaw = std::atan2(debris_location.y - py, debris_location.x - px);
    
    // Convert yaw to quaternion (rotation around Z axis)
    const double half_yaw = yaw * 0.5;
    const double qw = std::cos(half_yaw);
    const double qz = std::sin(half_yaw);
    
    // Create waypoint
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = "map";
    waypoint.header.stamp = rclcpp::Time(0);
    waypoint.pose.position.x = px;
    waypoint.pose.position.y = py;
    waypoint.pose.position.z = debris_location.z;  // Same altitude as debris
    waypoint.pose.orientation.w = qw;
    waypoint.pose.orientation.x = 0.0;
    waypoint.pose.orientation.y = 0.0;
    waypoint.pose.orientation.z = qz;
    
    orbit_waypoints.push_back(waypoint);
  }
  
  // Set the orbit waypoints for execution
  path_planner->setWaypoints(orbit_waypoints);
  
  RCLCPP_INFO(logger_, "Debris inspection waypoints configured:");
  RCLCPP_INFO(logger_, "  Orbit radius: %.1fm with %d waypoints", ORBIT_RADIUS, ORBIT_POINTS);
  RCLCPP_INFO(logger_, "  Drone will orbit debris for visual inspection");
  RCLCPP_INFO(logger_, "=== Mission waypoints set, execution delegated to waypoint navigation ===");
  
  return true;
}

}  // namespace drone_swarm
