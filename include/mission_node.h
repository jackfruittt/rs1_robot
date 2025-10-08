/**
 * @file mission_node.h
 * @brief Mission planning and coordination node for autonomous drone operations
 * @author Jackson Russell
 * ADD OTHER AUTHORS ADD HERE AND BELOW
 * @date Aug-2025
 */

#ifndef DRONE_SWARM_MISSION_NODE_H_
#define DRONE_SWARM_MISSION_NODE_H_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"


#include "mission/state_machine.h"
#include "mission/path_planner.h"
#include "mission/mission_executor.h"

namespace drone_swarm
{

// Add this enum (names match what Perception publishes)
enum class Scenario {
  UNKNOWN = 0,
  STRANDED_HIKER,
  WILDFIRE,
  DEBRIS_OBSTRUCTION
};

// Event decoded from the CSV payload
struct ScenarioEvent {
  Scenario type{Scenario::UNKNOWN};
  MissionState state;
  geometry_msgs::msg::Point target{};
  double heading{0.0};          // radians (as published by perception/IMU)
  bool can_respond{false};
  rclcpp::Time stamp{};         // when we received it
  std::string raw;              // original raw string (for audit/logs)
};

/**
 * @class MissionPlannerNode
 * @brief ROS 2 node for autonomous mission planning and execution
 * 
 * Coordinates drone missions including takeoff, waypoint navigation, hovering,
 * and landing. Integrates with state machine, path planner, and drone control
 * systems to provide complete autonomous flight.
 */
class MissionPlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor with configurable node options
   * @param options ROS 2 node options for composition and configuration
   */
  explicit MissionPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Callback methods
  /**
   * @brief Process odometry updates from drone
   * @param msg Odometry message with position and velocity
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  /**
   * @brief Process velocity updates (currently unused)
   * @param msg Velocity message from drone
   */
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  /**
   * @brief Process external waypoint commands
   * @param msg Waypoint pose command for navigation
   */
  void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  /**
   * @brief Main mission timer callback for periodic execution
   */
  void missionTimerCallback();
  
  // Service callback methods
  /**
   * @brief Start mission service callback
   * @param request Service request (empty)
   * @param response Service response with success status and message
   */
  void startMissionCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  /**
   * @brief Stop mission service callback
   * @param request Service request (empty)
   * @param response Service response with success status and message
   */
  void stopMissionCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Mission execution methods
  /**
   * @brief Execute current mission logic based on state machine
   */
  void executeMission();
  
  /**
   * @brief Publish velocity commands for waypoint navigation
   */
  void publishMissionCommand();
  
  /**
   * @brief Load waypoints from ROS parameters (future implementation)
   * 
   *
   */
  void loadWaypointsFromParams();
  
  /**
   * @brief Load waypoints from inside source code (fallback)
   * 
   *
   */
  void loadFallbackWaypoints();

  /**
   * @brief Load mission parameters from ROS 2 parameters
   *
   * This function retrieves mission-specific parameters from the ROS 2 parameter server
   * and updates the mission planner's configuration accordingly.
   */
  void loadMissionParams();

  // Utility methods
  /**
   * @brief Check if current waypoint has been reached
   * @return True if within tolerance of current waypoint
   */
  bool isWaypointReached() const;

  void scenarioDetectionCallback(const std_msgs::msg::String::SharedPtr msg);

  void takeoff(void);
  void waypointNavigation(void);
  void hovering(void);
  void landing(void);
  void manualControl(void);
  void emergency(void); 
  void wildFireReaction(void);

  void alertIncidentGui(const std::optional<ScenarioEvent>& ev);
  const char* evTypeToString(Scenario s) const ;

  // Parser: turns CSV string into a typed ScenarioEvent
  std::optional<ScenarioEvent> parseScenarioDetection(const std_msgs::msg::String& msg);

  // Local string->enum mapper (kept private to this package)
  static Scenario scenarioFromString(const std::string& s);

  // Determine target mission state for a detected scenario
  MissionState targetStateForScenario(Scenario s);

  // ROS 2 communication interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;              ///< Odometry subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;        ///< Velocity subscription (unused)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;  ///< Waypoint command subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scenario_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;            ///< Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;  ///< Target pose publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;          ///< Mission state publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr incident_pub_;               ///< Incident publisher
  
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;       ///< Start mission service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_mission_service_;        ///< Stop mission service
  
  rclcpp::TimerBase::SharedPtr mission_timer_;                                     ///< Periodic mission timer

  // Mission management components
  std::unique_ptr<StateMachine> state_machine_;      ///< Mission state machine
  std::unique_ptr<PathPlanner> path_planner_;        ///< Waypoint path planner
  std::unique_ptr<MissionExecutor> mission_executor_; ///< Advanced mission executor (placeholder)

  // Current state variables
  geometry_msgs::msg::PoseStamped current_pose_;  ///< Current drone pose from odometry
  geometry_msgs::msg::Twist current_velocity_;    ///< Current velocity (currently unused)
  
  // Configuration parameters
  std::string drone_namespace_;   ///< ROS namespace for this drone
  std::string drone_id_;          ///< Unique drone identifier
  double mission_update_rate_;    ///< Mission timer frequency in Hz
  double waypoint_tolerance_;     ///< Waypoint arrival tolerance in metres
  int incident_counter_;
  int drone_numeric_id_;
};

}  // namespace drone_swarm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_swarm::MissionPlannerNode)

#endif  // DRONE_SWARM_MISSION_NODE_H_