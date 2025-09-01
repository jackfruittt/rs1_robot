/**
 * @file drone_node.h
 * @brief Drone controller node for autonomous flight operations
 * @author Jackson Russell
 * ADD AUTHORS HERE AND BELOW
 * @date Aug-2025
 */

#ifndef DRONE_SWARM_DRONE_CONTROLLER_NODE_H_
#define DRONE_SWARM_DRONE_CONTROLLER_NODE_H_

#include <memory>
#include <string>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

// TODO: ADD NAV2 CONTROLLER INCLUDES FOR LOCAL PLANNING AND OBSTACLE AVOIDANCE (OPTIONAL)
// #include "nav2_msgs/action/follow_path.hpp"
// #include "nav2_msgs/msg/speed_limit.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"
// Example: Include nav2 controller interfaces for local planning and dynamic obstacle avoidance

#include "drone_control.h"
// #include "drone/sensor_manager.h"     // Commented out - future implementation
// #include "drone/safety_monitor.h"     // Commented out - future implementation

namespace drone_swarm
{

/**
 * @enum FlightMode
 * @brief Flight mode enumeration for drone operation states
 * 
 * Defines various flight modes for autonomous and manual control.
 * Currently only basic modes are implemented.
 */
enum class FlightMode
{
  DISARMED,       ///< Drone motors disarmed, safe state
  ARMED,          ///< Drone motors armed, ready for flight
  MANUAL,         ///< Manual control mode (future implementation)
  STABILISE,      ///< Stabilised flight mode (future implementation)  
  GUIDED,         ///< Guided autonomous mode
  AUTO,           ///< Full autonomous mode (future implementation)
  EMERGENCY_LAND  ///< Emergency landing mode
};

/**
 * @class DroneControllerNode
 * @brief ROS 2 node for comprehensive drone flight control
 * 
 * Manages drone flight operations including takeoff, waypoint navigation,
 * hovering, and landing. Integrates with mission planner and provides
 * low-level flight control with safety monitoring capabilities.
 */
class DroneControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor with configurable node options
   * @param options ROS 2 node options for composition and configuration
   */
  explicit DroneControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Callback methods
  /**
   * @brief Process target pose commands from mission planner
   * @param msg Target pose for navigation
   */
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  /**
   * @brief Process odometry updates for position control
   * @param msg Odometry message with position and velocity
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  /**
   * @brief Process goal waypoints (currently unused)
   * @param msg Array of goal poses for mission planning
   * 
   * TODO: Implement multi-waypoint mission execution
   */
  void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  
  /**
   * @brief Process IMU data for attitude estimation (currently unused)
   * @param msg IMU sensor data
   * 
   * TODO: Implement attitude-based control
   */
  // void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  /**
   * @brief Process GPS data for global positioning (currently unused)
   * @param msg GPS navigation satellite fix
   * 
   * TODO: Implement GPS-based navigation
   */
  // void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  
  /**
   * @brief Process mission state updates from mission planner
   * @param msg Mission state string message
   */
  void missionStateCallback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Main control loop timer callback
   */
  void controlLoopCallback();
  
  // Service callback methods (currently commented out for future implementation)
  // /**
  //  * @brief Arm drone service callback
  //  * @param request Service request
  //  * @param response Service response with success status
  //  */
  // void armServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  //                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Mission execution methods
  /**
   * @brief Execute takeoff sequence based on mission state
   */
  void executeTakeoffSequence();
  
  /**
   * @brief Execute waypoint navigation control
   */
  void executeWaypointNavigation();
  
  /**
   * @brief Execute landing sequence control
   */
  void executeLandingSequence();
  
  /**
   * @brief Execute hovering position control
   */
  void executeHoverControl();

  // Core control methods
  /**
   * @brief Main control execution logic
   */
  void executeControl();
  
  /**
   * @brief Publish control commands to actuators
   */
  void publishControlOutputs();
  
  /**
   * @brief Publish telemetry data for monitoring
   */
  void publishTelemetry();
  
  /**
   * @brief Load control parameters from ROS parameter server
   */
  void loadControlParams();
  
  // Emergency handling (commented out for future implementation)
  // /**
  //  * @brief Handle emergency situations
  //  * 
  //  * TODO: Implement emergency response procedures
  //  */
  // void handleEmergency();
  
  // Utility methods
  /**
   * @brief Calculate distance to current waypoint
   * @param current Current drone position
   * @param target Target waypoint position
   * @return Euclidean distance in metres
   */
  double calculateDistanceToWaypoint(const geometry_msgs::msg::Pose& current, 
                                    const geometry_msgs::msg::Pose& target) const;
                                    
  /**
   * @brief Convert flight mode enum to string
   * @param mode Flight mode enumeration
   * @return Human-readable flight mode string
   */
  std::string flightModeToString(FlightMode mode) const;

  // ROS 2 communication interfaces
  // Subscriber interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;              ///< Odometry subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_sub_;       ///< Goal waypoints (unused)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_; ///< Target pose subscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;                 ///< IMU data (unused)
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;           ///< GPS data (unused)
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;         ///< Laser scan (unused)
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;             ///< Sonar range (unused)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_sub_;       ///< Mission state subscription

  // TODO: ADD NAV2 CONTROLLER SUBSCRIPTIONS FOR LOCAL PLANNING (OPTIONAL)
  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr nav2_cmd_vel_sub_; ///< Nav2 velocity commands
  // rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;        ///< Dynamic speed limits

  // Publisher interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;            ///< Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;         ///< Current pose publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;           ///< Current velocity publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flight_mode_pub_;            ///< Flight mode publisher

  // Service interfaces (commented out for future implementation)
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;              ///< Arm service
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;           ///< Disarm service
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_;        ///< Emergency service

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;                                     ///< Main control timer

  // Control and sensor management components
  std::unique_ptr<DroneControl> drone_control_;                                    ///< Low-level drone control
  // std::unique_ptr<SensorManager> sensor_manager_;                               ///< Sensor data management (future)
  // std::unique_ptr<SafetyMonitor> safety_monitor_;                               ///< Safety monitoring (future)

  // Current state variables
  geometry_msgs::msg::PoseStamped target_pose_;    ///< Current target pose from mission planner
  nav_msgs::msg::Odometry current_odom_;           ///< Current odometry data
  FlightMode current_flight_mode_;                 ///< Current flight mode
  std::string current_mission_state_;              ///< Current mission state from planner
  
  // Waypoint navigation (currently unused - future implementation)
  // std::vector<geometry_msgs::msg::PoseStamped> current_waypoints_;  ///< Waypoint sequence
  // size_t current_waypoint_index_;                                   ///< Current waypoint index
  
  // Mission timing variables
  rclcpp::Time takeoff_start_time_;                ///< Takeoff sequence start time
  rclcpp::Time landing_start_time_;                ///< Landing sequence start time
  
  // Configuration parameters
  std::string drone_namespace_;                    ///< ROS namespace for this drone
  double control_frequency_;                       ///< Control loop frequency in Hz
  double telemetry_frequency_;                     ///< Telemetry publishing frequency in Hz
  bool armed_;                                     ///< Drone armed status
  std::chrono::steady_clock::time_point last_control_update_;  ///< Last control update timestamp
};

}  // namespace drone_swarm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_swarm::DroneControllerNode)

#endif  // DRONE_SWARM_DRONE_CONTROLLER_NODE_H_