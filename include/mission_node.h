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

// TODO: ADD NAV2 RRT PLANNER INCLUDES FOR AUTONOMOUS PATH PLANNING
// #include "nav2_msgs/action/compute_path_to_pose.hpp"
// #include "nav2_msgs/msg/costmap.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_util/simple_action_server.hpp"
// These headers provide access to nav2's path planning services and action interfaces

#include "mission/state_machine.h"
#include "mission/path_planner.h"
#include "mission/mission_executor.h"

namespace drone_swarm
{

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
  
  // TODO: ADD NAV2 RRT PLANNER CALLBACK METHODS FOR AUTONOMOUS PATH PLANNING
  // /**
  //  * @brief Handle path planning result from nav2 RRT planner
  //  * @param goal_handle Action goal handle for the planning request
  //  */
  // void pathPlanningResultCallback(
  //   const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult& result);
  // 
  // /**
  //  * @brief Handle map/costmap updates for replanning
  //  * @param msg Updated costmap from nav2 stack
  //  */
  // void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  // These callbacks will handle results from autonomous path planning requests
  
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
   * TODO: Implement parameter-based waypoint loading for pre-planned missions
   */
  void loadWaypointsFromParams();
  
  // TODO: ADD NAV2 RRT PLANNER EXECUTION METHODS FOR AUTONOMOUS PATH PLANNING
  // /**
  //  * @brief Request path planning from nav2 RRT planner
  //  * @param start_pose Starting pose for path planning
  //  * @param goal_pose Target goal pose for navigation
  //  * @return True if planning request was successfully sent
  //  */
  // bool requestPathPlanning(const geometry_msgs::msg::PoseStamped& start_pose,
  //                         const geometry_msgs::msg::PoseStamped& goal_pose);
  // 
  // /**
  //  * @brief Convert nav2 path to waypoints for PathPlanner
  //  * @param path Planned path from nav2 RRT planner
  //  */
  // void convertPathToWaypoints(const nav_msgs::msg::Path& path);
  // 
  // /**
  //  * @brief Check if replanning is needed due to obstacles or goal changes
  //  * @return True if new path planning is required
  //  */
  // bool needsReplanning() const;
  // These methods will handle autonomous path planning without manual waypoints
  
  // Utility methods
  /**
   * @brief Check if current waypoint has been reached
   * @return True if within tolerance of current waypoint
   */
  bool isWaypointReached() const;

  // ROS 2 communication interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;              ///< Odometry subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;        ///< Velocity subscription (unused)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;  ///< Waypoint command subscription
  
  // TODO: ADD NAV2 RRT PLANNER SUBSCRIPTIONS FOR AUTONOMOUS PATH PLANNING
  // rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;        ///< Costmap updates subscription
  // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;       ///< Map updates subscription
  // Example: Subscribe to costmap and map updates to trigger replanning when environment changes
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;            ///< Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;  ///< Target pose publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;          ///< Mission state publisher
  
  // TODO: ADD NAV2 RRT PLANNER ACTION CLIENT FOR AUTONOMOUS PATH PLANNING
  // using ComputePathToPoseClient = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>;
  // ComputePathToPoseClient::SharedPtr path_planner_client_;                      ///< Nav2 path planner action client
  // Example: This client will send path planning requests to the nav2 RRT planner server
  
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
  
  // TODO: ADD NAV2 RRT PLANNER STATE VARIABLES FOR AUTONOMOUS PATH PLANNING
  // geometry_msgs::msg::PoseStamped mission_goal_;                    ///< Current mission goal pose
  // nav_msgs::msg::Path current_planned_path_;                        ///< Currently planned path from RRT
  // bool path_planning_active_;                                       ///< Flag indicating if planning is in progress
  // rclcpp::Time last_replan_time_;                                   ///< Timestamp of last replanning request
  // Example: These variables will track the autonomous planning state and current path
  
  // Configuration parameters
  std::string drone_namespace_;   ///< ROS namespace for this drone
  std::string drone_id_;          ///< Unique drone identifier
  double mission_update_rate_;    ///< Mission timer frequency in Hz
  double waypoint_tolerance_;     ///< Waypoint arrival tolerance in metres
  
  // TODO: ADD NAV2 RRT PLANNER CONFIGURATION PARAMETERS
  // double replanning_frequency_;                                     ///< How often to check for replanning needs (Hz)
  // double planning_timeout_;                                         ///< Timeout for path planning requests (seconds)
  // std::string planner_id_;                                          ///< RRT planner plugin ID (e.g., "RRT*")
  // Example: Configure planning parameters like timeout, planner type, and replanning frequency
};

}  // namespace drone_swarm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_swarm::MissionPlannerNode)

#endif  // DRONE_SWARM_MISSION_NODE_H_