/**
 * @file mission_node.h
 * @brief Mission planning and coordination node for autonomous drone operations
 * @author Jackson Russell
 * @author Matthew Chua
 * ADD OTHER AUTHORS ADD HERE AND BELOW
 * @date Aug-2025
 */

#ifndef DRONE_SWARM_MISSION_NODE_H_
#define DRONE_SWARM_MISSION_NODE_H_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <regex>   
#include <set>     
#include <mutex>   
#include <unordered_map>
#include <optional>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <std_msgs/msg/empty.hpp>

// Service includes for scenario reactions
#include "rs1_robot/srv/react_to_wildfire.hpp"
#include "rs1_robot/srv/react_to_hiker.hpp"
#include "rs1_robot/srv/react_to_debris.hpp"

#include "mission/state_machine.h"
#include "mission/waypoint_planner.h"
#include "mission/mission_state.h"
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

enum class ReactionPhase { 
  NONE, 
  INIT, 
  COLLECT_INFO, 
  ASSIGN_PEER, 
  SELF_ORBIT, 
  COMPLETE 
};

struct DroneInfo {
  int drone_id;
  double x, y, z;
  double battery_level;
  std::string mission_state;
  rclcpp::Time timestamp;
  bool valid;
};

struct DispatchCooldown {
  rclcpp::Time until;                 // steady time
  geometry_msgs::msg::Point target;   // last dispatched target
  int responder_id{-1};
};

struct OrbitPoint
{
  float x;
  float y;
  float z;
  float yaw; // radians, heading toward the centre (cx, cy)
};

/**
 * @brief Simple data structure to hold parsed scenario information
 * 
 * Contains all the information from a scenario detection message
 * in an easy-to-use format.
 */
struct ScenarioData {
  std::string scenario_name;  // e.g., "STRANDED_HIKER", "WILDFIRE", "DEBRIS_OBSTRUCTION"
  int severity;               // Severity level (1-10)
  double x;                   // X position in world coordinates (metres)
  double y;                   // Y position in world coordinates (metres)
  double z;                   // Z position (altitude) in metres
  double yaw;                 // Heading in radians
  bool can_respond;           // Whether drone should respond to this scenario
  bool valid;                 // Whether parsing was successful
};

// Info known about a peer (extensible)
struct PeerInfo {
  double battery{0.0};
  MissionState state{MissionState::IDLE};
  geometry_msgs::msg::PoseStamped pose{};
  rclcpp::Time stamp{};
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

struct ActiveIncident {
  std::string scenario_name;
  geometry_msgs::msg::Point location;
  int responder_id;
  rclcpp::Time dispatch_time;
  rclcpp::Time expires_at;
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
  explicit MissionPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), const std::string & name = "");

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
   * @brief Sonar sensor callback for altitude measurement during takeoff
   * @param msg Range sensor data for altitude measurement
   */
  void sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  
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
    
  /**
   * @brief Takeoff drone service callback
   * @param request Service request (empty)
   * @param response Service response with success status and message
   */
  void takeoffDroneCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  /**
   * @brief Landing drone service callback
   * @param request Service request (empty)
   * @param response Service response with success status and message
   */
  void landDroneCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  /**
   * @brief Wildfire reaction service callback
   * @param request Service request with fire and depot locations
   * @param response Service response with success status and completion time
   * 
   * Handles wildfire response mission: Navigate to depot, collect retardant,
   * deploy at fire location, then return to previous mission state
   */
  void wildfireReactionCallback(
    const std::shared_ptr<rs1_robot::srv::ReactToWildfire::Request> request,
    std::shared_ptr<rs1_robot::srv::ReactToWildfire::Response> response);
  
  /**
   * @brief Hiker rescue service callback
   * @param request Service request with hiker and depot locations
   * @param response Service response with success status and completion time
   * 
   * Handles hiker rescue mission: Navigate to depot, collect medkit,
   * deliver to hiker location, then return to previous mission state
   */
  void hikerRescueCallback(
    const std::shared_ptr<rs1_robot::srv::ReactToHiker::Request> request,
    std::shared_ptr<rs1_robot::srv::ReactToHiker::Response> response);
  
  /**
   * @brief Debris notification service callback
   * @param request Service request with debris location
   * @param response Service response with success status
   * 
   * Handles debris obstruction notification (logging only, no active response)
   */
  void debrisReactionCallback(
    const std::shared_ptr<rs1_robot::srv::ReactToDebris::Request> request,
    std::shared_ptr<rs1_robot::srv::ReactToDebris::Response> response);

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

  /**
   * Generate N points on a circle of radius `radius` in the XY plane centred at (x,y),
   * at altitude z. Each point's yaw faces the centre (x,y).
   *
   * @param x          centre X (metres)
   * @param y          centre Y (metres)
   * @param z          altitude Z (metres)
   * @param radius     circle radius (metres), negative treated as |radius|
   * @param pointCount number of points to produce (min 1)
   * @return           vector of OrbitPoint {X,Y,Z,Yaw}
   */
  std::vector<OrbitPoint> orbitTrajectory(float x, float y, float z,
                                          float radius, int pointCount);

  /**
   * @brief Normalises an angle
   * @return Normalised angle
   */
  inline float normalizeAngle(float a);

  void scenarioDetectionCallback(const std_msgs::msg::String::SharedPtr msg);

  // Missions
  /**
   * @brief Execute takeoff sequence using sonar feedback
   * 
   * Uses sonar sensor to monitor altitude and climb to target_takeoff_altitude_ (5m).
   * Publishes climb commands and transitions to WAYPOINT_NAVIGATION or HOVERING when complete.
   * Must be initiated via takeoffDroneCallback service call.
   */
  void takeoff(void);
  void waypointNavigation(void);
  void hovering(void);
  /**
   * @brief Execute landing sequence using sonar feedback
   * 
   * Uses sonar sensor to monitor altitude and descend to target_landing_altitude_ (0.2m).
   * Publishes descent commands and transitions to IDLE when complete.
   * Must be initiated via landDroneCallback service call.
   */
  void landing(void);
  void manualControl(void);
  void emergency(void); 
  void wildFireReaction(void);
  void debrisReaction(void);
  void strandedHikerReaction(void);
  void orbitIncident(void);
  void alertIncidentGui(const std::optional<ScenarioEvent>& ev);
  const char* evTypeToString(Scenario s) const ;

  void discoverPeerDrones(void);                  // scans get_topic_names_and_types(), extracts numeric ids, adds/removes peers
  void createPeerSubscriptionForId(int id);       // create odom subscription + cached assignment publisher for drone id
  void removePeerSubscriptionForId(int id);       // tear down subscription/publisher and cached state
  void infoRequestPingCallback(const std_msgs::msg::Empty::SharedPtr msg);  // Will send a csv of required drone information back to the management drone
  void resetMissioncallback(const std_msgs::msg::String::SharedPtr msg); // Allows a user to reset a drone's mission after reacting to a scenario
  std::string buildInfoManifestCsv(void);         // Helper for infoRequestPingCallback
  int findClosestPeerToOrigin(void) const;
  // Callback for receiving mission assignments from other drones
  void assignmentCallback(const std_msgs::msg::String::SharedPtr msg);

  // --- Mission Assignment State Flags ---
  // For STRANDED_HIKER fetch-and-deliver
  bool in_hiker_rescue_{false};
  bool medkit_collected_{false};
  bool in_hiker_rescue_awaiting_takeoff_{false}; // Prevents state machine loop
  rclcpp::Time medkit_collect_stamp_;
  geometry_msgs::msg::Point hiker_target_xyz_{};
  geometry_msgs::msg::Point medkit_depot_xyz_{}; // Loaded from params

  static bool parseKeyVal(const std::string& tok, std::string& key, std::string& val);
  static MissionState stateFromString(const std::string& s);


  DroneInfo parseInfoManifest(const std::string& manifest_data);
  std::map<int, DroneInfo> pingDronesForInfo(const std::vector<int>& drone_ids);
  int selectLowestDronId();
  void performCoordination(const ScenarioData& scenario);

  // Service-based coordination helpers
  void callWildfireService(int responder_id, const ScenarioData& scenario, const std::string& incident_id);
  void callHikerService(int responder_id, const ScenarioData& scenario, const std::string& incident_id);
  void callDebrisService(int responder_id, const ScenarioData& scenario, const std::string& incident_id);
  rclcpp::Client<rs1_robot::srv::ReactToWildfire>::SharedPtr getOrCreateWildfireClient(int drone_id);
  rclcpp::Client<rs1_robot::srv::ReactToHiker>::SharedPtr getOrCreateHikerClient(int drone_id);
  rclcpp::Client<rs1_robot::srv::ReactToDebris>::SharedPtr getOrCreateDebrisClient(int drone_id);

  // Parser: turns CSV string into a typed ScenarioEvent
  std::optional<ScenarioEvent> parseScenarioDetection(const std_msgs::msg::String& msg);

  // Local string->enum mapper (kept private to this package)
  static Scenario scenarioFromString(const std::string& s);

  // Determine target mission state for a detected scenario
  MissionState targetStateForScenario(Scenario s);  
  
  //--- ROS 2 communication interfaces ---///
  // SUBS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;              ///< Odometry subscription
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;            ///< Sonar subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;        ///< Velocity subscription (unused)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;  ///< Waypoint command subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scenario_sub_;            ///< For perception to send scenario
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr info_request_sub_;         ///< For management drones to contact other drones
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr assignment_subs_;         ///< Allows management drones to set the state of other drones
  std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> info_manifest_subs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr incident_dispatch_sub_;   /// "DISPATCH,<incident_id>,<scenario_name>,<x>,<y>,<z>,<responder_id>,<timestamp>" Example: "DISPATCH,INC-001,WILDFIRE,10.5,5.2,2.1,3,1234567890"
  
  // PUBS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;            ///< Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;  ///< Target pose publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;          ///< Mission state publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr incident_pub_;               ///< Incident publisher (GUI)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_manifest_pub_;          ///< Info manifest publisher for updating management drones
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr info_request_pub_;            ///< Info request for management drones to ping
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> info_request_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr incident_dispatch_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;                 ///< Takeoff command publisher
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;                    ///< Landing command publisher
  
  // SRV
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;       ///< Start mission service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_mission_service_;        ///< Stop mission service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_drone_service_;       ///< Takeoff drone service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_drone_service_;          ///< Landing drone service
  
  // Scenario reaction services
  rclcpp::Service<rs1_robot::srv::ReactToWildfire>::SharedPtr wildfire_service_;   ///< Wildfire reaction service
  rclcpp::Service<rs1_robot::srv::ReactToHiker>::SharedPtr hiker_service_;         ///< Hiker rescue service
  rclcpp::Service<rs1_robot::srv::ReactToDebris>::SharedPtr debris_service_;       ///< Debris notification service
  
  // Scenario reaction service clients (for calling our own services or peers')
  rclcpp::Client<rs1_robot::srv::ReactToWildfire>::SharedPtr wildfire_client_;     ///< Wildfire reaction client
  rclcpp::Client<rs1_robot::srv::ReactToHiker>::SharedPtr hiker_client_;           ///< Hiker rescue client
  rclcpp::Client<rs1_robot::srv::ReactToDebris>::SharedPtr debris_client_;         ///< Debris notification client
  
  // Per-peer service clients (created dynamically as peers are discovered)
  std::map<int, rclcpp::Client<rs1_robot::srv::ReactToWildfire>::SharedPtr> peer_wildfire_clients_;
  std::map<int, rclcpp::Client<rs1_robot::srv::ReactToHiker>::SharedPtr> peer_hiker_clients_;
  std::map<int, rclcpp::Client<rs1_robot::srv::ReactToDebris>::SharedPtr> peer_debris_clients_;
  
  // TIM
  rclcpp::TimerBase::SharedPtr mission_timer_;                                     ///< Periodic mission timer
  rclcpp::TimerBase::SharedPtr waypoint_load_timer_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr mission_params_timer_;
  rclcpp::TimerBase::SharedPtr status_broadcast_timer_;  ///< Periodic status broadcast for peer discovery
  rclcpp::TimerBase::SharedPtr tiebreaker_timer_;        ///< One-shot timer for tie-breaker delay

  // Mission management components
  std::unique_ptr<StateMachine> state_machine_;       ///< Mission state machine
  std::unique_ptr<WaypointPlanner> path_planner_;         ///< Waypoint path planner
  std::unique_ptr<MissionExecutor> mission_executor_; ///< Advanced mission executor (placeholder)

  // Current state variables
  geometry_msgs::msg::PoseStamped current_pose_;  ///< Current drone pose from odometry
  geometry_msgs::msg::Twist current_velocity_;    ///< Current velocity (currently unused)
  
  // Takeoff control variables
  double current_sonar_range_;                     ///< Current sonar reading in metres
  sensor_msgs::msg::LaserScan current_lidar_data_; ///< Current LiDAR scan for obstacle detection
  std::mutex lidar_mutex_;                         ///< Mutex for LiDAR data protection
  bool takeoff_in_progress_;                       ///< Flag indicating if takeoff is in progress
  std::chrono::steady_clock::time_point takeoff_start_time_; ///< Time when takeoff started
  double target_takeoff_altitude_;                 ///< Target altitude for takeoff (4.0m)
  bool takeoff_complete_;                          ///< Flag indicating takeoff completion
  
  // Landing control variables
  bool landing_in_progress_;                       ///< Flag indicating if landing is in progress
  std::chrono::steady_clock::time_point landing_start_time_; ///< Time when landing started
  double target_landing_altitude_;                 ///< Target altitude for landing (0.2m)
  bool landing_complete_;                          ///< Flag indicating landing completion
  
  mutable std::mutex sonar_mutex_;                 ///< Mutex for sonar data protection

  // Variables for drone management and collaboration
  mutable std::mutex peers_mutex_;
  std::unordered_map<int, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_odom_subs_;
  std::unordered_map<int, geometry_msgs::msg::PoseStamped> peer_poses_;
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> assignment_pubs_;
  std::unordered_map<int, PeerInfo> peer_info_;
  mutable std::mutex incident_mutex_;
  std::optional<ScenarioEvent> active_incident_event_;
  std::vector<geometry_msgs::msg::PoseStamped> route_waypoints_cached_;
  std::vector<geometry_msgs::msg::PoseStamped> original_waypoints_cached_;
  std::mutex cache_mutex_; // mutex for saving the waypoints

  
  // Configuration parameters
  std::string drone_namespace_;   ///< ROS namespace for this drone
  std::string drone_id_;          ///< Unique drone identifier
  double mission_update_rate_;    ///< Mission timer frequency in Hz
  double waypoint_tolerance_;     ///< Waypoint arrival tolerance in metres
  int incident_counter_;
  int drone_numeric_id_;

  ReactionPhase wildfire_phase_{ReactionPhase::NONE};
  rclcpp::Time   wildfire_phase_start_;
  std::optional<int> assigned_peer_id_;

  double battery_level_{0.8};                 // our own (stubbed via param)
  int    collect_window_ms_{400};             // reply window
  geometry_msgs::msg::Point depot_xyz_{};     // retardant depot (defaults to 0,0,2) 
  
  bool is_coordinating_ = false;
  std::optional<ScenarioData> active_coordination_scenario_;
  std::mutex coordination_mutex_;
  
  // Flag to indicate if currently executing a scenario reaction mission
  bool in_scenario_reaction_ = false;
  std::string active_scenario_type_;  // "WILDFIRE", "STRANDED_HIKER", etc.
  std::string active_scenario_incident_id_;  // Track the incident ID for resolution
  bool was_idle_before_reaction_ = false;  // Track if drone was IDLE before scenario mission
  bool payload_collected_ = false;  // Track if retardant/medkit has been collected from depot

  geometry_msgs::msg::Pose hover_hold_pose_;

  bool repeat_Waypoint_Path_ = false; // to determine whether a drone is stationary or starts waypoints over again


  mutable std::mutex dispatch_mutex_;
  std::unordered_map<std::string, DispatchCooldown> dispatch_cooldown_;

  // steady clock just for gating/cooldowns
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Duration coordination_cooldown_{std::chrono::seconds(5)};
  double incident_merge_radius_m_{7.0};  // 7m radius for 2D (X,Y) incident matching, ignoring Z noise

  std::map<std::string, ActiveIncident> fleet_incident_registry_;
  std::mutex registry_mutex_;

  // Recently resolved incidents - prevent re-detection after resuming patrol
  struct ResolvedIncident {
    geometry_msgs::msg::Point location;
    std::string scenario_name;
    rclcpp::Time resolved_at;
  };
  std::vector<ResolvedIncident> recently_resolved_incidents_;
  std::mutex resolved_incidents_mutex_;
  rclcpp::Duration resolved_incident_ignore_duration_{std::chrono::seconds(1200)};  // Ignore for 20 minutes
  double resolved_incident_match_radius_{15.0};  // 15m radius for matching resolved incidents

  // 14 OCT
  geometry_msgs::msg::Point helipad_location_;
  // Intelligent drone selection
  int selectBestResponderDrone(const std::vector<int>& all_drone_ids,
                              MissionState required_state,
                              const geometry_msgs::msg::Point& incident_xyz);

  void infoManifestCallback(int peer_id, const std_msgs::msg::String::SharedPtr& msg);
  bool waitForPeerMatch(int id, std::chrono::milliseconds max_wait) ;
  bool waitForPeerPingSubscriber(int peer_id, std::chrono::milliseconds timeout);
  bool shouldSuppressIncident(const ScenarioData& s);
  inline bool isBusyWithAssignedMission() const {
    return in_hiker_rescue_;
  }

  bool isIncidentAlreadyManaged(const ScenarioData& scenario);
  void incidentDispatchCallback(const std_msgs::msg::String::SharedPtr msg);
  std::string generateIncidentId(const ScenarioData& scenario);
  void recordIncidentDispatch(const std::string& incident_id, const ScenarioData& scenario, int responder_id);
    
  std::vector<int> getKnownDroneIds(void);
};


}  // namespace drone_swarm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_swarm::MissionPlannerNode)

#endif  // DRONE_SWARM_MISSION_NODE_H_