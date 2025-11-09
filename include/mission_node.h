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

/**
 * @brief Scenario types that can be detected during missions
 * 
 * Matches scenario names published by the perception system
 */
enum class Scenario {
  UNKNOWN = 0,              ///< Unknown or unrecognised scenario type
  STRANDED_HIKER,          ///< Stranded hiker requiring rescue
  WILDFIRE,                ///< Wildfire requiring suppression
  DEBRIS_OBSTRUCTION       ///< Debris obstruction requiring inspection
};

/**
 * @brief Reaction phase for scenario response coordination
 * 
 * Tracks the progression of multi-drone coordination for scenario responses
 */
enum class ReactionPhase { 
  NONE,           ///< No active reaction
  INIT,           ///< Initialisation phase
  COLLECT_INFO,   ///< Collecting information from peer drones
  ASSIGN_PEER,    ///< Assigning peer drone for response
  SELF_ORBIT,     ///< Self-executing orbital inspection
  COMPLETE        ///< Reaction complete
};

/**
 * @brief Information about a drone in the fleet
 * 
 * Contains state, position, and capability information for coordination
 */
struct DroneInfo {
  int drone_id;                    ///< Numeric drone identifier
  double x, y, z;                  ///< Position in world coordinates (metres)
  double battery_level;            ///< Battery level (0.0 to 1.0)
  std::string mission_state;       ///< Current mission state as string
  rclcpp::Time timestamp;          ///< Time of last update
  bool valid;                      ///< Whether the information is valid
};

/**
 * @brief Cooldown tracking for incident dispatch
 * 
 * Prevents duplicate responses to the same incident by multiple drones
 */
struct DispatchCooldown {
  rclcpp::Time until;                 ///< Cooldown expires at this time
  geometry_msgs::msg::Point target;   ///< Location of dispatched incident
  int responder_id{-1};              ///< ID of drone assigned to respond
};

/**
 * @brief Point on an orbital trajectory around an incident
 * 
 * Used for orbital inspection missions around detected incidents
 */
struct OrbitPoint
{
  float x;      ///< X position in world coordinates (metres)
  float y;      ///< Y position in world coordinates (metres)
  float z;      ///< Z position (altitude) in metres
  float yaw;    ///< Heading in radians, facing toward centre
};

/**
 * @brief Simple data structure to hold parsed scenario information
 * 
 * Contains all the information from a scenario detection message
 * in an easy-to-use format.
 */
struct ScenarioData {
  std::string scenario_name;  ///< Scenario type (e.g., "STRANDED_HIKER", "WILDFIRE", "DEBRIS_OBSTRUCTION")
  int severity;               ///< Severity level (1-10)
  double x;                   ///< X position in world coordinates (metres)
  double y;                   ///< Y position in world coordinates (metres)
  double z;                   ///< Z position (altitude) in metres
  double yaw;                 ///< Heading in radians
  bool can_respond;           ///< Whether drone should respond to this scenario
  bool valid;                 ///< Whether parsing was successful
};

/**
 * @brief Cached information about a peer drone
 * 
 * Continuously updated from peer broadcasts for non-blocking coordination
 */
struct PeerInfo {
  double battery{0.0};                          ///< Battery level (0.0 to 1.0)
  MissionState state{MissionState::IDLE};      ///< Current mission state
  geometry_msgs::msg::PoseStamped pose{};      ///< Current position and orientation
  rclcpp::Time stamp{};                        ///< Time of last update
};

/**
 * @brief Decoded scenario event from perception system
 * 
 * Parsed and typed representation of a scenario detection message
 */
struct ScenarioEvent {
  Scenario type{Scenario::UNKNOWN};            ///< Type of scenario detected
  MissionState state;                          ///< Recommended mission state for response
  geometry_msgs::msg::Point target{};          ///< Location of scenario in world coordinates
  double heading{0.0};                         ///< Heading in radians (from perception/IMU)
  bool can_respond{false};                     ///< Whether response is recommended
  rclcpp::Time stamp{};                        ///< Time when event was received
  std::string raw;                             ///< Original raw CSV string for audit logs
};

/**
 * @brief Active incident being managed by the fleet
 * 
 * Fleet-wide registry entry for incident tracking and coordination
 */
struct ActiveIncident {
  std::string scenario_name;                   ///< Type of scenario (e.g., "WILDFIRE")
  geometry_msgs::msg::Point location;          ///< Location in world coordinates (metres)
  int responder_id;                            ///< ID of drone assigned to respond
  rclcpp::Time dispatch_time;                  ///< Time when incident was dispatched
  rclcpp::Time expires_at;                     ///< Time when incident expires (5 minutes)
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
   * @param name Optional node name override (defaults to mission_planner_<pid>)
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
   * 
   * Executes mission logic, publishes mission state, and sends control commands
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
   * @brief Load waypoints from ROS parameters
   * 
   * Attempts to enumerate waypoint parameters in the format:
   * waypoints.<index>.position.{x,y,z}
   * Falls back to loadFallbackWaypoints() if no parameters found
   */
  void loadWaypointsFromParams();
  
  /**
   * @brief Load waypoints from inside source code (fallback)
   * 
   * Provides hardcoded default waypoints specific to each drone ID
   * when parameter-based waypoint loading fails
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
   * @param radius     circle radius (metres), negative treated as absolute value
   * @param pointCount number of points to produce (minimum 1)
   * @return           vector of OrbitPoint {X,Y,Z,Yaw}
   */
  std::vector<OrbitPoint> orbitTrajectory(float x, float y, float z,
                                          float radius, int pointCount);

  /**
   * @brief Normalise an angle to the range [-pi, pi]
   * @param a Angle in radians
   * @return Normalised angle in radians
   */
  inline float normalizeAngle(float a);

  /**
   * @brief Process scenario detection messages from perception system
   * @param msg String message containing scenario type, location, and severity
   * 
   * Parses scenario detection, performs coordination, and dispatches appropriate
   * drone response. Implements tie-breaker logic to prevent duplicate responses
   */
  void scenarioDetectionCallback(const std_msgs::msg::String::SharedPtr msg);

  // Mission state execution methods
  /**
   * @brief Execute takeoff sequence using sonar feedback
   * 
   * Uses sonar sensor to monitor altitude and climb to target_takeoff_altitude_ (4m).
   * Publishes climb commands and transitions to WAYPOINT_NAVIGATION or HOVERING when complete.
   * Must be initiated via takeoffDroneCallback service call.
   */
  void takeoff(void);
  
  /**
   * @brief Execute waypoint navigation logic
   * 
   * Checks waypoint arrival, advances to next waypoint, and handles mission completion.
   * For scenario reaction missions, restores previous mission state upon completion.
   * Generates new random patrol waypoints when autonomous patrol completes
   */
  void waypointNavigation(void);
  
  /**
   * @brief Execute hovering behaviour
   * 
   * Maintains current position. Actual position hold is handled by drone_controller
   */
  void hovering(void);
  
  /**
   * @brief Execute landing sequence using sonar feedback
   * 
   * Uses sonar sensor to monitor altitude and descend to target_landing_altitude_ (0.2m).
   * Publishes descent commands and transitions to IDLE when complete.
   * Must be initiated via landDroneCallback service call.
   */
  void landing(void);
  
  /**
   * @brief Execute manual control mode
   * 
   * Allows external velocity commands. Mission planner has no action in this state
   */
  void manualControl(void);
  
  /**
   * @brief Execute emergency descent procedure
   * 
   * Transitions to LANDING state for emergency landing. Actual descent is handled
   * by drone_controller to ensure safety
   */
  void emergency(void);
  
  /**
   * @brief Execute wildfire reaction mission (deprecated)
   * 
   * Replaced by service-based wildfireReactionCallback. Kept for backwards compatibility
   */
  void wildFireReaction(void);
  
  /**
   * @brief Execute debris reaction mission (deprecated)
   * 
   * Replaced by service-based debrisReactionCallback. Kept for backwards compatibility
   */
  void debrisReaction(void);
  
  /**
   * @brief Execute stranded hiker reaction mission (deprecated)
   * 
   * Replaced by service-based hikerRescueCallback. Kept for backwards compatibility
   */
  void strandedHikerReaction(void);
  
  /**
   * @brief Execute orbital inspection around an incident
   * 
   * Generates circular waypoints around detected incident for visual inspection
   */
  void orbitIncident(void);
  
  /**
   * @brief Send incident alert to GUI
   * @param ev Optional scenario event to alert about
   * 
   * Formats and publishes incident information in CSV format for GUI display
   */
  void alertIncidentGui(const std::optional<ScenarioEvent>& ev);
  
  /**
   * @brief Send resolved incident notification to GUI
   * @param location Location of the resolved incident
   * @param scenario_name Name of the scenario (e.g., "WILDFIRE", "DEBRIS_OBSTRUCTION")
   * @param resolved_at Time when incident was resolved
   * 
   * Formats and publishes resolved incident information in CSV format for GUI display
   */
  void alertResolvedIncidentGui(const geometry_msgs::msg::Point& location, 
                                 const std::string& scenario_name,
                                 const rclcpp::Time& resolved_at);
  
  /**
   * @brief Convert scenario enum to string representation
   * @param s Scenario type enum
   * @return String name of scenario (e.g., "WILDFIRE")
   */
  const char* evTypeToString(Scenario s) const;

  /**
   * @brief Discover peer drones in the fleet
   * 
   * Scans ROS graph for /rs1_drone_<id>/odom topics to identify active drones.
   * Creates subscriptions and publishers for newly discovered peers
   */
  void discoverPeerDrones(void);
  
  /**
   * @brief Create communication endpoints for a specific peer drone
   * @param id Numeric ID of peer drone
   * 
   * Sets up subscriptions and publishers for odometry, info manifests,
   * and mission assignments for the specified peer
   */
  void createPeerSubscriptionForId(int id);
  
  /**
   * @brief Handle information request pings from other drones
   * @param msg Empty message triggering info response
   * 
   * Responds to ping by publishing current drone status as CSV manifest
   */
  void infoRequestPingCallback(const std_msgs::msg::Empty::SharedPtr msg);
  
  /**
   * @brief Handle mission reset commands
   * @param msg String message specifying reset type (ORIGINAL_MISSION or ROUTE_MISSION)
   * 
   * Allows user to restore drone to original waypoint mission or last assigned route
   * after completing a scenario reaction
   */
  void resetMissioncallback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Build CSV info manifest for this drone
   * @return CSV string with drone ID, battery, state, position, and timestamp
   * 
   * Format: "id:<n>,battery:<0-1>,state:<STATE>,x:..,y:..,z:..,t:<ns>"
   */
  std::string buildInfoManifestCsv(void);
  
  /**
   * @brief Find peer drone closest to world origin
   * @return ID of closest peer drone, or -1 if no peers available
   */
  int findClosestPeerToOrigin(void) const;
  
  /**
   * @brief Process mission assignment from coordination drone
   * @param msg String message with mission type and waypoint coordinates
   * 
   * Parses ROUTE assignments in CSV format: ASSIGN,ROUTE,x1,y1,z1,x2,y2,z2,...
   * Sets waypoints and transitions to navigation state
   */
  void assignmentCallback(const std::shared_ptr<std_msgs::msg::String> msg);

  /**
   * @brief Parse key-value pair from CSV token
   * @param tok Token string in format "key:value"
   * @param key Output parameter for extracted key
   * @param val Output parameter for extracted value
   * @return True if parsing successful
   */
  static bool parseKeyVal(const std::string& tok, std::string& key, std::string& val);
  
  /**
   * @brief Convert string to MissionState enum
   * @param s String representation of mission state
   * @return Corresponding MissionState enum value
   */
  static MissionState stateFromString(const std::string& s);

  /**
   * @brief Parse drone info manifest from CSV string
   * @param manifest_data CSV string containing drone status information
   * @return DroneInfo struct with parsed data
   */
  DroneInfo parseInfoManifest(const std::string& manifest_data);
  
  /**
   * @brief Ping multiple drones for current status information
   * @param drone_ids Vector of drone IDs to query
   * @return Map of drone ID to DroneInfo structs
   * 
   * Non-blocking in composed mode. Uses cached peer info from continuous broadcasts
   * instead of waiting for ping responses to avoid blocking the executor
   */
  std::map<int, DroneInfo> pingDronesForInfo(const std::vector<int>& drone_ids);
  
  /**
   * @brief Select drone with lowest ID from available drones
   * @return ID of drone with lowest numeric ID
   */
  int selectLowestDronId();
  
  /**
   * @brief Coordinate fleet response to detected scenario
   * @param scenario Detected scenario data with type and location
   * 
   * Selects best available responder drone, generates incident ID,
   * records dispatch in fleet registry, and calls appropriate service
   */
  void performCoordination(const ScenarioData& scenario);

  /**
   * @brief Call wildfire reaction service on selected responder drone
   * @param responder_id ID of drone to respond
   * @param scenario Scenario data with fire location
   * @param incident_id Unique incident identifier for tracking
   * 
   * Sends async service request with fire location and depot coordinates
   */
  void callWildfireService(int responder_id, const ScenarioData& scenario, const std::string& incident_id);
  
  /**
   * @brief Call hiker rescue service on selected responder drone
   * @param responder_id ID of drone to respond
   * @param scenario Scenario data with hiker location
   * @param incident_id Unique incident identifier for tracking
   * 
   * Sends async service request with hiker location and medkit depot coordinates
   */
  void callHikerService(int responder_id, const ScenarioData& scenario, const std::string& incident_id);
  
  /**
   * @brief Call debris notification service on selected responder drone
   * @param responder_id ID of drone to respond
   * @param scenario Scenario data with debris location
   * @param incident_id Unique incident identifier for tracking
   * 
   * Sends async service request for orbital inspection of debris
   */
  void callDebrisService(int responder_id, const ScenarioData& scenario, const std::string& incident_id);
  
  /**
   * @brief Get or create wildfire service client for peer drone
   * @param drone_id ID of target drone
   * @return Shared pointer to wildfire service client
   * 
   * Returns cached client if exists, otherwise creates new client dynamically
   */
  rclcpp::Client<rs1_robot::srv::ReactToWildfire>::SharedPtr getOrCreateWildfireClient(int drone_id);
  
  /**
   * @brief Get or create hiker rescue service client for peer drone
   * @param drone_id ID of target drone
   * @return Shared pointer to hiker rescue service client
   * 
   * Returns cached client if exists, otherwise creates new client dynamically
   */
  rclcpp::Client<rs1_robot::srv::ReactToHiker>::SharedPtr getOrCreateHikerClient(int drone_id);
  
  /**
   * @brief Get or create debris notification service client for peer drone
   * @param drone_id ID of target drone
   * @return Shared pointer to debris notification service client
   * 
   * Returns cached client if exists, otherwise creates new client dynamically
   */
  rclcpp::Client<rs1_robot::srv::ReactToDebris>::SharedPtr getOrCreateDebrisClient(int drone_id);

  /**
   * @brief Parse scenario detection message from perception system
   * @param msg String message in CSV format
   * @return Optional ScenarioEvent if parsing successful
   * 
   * Expected format: "SCENARIO_NAME,x,y,z,heading,respond:1"
   */
  std::optional<ScenarioEvent> parseScenarioDetection(const std_msgs::msg::String& msg);

  /**
   * @brief Convert string to Scenario enum
   * @param s String scenario name (e.g., "WILDFIRE")
   * @return Corresponding Scenario enum value
   */
  static Scenario scenarioFromString(const std::string& s);

  /**
   * @brief Determine required mission state for scenario type
   * @param s Scenario type
   * @return Target mission state for drones to accept the scenario service
   * 
   * Returns WAYPOINT_NAVIGATION for actionable scenarios (wildfire, hiker, debris)
   * indicating drones must be in IDLE or WAYPOINT_NAVIGATION to respond
   */
  MissionState targetStateForScenario(Scenario s);
  
  //--- ROS 2 communication interfaces ---///
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;              ///< Odometry subscription
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;            ///< Sonar subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;  ///< Waypoint command subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scenario_sub_;            ///< Scenario detection subscription from perception
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr info_request_sub_;         ///< Info request subscription for coordination
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr assignment_subs_;         ///< Mission assignment subscription
  std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> info_manifest_subs_;  ///< Per-peer info manifest subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_mission_sub_;       ///< Mission reset subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr incident_dispatch_sub_;   ///< Fleet-wide incident dispatch subscription
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr landing_complete_sub_;     ///< Landing completion subscription from drone controller
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;            ///< Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;  ///< Target pose publisher for drone controller
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;          ///< Mission state publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr incident_pub_;               ///< Incident publisher for GUI
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr resolved_incident_pub_;      ///< Resolved incident publisher for GUI
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_manifest_pub_;          ///< Info manifest publisher for status broadcasts
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr info_request_pub_;            ///< Info request publisher (unused)
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> info_request_pubs_;  ///< Per-peer info request publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr incident_dispatch_pub_;      ///< Fleet-wide incident dispatch publisher
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;                 ///< Takeoff command publisher
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;                    ///< Landing command publisher
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;       ///< Start mission service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_mission_service_;        ///< Stop mission service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_drone_service_;       ///< Takeoff drone service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_drone_service_;          ///< Landing drone service
  
  // Scenario reaction services
  rclcpp::Service<rs1_robot::srv::ReactToWildfire>::SharedPtr wildfire_service_;   ///< Wildfire reaction service
  rclcpp::Service<rs1_robot::srv::ReactToHiker>::SharedPtr hiker_service_;         ///< Hiker rescue service
  rclcpp::Service<rs1_robot::srv::ReactToDebris>::SharedPtr debris_service_;       ///< Debris notification service
  
  // Service clients
  rclcpp::Client<rs1_robot::srv::ReactToWildfire>::SharedPtr wildfire_client_;     ///< Wildfire reaction client (self and peers)
  rclcpp::Client<rs1_robot::srv::ReactToHiker>::SharedPtr hiker_client_;           ///< Hiker rescue client (self and peers)
  rclcpp::Client<rs1_robot::srv::ReactToDebris>::SharedPtr debris_client_;         ///< Debris notification client (self and peers)
  
  // Per-peer service clients (dynamically created)
  std::map<int, rclcpp::Client<rs1_robot::srv::ReactToWildfire>::SharedPtr> peer_wildfire_clients_;  ///< Peer wildfire clients
  std::map<int, rclcpp::Client<rs1_robot::srv::ReactToHiker>::SharedPtr> peer_hiker_clients_;        ///< Peer hiker rescue clients
  std::map<int, rclcpp::Client<rs1_robot::srv::ReactToDebris>::SharedPtr> peer_debris_clients_;      ///< Peer debris clients
  
  // Timers
  rclcpp::TimerBase::SharedPtr mission_timer_;                                     ///< Periodic mission execution timer
  rclcpp::TimerBase::SharedPtr waypoint_load_timer_;                              ///< One-shot waypoint loading timer
  rclcpp::TimerBase::SharedPtr discovery_timer_;                                  ///< Periodic peer discovery timer
  rclcpp::TimerBase::SharedPtr mission_params_timer_;                             ///< One-shot mission params loading timer
  rclcpp::TimerBase::SharedPtr status_broadcast_timer_;                           ///< Periodic status broadcast timer
  rclcpp::TimerBase::SharedPtr tiebreaker_timer_;                                 ///< One-shot tie-breaker delay timer

  // Mission management components
  std::unique_ptr<StateMachine> state_machine_;       ///< Mission state machine for transitions
  std::unique_ptr<WaypointPlanner> path_planner_;     ///< Waypoint path planner and trajectory generator
  std::unique_ptr<MissionExecutor> mission_executor_; ///< Advanced mission executor for scenario reactions

  // Current state variables
  geometry_msgs::msg::PoseStamped current_pose_;  ///< Current drone pose from odometry
  
  // Takeoff control variables
  double current_sonar_range_;                     ///< Current sonar reading in metres
  sensor_msgs::msg::LaserScan current_lidar_data_; ///< Current LiDAR scan for obstacle detection
  std::mutex lidar_mutex_;                         ///< Mutex for LiDAR data thread safety
  bool takeoff_in_progress_;                       ///< Flag indicating takeoff sequence active
  std::chrono::steady_clock::time_point takeoff_start_time_; ///< Time when takeoff initiated
  double target_takeoff_altitude_;                 ///< Target altitude for takeoff (4.0m)
  bool takeoff_complete_;                          ///< Flag indicating takeoff completion
  
  // Landing control variables
  bool landing_in_progress_;                       ///< Flag indicating landing sequence active
  std::chrono::steady_clock::time_point landing_start_time_; ///< Time when landing initiated
  double target_landing_altitude_;                 ///< Target altitude for landing (0.2m)
  bool landing_complete_;                          ///< Flag indicating landing completion
  bool landing_completion_detected_;               ///< Flag to prevent repeated completion checks
  std::chrono::steady_clock::time_point landing_complete_time_; ///< Time when landing completion detected
  
  mutable std::mutex sonar_mutex_;                 ///< Mutex for sonar data thread safety

  // Fleet coordination variables
  mutable std::mutex peers_mutex_;                                                                          ///< Mutex for peer data structures
  std::unordered_map<int, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_odom_subs_;      ///< Per-peer odometry subscriptions
  std::unordered_map<int, geometry_msgs::msg::PoseStamped> peer_poses_;                                   ///< Cached peer positions
  std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> assignment_pubs_;          ///< Per-peer assignment publishers
  std::unordered_map<int, PeerInfo> peer_info_;                                                           ///< Cached peer status info
  mutable std::mutex incident_mutex_;                                                                      ///< Mutex for incident data
  std::optional<ScenarioEvent> active_incident_event_;                                                     ///< Currently active scenario event
  std::vector<geometry_msgs::msg::PoseStamped> route_waypoints_cached_;                                   ///< Cached route mission waypoints
  std::vector<geometry_msgs::msg::PoseStamped> original_waypoints_cached_;                                ///< Cached original mission waypoints
  std::mutex cache_mutex_;                                                                                 ///< Mutex for waypoint cache thread safety

  // Configuration parameters
  std::string drone_namespace_;   ///< ROS namespace for this drone (e.g., "rs1_drone_1")
  std::string drone_id_;          ///< Unique drone identifier string
  double mission_update_rate_;    ///< Mission timer frequency in Hz
  double waypoint_tolerance_;     ///< Waypoint arrival tolerance in metres
  int incident_counter_;          ///< Counter for generating unique incident IDs
  int drone_numeric_id_;          ///< Numeric drone identifier extracted from namespace

  // Scenario reaction state
  ReactionPhase wildfire_phase_{ReactionPhase::NONE};      ///< Current wildfire reaction phase
  rclcpp::Time   wildfire_phase_start_;                    ///< Time when current phase started
  std::optional<int> assigned_peer_id_;                    ///< ID of peer assigned to scenario

  double battery_level_{0.8};                              ///< Current battery level (0.0 to 1.0)
  int    collect_window_ms_{400};                          ///< Info collection window in milliseconds
  geometry_msgs::msg::Point depot_xyz_{};                  ///< Fire retardant depot location
  
  // Coordination state flags
  bool is_coordinating_ = false;                           ///< Flag indicating active coordination
  std::optional<ScenarioData> active_coordination_scenario_;  ///< Currently coordinating scenario
  std::mutex coordination_mutex_;                          ///< Mutex for coordination state thread safety
  
  // Scenario reaction mission state
  bool in_scenario_reaction_ = false;                      ///< Flag indicating active scenario mission
  std::string active_scenario_type_;                       ///< Type of active scenario (e.g., "WILDFIRE")
  std::string active_scenario_incident_id_;                ///< Incident ID for active scenario
  bool was_idle_before_reaction_ = false;                  ///< Whether drone was IDLE before scenario
  bool payload_collected_ = false;                         ///< Whether retardant/medkit collected from depot

  geometry_msgs::msg::Pose hover_hold_pose_;               ///< Pose to hold during hovering

  bool repeat_Waypoint_Path_ = false;                      ///< Whether to repeat waypoint path or stop

  // Incident dispatch cooldown tracking
  mutable std::mutex dispatch_mutex_;                                             ///< Mutex for dispatch cooldown map
  std::unordered_map<std::string, DispatchCooldown> dispatch_cooldown_;          ///< Per-scenario dispatch cooldowns

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};            ///< Steady clock for cooldown timing

  rclcpp::Duration coordination_cooldown_{std::chrono::seconds(5)};  ///< Cooldown duration between dispatches (5 seconds)
  double incident_merge_radius_m_{7.0};                    ///< Radius for merging duplicate incidents (7 metres)

  // Fleet-wide incident registry
  std::map<std::string, ActiveIncident> fleet_incident_registry_;  ///< Fleet-wide active incident registry
  std::mutex registry_mutex_;                                      ///< Mutex for registry thread safety

  // Recently resolved incidents tracking
  /**
   * @brief Recently resolved incident for preventing re-detection
   * 
   * Stores resolved incident location and time to prevent drones from
   * re-detecting and responding to the same incident after resuming patrol
   */
  struct ResolvedIncident {
    geometry_msgs::msg::Point location;    ///< Location of resolved incident
    std::string scenario_name;             ///< Type of scenario that was resolved
    rclcpp::Time resolved_at;              ///< Time when incident was resolved
  };
  std::vector<ResolvedIncident> recently_resolved_incidents_;         ///< List of recently resolved incidents
  std::mutex resolved_incidents_mutex_;                               ///< Mutex for resolved incidents list
  rclcpp::Duration resolved_incident_ignore_duration_{std::chrono::seconds(1200)};  ///< Ignore duration (20 minutes)
  double resolved_incident_match_radius_{15.0};                       ///< Matching radius for resolved incidents (15 metres)

  geometry_msgs::msg::Point helipad_location_;             ///< Helipad location for landing and depot
  
  // Mission assignment state flags
  bool in_hiker_rescue_{false};                            ///< Flag indicating active hiker rescue mission
  bool medkit_collected_{false};                           ///< Whether medkit collected from depot
  bool in_hiker_rescue_awaiting_takeoff_{false};           ///< Prevents state machine loop during takeoff
  rclcpp::Time medkit_collect_stamp_;                      ///< Time when medkit was collected
  geometry_msgs::msg::Point hiker_target_xyz_{};           ///< Hiker location for rescue mission
  geometry_msgs::msg::Point medkit_depot_xyz_{};           ///< Medkit depot location (loaded from params)

  /**
   * @brief Select best available responder drone for scenario
   * @param all_drone_ids Vector of all known drone IDs (including self)
   * @param required_state Required mission state for eligibility
   * @param incident_xyz Location of incident in world coordinates
   * @return ID of best responder drone, or -1 if none available
   * 
   * Selects closest IDLE drone with sufficient battery (>50%).
   * Returns -1 if no suitable responder found
   */
  int selectBestResponderDrone(const std::vector<int>& all_drone_ids,
                              MissionState required_state,
                              const geometry_msgs::msg::Point& incident_xyz);

  /**
   * @brief Process info manifest broadcast from peer drone
   * @param peer_id ID of peer drone
   * @param msg CSV string with peer status information
   * 
   * Updates cached peer_info_ for non-blocking coordination
   */
  void infoManifestCallback(int peer_id, const std_msgs::msg::String::SharedPtr& msg);
  
  /**
   * @brief Wait for peer DDS match (deprecated, unused)
   * @param id Peer drone ID
   * @param max_wait Maximum time to wait
   * @return True if matched within timeout
   */
  bool waitForPeerMatch(int id, std::chrono::milliseconds max_wait);
  
  /**
   * @brief Wait for peer info_request subscriber to be ready
   * @param peer_id ID of peer drone
   * @param timeout Maximum time to wait for subscriber match
   * @return True if subscriber matched within timeout
   * 
   * Used before sending ping to ensure DDS connection established
   */
  bool waitForPeerPingSubscriber(int peer_id, std::chrono::milliseconds timeout);
  
  /**
   * @brief Check if incident should be suppressed due to cooldown
   * @param s Scenario data to check
   * @return True if incident should be suppressed
   * 
   * Checks dispatch cooldown and responder busy state to prevent duplicate responses
   */
  bool shouldSuppressIncident(const ScenarioData& s);
  
  /**
   * @brief Check if drone is busy with assigned mission
   * @return True if drone is executing hiker rescue mission
   */
  inline bool isBusyWithAssignedMission() const {
    return in_hiker_rescue_;
  }

  /**
   * @brief Check if incident is already being managed by fleet
   * @param scenario Scenario data to check
   * @return True if incident already has assigned responder
   * 
   * Searches fleet registry for matching incidents within merge radius.
   * Cleans up expired registry entries during check
   */
  bool isIncidentAlreadyManaged(const ScenarioData& scenario);
  
  /**
   * @brief Process fleet-wide incident dispatch broadcast
   * @param msg CSV string with dispatch information
   * 
   * Updates local fleet registry when other drones dispatch incidents.
   * Format: "DISPATCH,<id>,<type>,<x>,<y>,<z>,<responder_id>,<timestamp>"
   */
  void incidentDispatchCallback(const std::shared_ptr<std_msgs::msg::String> msg);

  /**
   * @brief Handle landing completion notification from drone controller
   * @param msg Empty message indicating landing is complete
   * 
   * Immediately transitions state to IDLE when landing completion is detected.
   */
  void landingCompleteCallback(const std::shared_ptr<std_msgs::msg::Empty> msg);
  
  /**
   * @brief Generate deterministic incident ID from scenario data
   * @param scenario Scenario data with type and location
   * @return Unique incident ID string (e.g., "WILDFIRE_105_52_21")
   * 
   * Rounds coordinates to 0.1m precision to ensure all drones generate
   * same ID for the same incident
   */
  std::string generateIncidentId(const ScenarioData& scenario);
  
  /**
   * @brief Record incident dispatch in fleet registry and broadcast
   * @param incident_id Unique incident identifier
   * @param scenario Scenario data with type and location
   * @param responder_id ID of drone assigned to respond
   * 
   * Updates fleet registry, broadcasts dispatch message, and sets local cooldown
   */
  void recordIncidentDispatch(const std::string& incident_id, const ScenarioData& scenario, int responder_id);
    
  /**
   * @brief Get list of all known drone IDs in fleet
   * @return Vector of drone IDs (including self)
   * 
   * Combines discovered peer IDs with self ID
   */
  std::vector<int> getKnownDroneIds(void);
};

}  // namespace drone_swarm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_swarm::MissionPlannerNode)

#endif  // DRONE_SWARM_MISSION_NODE_H_