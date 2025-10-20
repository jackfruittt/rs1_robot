#include "mission_node.h"
#include <thread>

namespace drone_swarm
{

// --- FORWARD DECLARATIONS for helper functions ---
static inline std::string trimCopy(std::string s);
static inline std::vector<std::string> splitCSV(const std::string& s);
static inline bool parseDouble(const std::string& s, double& out);
ScenarioData parseScenarioMessage(const std::string& message_data);
static std::string missionStateToString(MissionState state);  // ADD THIS LINE

// --- END FORWARD DECLARATIONS ---

  MissionPlannerNode::MissionPlannerNode(const rclcpp::NodeOptions& options)
    : Node("mission_planner", options)
  {
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // --- Parameter Loading (unchanged, all correct) ---
    this->declare_parameter("drone_namespace", std::string("rs1_drone"));
    this->declare_parameter<double>("mission_update_rate", 5.0);
    this->declare_parameter<double>("waypoint_tolerance", 0.5);
    this->declare_parameter<double>("helipad_location.x", 0.0);
    this->declare_parameter<double>("helipad_location.y", 0.0);
    this->declare_parameter<double>("helipad_location.z", 0.0);
    this->declare_parameter<double>("battery_level", 0.8);
    this->declare_parameter<double>("retardant_depot.x", -28.3);
    this->declare_parameter<double>("retardant_depot.y",  14.0);
    this->declare_parameter<double>("retardant_depot.z",  14.0);
    this->declare_parameter<double>("medkit_depot.x", -20.0);
    this->declare_parameter<double>("medkit_depot.y", -15.0);
    this->declare_parameter<double>("medkit_depot.z",  14.0);

    drone_namespace_ = this->get_parameter("drone_namespace").as_string();
    mission_update_rate_ = this->get_parameter("mission_update_rate").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
    helipad_location_.x = this->get_parameter("helipad_location.x").as_double();
    helipad_location_.y = this->get_parameter("helipad_location.y").as_double();
    helipad_location_.z = this->get_parameter("helipad_location.z").as_double();
    battery_level_ = this->get_parameter("battery_level").as_double();
    depot_xyz_.x = this->get_parameter("retardant_depot.x").as_double();
    depot_xyz_.y = this->get_parameter("retardant_depot.y").as_double();
    depot_xyz_.z = this->get_parameter("retardant_depot.z").as_double();
    medkit_depot_xyz_.x = this->get_parameter("medkit_depot.x").as_double();
    medkit_depot_xyz_.y = this->get_parameter("medkit_depot.y").as_double();
    medkit_depot_xyz_.z = this->get_parameter("medkit_depot.z").as_double();

    // --- Component Initialization ---
    state_machine_ = std::make_unique<StateMachine>();
    path_planner_ = std::make_unique<PathPlanner>();
    
    drone_id_ = drone_namespace_;
    try {
        std::string num_part = drone_id_.substr(drone_id_.find_last_of('_') + 1);
        drone_numeric_id_ = std::stoi(num_part);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "FATAL: Could not parse numeric ID from namespace: %s", drone_id_.c_str());
    }

    loadWaypointsFromParams();
    
    // --- Subscriptions & Publishers (with corrected absolute paths) ---
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + drone_namespace_ + "/odom", 10, std::bind(&MissionPlannerNode::odomCallback, this, std::placeholders::_1));
    scenario_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/" + drone_namespace_ + "/scenario_detection", reliable_qos, std::bind(&MissionPlannerNode::scenarioDetectionCallback, this, std::placeholders::_1));
    info_request_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/" + drone_namespace_ + "/info_request", reliable_qos, std::bind(&MissionPlannerNode::infoRequestPingCallback, this, std::placeholders::_1));
    assignment_subs_ = this->create_subscription<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/mission_assignment", reliable_qos, 
      std::bind(&MissionPlannerNode::assignmentCallback, this, std::placeholders::_1));

      
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + drone_namespace_ + "/cmd_vel", 10);
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + drone_namespace_ + "/target_pose", 10);
    mission_state_pub_ = this->create_publisher<std_msgs::msg::String>("/" + drone_namespace_ + "/mission_state", reliable_qos);
    info_manifest_pub_ = this->create_publisher<std_msgs::msg::String>("/" + drone_namespace_ + "/info_manifest", reliable_qos);
      
    // --- Services (with corrected absolute paths) ---
    start_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/start_mission", std::bind(&MissionPlannerNode::startMissionCallback, this, std::placeholders::_1, std::placeholders::_2));
    stop_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/stop_mission", std::bind(&MissionPlannerNode::stopMissionCallback, this, std::placeholders::_1, std::placeholders::_2));

    // --- Timers ---
    auto mission_timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / mission_update_rate_));
    mission_timer_ = this->create_wall_timer(mission_timer_period, std::bind(&MissionPlannerNode::missionTimerCallback, this));
    
    discovery_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&MissionPlannerNode::discoverPeerDrones, this));

    RCLCPP_INFO(this->get_logger(), "Mission Planner Node initialised for %s", drone_id_.c_str());

    discoverPeerDrones();
  }

  std::map<int, DroneInfo> MissionPlannerNode::pingDronesForInfo(
      const std::vector<int>& drone_ids, int timeout_ms) {
    
    std::map<int, DroneInfo> results;
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Separate self from peers
    std::vector<int> peers_to_ping;
    for (int id : drone_ids) {
      if (id == drone_numeric_id_) {
        DroneInfo self_info = parseInfoManifest(buildInfoManifestCsv());
        results[id] = self_info;
      } else {
        peers_to_ping.push_back(id);
        results[id] = {id, 0,0,0,0.0,"",this->now(),false}; // pre-fill with invalid
      }
    }

    if (peers_to_ping.empty()) {
      RCLCPP_INFO(this->get_logger(), "Ping complete: 1/1 responses (self)");
      return results;
    }
    
    // Ensure peer wiring exists before pinging (idempotent & cheap)
    for (int id : peers_to_ping) {
      // Make sure we have the critical subscription on /rs1_drone_<id>/info_manifest
      createPeerSubscriptionForId(id);
    }

    RCLCPP_INFO(this->get_logger(), "Pinging %zu peers...", peers_to_ping.size());
    
    // Record when we sent pings - only accept responses after this
    auto ping_start_time = this->now();
  

    // Mark existing peer_info as stale so we only accept fresh responses
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      for (int id : peers_to_ping) {
        auto it = peer_info_.find(id);
        if (it != peer_info_.end()) {
          it->second.stamp = rclcpp::Time(0);  // Mark as stale
        }
      }
    }
    
    // Send pings using persistent publishers
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      for (int drone_id : peers_to_ping) {
        auto it = info_request_pubs_.find(drone_id);
        if (it == info_request_pubs_.end() || !it->second) {
          // Create JIT publisher if discovery hasn't run yet
          std::string topic = "/rs1_drone_" + std::to_string(drone_id) + "/info_request";
          info_request_pubs_[drone_id] = this->create_publisher<std_msgs::msg::Empty>(topic, reliable_qos);
          RCLCPP_WARN(this->get_logger(), "Created JIT publisher for drone %d", drone_id);
        }
        info_request_pubs_[drone_id]->publish(std_msgs::msg::Empty());
      }
    }
    
    // Wait for responses to arrive via persistent subscriptions
    auto end_time = ping_start_time + rclcpp::Duration(std::chrono::milliseconds(timeout_ms));
    
    while (this->now() < end_time) {
      // Let any pending subscription callbacks run (unblocks the executor)
      bool all_received = true;
      
      {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        for (int id : peers_to_ping) {
          auto it = peer_info_.find(id);
          
          // Check if we have a fresh response (received after ping was sent)
          if (it == peer_info_.end() || it->second.stamp <= ping_start_time) {
            all_received = false;
            continue;
          }
          
          // We have fresh data - convert PeerInfo to DroneInfo
          if (!results[id].valid) {  // Only convert once
            DroneInfo info;
            info.drone_id = id;
            info.battery_level = it->second.battery;
            info.mission_state = missionStateToString(it->second.state);
            info.x = it->second.pose.pose.position.x;
            info.y = it->second.pose.pose.position.y;
            info.z = it->second.pose.pose.position.z;
            info.timestamp = it->second.stamp;
            info.valid = true;
            results[id] = info;
          }
        }
      }
      
      if (all_received) {
        RCLCPP_INFO(this->get_logger(), "All responses received early!");
        break;
      }
      
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Log results
    int valid_count = 0;
    for (const auto& [id, info] : results) {
      if (info.valid) {
        valid_count++;
        RCLCPP_DEBUG(this->get_logger(), "Drone %d: Valid response (battery=%.0f%%, state=%s)", 
                    id, info.battery_level * 100.0, info.mission_state.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Drone %d: No response received", id);
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Ping complete: %d/%zu responses", 
                valid_count, drone_ids.size());
    
    return results;
  }


  void MissionPlannerNode::missionTimerCallback() {
    executeMission();

    // --- NEW LOGIC FOR POST-LANDING PAUSE ---
    if (in_hiker_rescue_ && medkit_collected_) {
        // Wait 2 seconds to simulate collection
        if (this->get_clock()->now() - medkit_collect_stamp_ > rclcpp::Duration::from_seconds(2.0)) {
            RCLCPP_INFO(this->get_logger(), "Medkit collected, proceeding to hiker location.");
            geometry_msgs::msg::PoseStamped wp_hiker;
            wp_hiker.header.frame_id = "map";
            wp_hiker.pose.position = hiker_target_xyz_;
            wp_hiker.pose.orientation.w = 1.0;
            path_planner_->setWaypoints({wp_hiker});
            state_machine_->setState(MissionState::TAKEOFF);
            medkit_collected_ = false; // Prevent this block from running again
            in_hiker_rescue_awaiting_takeoff_ = true; // Prevent waypointNavigation from re-landing
        }
    }

    if (in_fetch_rt_ && fetch_landed_) {
      // Wait 2 seconds to simulate collection
      if (this->get_clock()->now() - fetch_land_stamp_ > rclcpp::Duration::from_seconds(2.0)) {
        RCLCPP_INFO(this->get_logger(), "Retardant collected, proceeding to fire location.");
        geometry_msgs::msg::PoseStamped wp_fire;
        wp_fire.header.frame_id = "map";
        wp_fire.pose.position = fetch_fire_target_;
        wp_fire.pose.orientation.w = 1.0;
        path_planner_->setWaypoints({wp_fire});
        state_machine_->setState(MissionState::TAKEOFF);
        fetch_landed_ = false; // Prevent this block from running again
      }
    }
    // --- END NEW LOGIC ---

    std_msgs::msg::String state_msg;
    state_msg.data = state_machine_->getStateString();
    mission_state_pub_->publish(state_msg);
    publishMissionCommand();
  }

  void MissionPlannerNode::waypointNavigation() {
    if (!path_planner_->hasNextWaypoint()) { 
      state_machine_->setState(MissionState::HOVERING);
      return;
    }
    if (!isWaypointReached()) return;

    // --- NEW LOGIC ---
    if (in_hiker_rescue_ && !medkit_collected_ && !in_hiker_rescue_awaiting_takeoff_) {
      RCLCPP_INFO(this->get_logger(), "Arrived at medkit depot. Landing to collect.");
      state_machine_->setState(MissionState::LANDING);
      return; 
    }

    if (in_fetch_rt_ && !fetch_landed_) {
      RCLCPP_INFO(this->get_logger(), "Arrived at retardant depot. Landing to collect.");
      state_machine_->setState(MissionState::LANDING);
      return; 
    }
    // --- END NEW LOGIC ---

    (void)path_planner_->getNextWaypoint();
    if (!path_planner_->hasNextWaypoint()) {
      RCLCPP_INFO(get_logger(), "Final waypoint reached. Mission complete. Hovering.");
      state_machine_->setState(MissionState::HOVERING);
      // Reset flags
      in_fetch_rt_ = false;
      in_hiker_rescue_ = false;
      in_hiker_rescue_awaiting_takeoff_ = false;
    } else {
      RCLCPP_INFO(get_logger(), "Waypoint reached - moving to next waypoint");
    }
  }

  // 2. Modify landing() to know it's just a pause, not the end of the mission
  void MissionPlannerNode::landing() {
    static std::map<std::string, std::chrono::steady_clock::time_point> landing_timers;
    if (landing_timers.find(drone_id_) == landing_timers.end()) {
        landing_timers[drone_id_] = std::chrono::steady_clock::now();
    }

    if (std::chrono::steady_clock::now() - landing_timers[drone_id_] > std::chrono::seconds(8)) {
      if (in_hiker_rescue_ && !medkit_collected_) {
        medkit_collected_ = true;
        medkit_collect_stamp_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Landed at medkit depot. Pausing for collection.");
        // Stay in LANDING state. The missionTimer will handle the next step.
      } 
      else if (in_fetch_rt_ && !fetch_landed_) {
        fetch_landed_ = true;
        fetch_land_stamp_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Landed at retardant depot. Pausing for collection.");
        // Stay in LANDING state.
      } else {
        // This is a normal, mission-ending landing.
        state_machine_->setState(MissionState::IDLE);
        path_planner_->reset();
        // Reset flags just in case
        in_fetch_rt_ = false;
        in_hiker_rescue_ = false;
        in_hiker_rescue_awaiting_takeoff_ = false;
      }
      landing_timers.erase(drone_id_);
    }
  }

  // In mission_node.cpp, add this new function
  void MissionPlannerNode::assignmentCallback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string payload = trimCopy(msg->data);
    auto tokens = splitCSV(payload);
    if (tokens.size() < 2 || tokens[0] != "ASSIGN") return;

    if (tokens[1] == "HIKER_RESCUE") {
      if (tokens.size() < 8) return; // Need 8 tokens
      double dx, dy, dz, hx, hy, hz;
      if (!parseDouble(tokens[2], dx) || !parseDouble(tokens[3], dy) || !parseDouble(tokens[4], dz) ||
          !parseDouble(tokens[5], hx) || !parseDouble(tokens[6], hy) || !parseDouble(tokens[7], hz)) {
        return;
      }

      RCLCPP_INFO(this->get_logger(), "HIKER_RESCUE mission assigned.");
      in_hiker_rescue_ = true;
      medkit_collected_ = false;
      in_hiker_rescue_awaiting_takeoff_ = false;
      hiker_target_xyz_.x = hx;
      hiker_target_xyz_.y = hy;
      hiker_target_xyz_.z = hz;

      geometry_msgs::msg::PoseStamped wp_depot;
      wp_depot.header.frame_id = "map";
      wp_depot.pose.position.x = dx;
      wp_depot.pose.position.y = dy;
      wp_depot.pose.position.z = dz;
      wp_depot.pose.orientation.w = 1.0;
      
      path_planner_->setWaypoints({wp_depot});
      if (canStateTransitionTo(state_machine_->getCurrentState(), MissionState::WAYPOINT_NAVIGATION)) {
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      }
    }
    else if (tokens[1] == "FETCH_RT") {
      if (tokens.size() < 8) return;
      double dx, dy, dz, fx, fy, fz;
      if (!parseDouble(tokens[2], dx) || !parseDouble(tokens[3], dy) || !parseDouble(tokens[4], dz) ||
          !parseDouble(tokens[5], fx) || !parseDouble(tokens[6], fy) || !parseDouble(tokens[7], fz)) {
        return;
      }

      RCLCPP_INFO(this->get_logger(), "FETCH_RT mission assigned.");
      in_fetch_rt_ = true;
      fetch_landed_ = false;
      fetch_fire_target_.x = fx;
      fetch_fire_target_.y = fy;
      fetch_fire_target_.z = fz;

      geometry_msgs::msg::PoseStamped wp_depot;
      wp_depot.header.frame_id = "map";
      wp_depot.pose.position.x = dx;
      wp_depot.pose.position.y = dy;
      wp_depot.pose.position.z = dz;
      wp_depot.pose.orientation.w = 1.0;

      path_planner_->setWaypoints({wp_depot});
      if (canStateTransitionTo(state_machine_->getCurrentState(), MissionState::WAYPOINT_NAVIGATION)) {
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      }
    }
    else if (tokens[1] == "ROUTE") {
      // Expected format: ASSIGN,ROUTE,x1,y1,z1,x2,y2,z2,...
      // Number of tokens must be 2 (ASSIGN,ROUTE) + a multiple of 3 (x,y,z)
      if (tokens.size() < 5 || (tokens.size() - 2) % 3 != 0) {
        RCLCPP_WARN(this->get_logger(), "ROUTE command requires tokens in multiples of 3 for coordinates.");
        return;
      }

      std::vector<geometry_msgs::msg::PoseStamped> new_waypoints;
      RCLCPP_INFO(this->get_logger(), "ROUTE mission assigned with %zu waypoints.", (tokens.size() - 2) / 3);

      // Loop through the coordinate triples
      for (size_t i = 2; i < tokens.size(); i += 3) {
        double target_x, target_y, target_z;
        if (!parseDouble(tokens[i], target_x) || !parseDouble(tokens[i+1], target_y) || !parseDouble(tokens[i+2], target_z)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse coordinates for ROUTE command at index %zu.", i);
          return; // Abort if any coordinate is invalid
        }

        geometry_msgs::msg::PoseStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.pose.position.x = target_x;
        waypoint.pose.position.y = target_y;
        waypoint.pose.position.z = target_z;
        waypoint.pose.orientation.w = 1.0;
        
        new_waypoints.push_back(waypoint);
      }

      // Set the full list of waypoints in the planner
      path_planner_->setWaypoints(new_waypoints);
      
      // FORCE the drone to start navigating the new route
      state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);

    }
  }

  void MissionPlannerNode::sendMissionToLowestDrone(const ScenarioData& scenario) {
    RCLCPP_INFO(this->get_logger(), "Coordinating response for %s", scenario.scenario_name.c_str());
    
    MissionState required_state = targetStateForScenario(scenarioFromString(scenario.scenario_name));
    
    // Assuming a fixed number of drones for now
    std::vector<int> all_drones = {1, 2, 3, 4}; 
    int responder_id = selectBestResponderDrone(all_drones, required_state);
    
    if (responder_id < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find suitable responder! No drones available.");
      return;
    }
    
    if (drone_numeric_id_ == responder_id) {
      RCLCPP_INFO(this->get_logger(), "I am drone %d - I will respond to this scenario myself.", responder_id);
      
      // --- SELF-ASSIGNMENT LOGIC ---
      if (scenario.scenario_name == "STRANDED_HIKER") {
        in_hiker_rescue_ = true;
        medkit_collected_ = false;
        in_hiker_rescue_awaiting_takeoff_ = false;
        hiker_target_xyz_.x = scenario.x;
        hiker_target_xyz_.y = scenario.y;
        hiker_target_xyz_.z = scenario.z;      
        geometry_msgs::msg::PoseStamped wp_depot;
        wp_depot.header.frame_id = "map";
        wp_depot.pose.position = medkit_depot_xyz_;
        wp_depot.pose.orientation.w = 1.0;
        
        path_planner_->setWaypoints({wp_depot});
        if (canStateTransitionTo(state_machine_->getCurrentState(), MissionState::WAYPOINT_NAVIGATION)) {
          state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
        }
      }
      else if (scenario.scenario_name == "WILDFIRE") {
        in_fetch_rt_ = true;
        fetch_landed_ = false;
        fetch_fire_target_.x = scenario.x;
        fetch_fire_target_.y = scenario.y;
        fetch_fire_target_.z = scenario.z;
        geometry_msgs::msg::PoseStamped wp_depot;
        wp_depot.header.frame_id = "map";
        wp_depot.pose.position = depot_xyz_;
        wp_depot.pose.orientation.w = 1.0;

        path_planner_->setWaypoints({wp_depot});
        if (canStateTransitionTo(state_machine_->getCurrentState(), MissionState::WAYPOINT_NAVIGATION)) {
          state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
        }
      }
    }
    else {
      RCLCPP_INFO(this->get_logger(), "I am drone %d - sending mission to drone %d", drone_numeric_id_, responder_id);
      
      std::ostringstream ss;
      
      if (scenario.scenario_name == "STRANDED_HIKER") {
        // Format: ASSIGN,HIKER_RESCUE,depot_x,y,z,hiker_x,y,z
        ss << "ASSIGN,HIKER_RESCUE,"
          << medkit_depot_xyz_.x << "," << medkit_depot_xyz_.y << "," << medkit_depot_xyz_.z << ","
          << scenario.x << "," << scenario.y << "," << scenario.z;
      }
      else if (scenario.scenario_name == "WILDFIRE") {
        // Format: ASSIGN,FETCH_RT,depot_x,y,z,fire_x,y,z
        ss << "ASSIGN,FETCH_RT,"
          << depot_xyz_.x << "," << depot_xyz_.y << "," << depot_xyz_.z << ","
          << scenario.x << "," << scenario.y << "," << scenario.z;
      }
      
      std_msgs::msg::String msg;
      msg.data = ss.str();
      
      std::lock_guard<std::mutex> lock(peers_mutex_);
      auto it = assignment_pubs_.find(responder_id);
      if (it != assignment_pubs_.end()) {
        it->second->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent mission to drone %d: %s", responder_id, msg.data.c_str());
      }
    }
  }

  bool MissionPlannerNode::canStateTransitionTo(MissionState current_state, MissionState target_state) {
    switch (current_state) {
      case MissionState::IDLE:
        return  target_state == MissionState::TAKEOFF || 
                target_state == MissionState::MANUAL_CONTROL;
        // If you want your newer behavior, also allow:
        // || target_state == MissionState::WAYPOINT_NAVIGATION;

      case MissionState::TAKEOFF:
        return  target_state == MissionState::WAYPOINT_NAVIGATION || 
                target_state == MissionState::HOVERING ||
                target_state == MissionState::EMERGENCY;

      case MissionState::WAYPOINT_NAVIGATION:
        return  target_state == MissionState::HOVERING || 
                target_state == MissionState::LANDING ||
                target_state == MissionState::EMERGENCY || 
                target_state == MissionState::WILDFIRE_REACTION ||
                target_state == MissionState::ORBIT_INCIDENT || 
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION;

      case MissionState::HOVERING:
        return  target_state == MissionState::WAYPOINT_NAVIGATION || 
                target_state == MissionState::LANDING ||
                target_state == MissionState::EMERGENCY || 
                target_state == MissionState::WILDFIRE_REACTION ||
                target_state == MissionState::ORBIT_INCIDENT || 
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

      case MissionState::ORBIT_INCIDENT:
      case MissionState::STRANDED_HIKER_REACTION:
      case MissionState::DEBRIS_OBSTRUCTION_REACTION:
      case MissionState::WILDFIRE_REACTION:
        return  target_state == MissionState::IDLE ||
                target_state == MissionState::TAKEOFF ||
                target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::HOVERING ||
                target_state == MissionState::LANDING ||
                target_state == MissionState::MANUAL_CONTROL ||
                target_state == MissionState::WILDFIRE_REACTION ||
                target_state == MissionState::ORBIT_INCIDENT ||
                target_state == MissionState::STRANDED_HIKER_REACTION ||
                target_state == MissionState::DEBRIS_OBSTRUCTION_REACTION ||
                target_state == MissionState::EMERGENCY;
    }
    return false;
  }


  void MissionPlannerNode::loadWaypointsFromParams() {
    RCLCPP_INFO(this->get_logger(), "Loading waypoints from flattened parameters for %s", drone_id_.c_str());
    
    try {
        // Check if mission name parameter exists
        if (this->has_parameter("mission_name")) {
            std::string mission_name = this->get_parameter("mission_name").as_string();
            RCLCPP_INFO(this->get_logger(), "Mission: %s", mission_name.c_str());
        }
        
        // Load waypoints from flattened parameters
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        int i = 0;
        
        while (true) {
            std::string x_key = "waypoints." + std::to_string(i) + ".position.x";
            
            if (!this->has_parameter(x_key)) {
                break; // No more waypoints
            }
            
            std::string y_key = "waypoints." + std::to_string(i) + ".position.y";
            std::string z_key = "waypoints." + std::to_string(i) + ".position.z";
            std::string dwell_time_key = "waypoints." + std::to_string(i) + ".dwell_time";
            
            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.header.frame_id = "map";
            waypoint.header.stamp = this->get_clock()->now();
            waypoint.pose.position.x = this->get_parameter(x_key).as_double();
            waypoint.pose.position.y = this->get_parameter(y_key).as_double();
            waypoint.pose.position.z = this->get_parameter(z_key).as_double();
            waypoint.pose.orientation.w = 1.0;
            
            // Optionally load dwell time if available
            if (this->has_parameter(dwell_time_key)) {
                double dwell_time = this->get_parameter(dwell_time_key).as_double();
                RCLCPP_DEBUG(this->get_logger(), "Waypoint %d dwell time: %.2f", i, dwell_time);
                // You could store this in a custom message or handle it in your path planner
            }
            
            waypoints.push_back(waypoint);
            RCLCPP_INFO(this->get_logger(), "Loaded waypoint %d: [%.2f, %.2f, %.2f]", 
                       i, waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
            i++;
        }
        
        if (!waypoints.empty()) {
            path_planner_->setWaypoints(waypoints);
            RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from parameters for %s", waypoints.size(), drone_id_.c_str());
            
            // Load mission parameters if available
            loadMissionParams();
        } else {
            RCLCPP_WARN(this->get_logger(), "No waypoints found in parameters for %s - using fallback", drone_id_.c_str());
            loadFallbackWaypoints();
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading waypoints from parameters: %s", e.what());
        RCLCPP_INFO(this->get_logger(), "Loading fallback waypoints for %s", drone_id_.c_str());
        loadFallbackWaypoints();
    }
}

// Load mission parameters
void MissionPlannerNode::loadMissionParams() {
    try {
        // Load mission parameters if they exist
        if (this->has_parameter("mission_params.takeoff_altitude")) {
            double takeoff_altitude = this->get_parameter("mission_params.takeoff_altitude").as_double();
            RCLCPP_INFO(this->get_logger(), "Takeoff altitude: %.2f", takeoff_altitude);
            // Store or use this parameter as needed
        }
        
        if (this->has_parameter("mission_params.landing_speed")) {
            double landing_speed = this->get_parameter("mission_params.landing_speed").as_double();
            RCLCPP_INFO(this->get_logger(), "Landing speed: %.2f", landing_speed);
        }
        
        if (this->has_parameter("mission_params.waypoint_tolerance")) {
            double yaml_tolerance = this->get_parameter("mission_params.waypoint_tolerance").as_double();
            // Override the default tolerance with YAML value
            waypoint_tolerance_ = yaml_tolerance;
            RCLCPP_INFO(this->get_logger(), "Updated waypoint tolerance to: %.2f", waypoint_tolerance_);
        }
        
        if (this->has_parameter("mission_params.max_velocity")) {
            double max_velocity = this->get_parameter("mission_params.max_velocity").as_double();
            RCLCPP_INFO(this->get_logger(), "Max velocity: %.2f", max_velocity);
        }
        
        if (this->has_parameter("mission_params.loop_missions")) {
            bool loop_missions = this->get_parameter("mission_params.loop_missions").as_bool();
            RCLCPP_INFO(this->get_logger(), "Loop missions: %s", loop_missions ? "true" : "false");
            // @TODO store this in a member variable for use in mission execution
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error loading mission parameters: %s", e.what());
    }
}


  // Fallback function in case YAML loading fails:
  void MissionPlannerNode::loadFallbackWaypoints() {
      std::vector<geometry_msgs::msg::PoseStamped> waypoints;
      
      // Simple fallback waypoint for each drone
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.header.stamp = this->get_clock()->now();
      waypoint.pose.orientation.w = 1.0;
      
      if (drone_id_ == "rs1_drone_1") {
          waypoint.pose.position.x = 4.0;
          waypoint.pose.position.y = 16.0;
          waypoint.pose.position.z = 19.0;
      } else if (drone_id_ == "rs1_drone_2") {
          waypoint.pose.position.x = -5.0;
          waypoint.pose.position.y = 5.0;
          waypoint.pose.position.z = 15.0;
      } else if (drone_id_ == "rs1_drone_3") {
          waypoint.pose.position.x = -5.0;
          waypoint.pose.position.y = -5.0;
          waypoint.pose.position.z = 3.0;
      } else if (drone_id_ == "rs1_drone_4") {
          waypoint.pose.position.x = 5.0;
          waypoint.pose.position.y = -5.0;
          waypoint.pose.position.z = 8.0;
      } else {
          waypoint.pose.position.x = 6.0;
          waypoint.pose.position.y = -6.0;
          waypoint.pose.position.z = 10.0;
      }
      
      waypoints.push_back(waypoint);
      path_planner_->setWaypoints(waypoints);
      RCLCPP_INFO(this->get_logger(), "Loaded fallback waypoint for %s", drone_id_.c_str());
  }

  // Service callback implementations
  void MissionPlannerNode::startMissionCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // Suppress unused parameter warning
    
    RCLCPP_INFO(this->get_logger(), "Start mission service called for %s", drone_id_.c_str());
    
    // Check if we can start mission (must be in IDLE state)
    if (state_machine_->getCurrentState() != MissionState::IDLE) {
      response->success = false;
      response->message = "Cannot start mission - drone not in IDLE state. Current state: " + 
                        state_machine_->getStateString();
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    // Transition to takeoff state
    state_machine_->setState(MissionState::TAKEOFF);
    
    // Publish mission state for drone controller
    std_msgs::msg::String state_msg;
    state_msg.data = state_machine_->getStateString();
    mission_state_pub_->publish(state_msg);
    
    response->success = true;
    response->message = "Mission started successfully - initiating takeoff sequence";
    
    RCLCPP_INFO(this->get_logger(), "Mission started for %s - transitioning to TAKEOFF", drone_id_.c_str());
  }

  void MissionPlannerNode::stopMissionCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // Suppress unused parameter warning
    
    RCLCPP_INFO(this->get_logger(), "Stop mission service called for %s", drone_id_.c_str());
    
    // Transition to landing state (or emergency if needed)
    MissionState current_state = state_machine_->getCurrentState();
    
    if (current_state == MissionState::IDLE) {
      response->success = false;
      response->message = "Mission already stopped - drone in IDLE state";
      return;
    }
    
    // Initiate landing sequence
    state_machine_->setState(MissionState::LANDING);
    
    // Publish mission state for drone controller
    std_msgs::msg::String state_msg;
    state_msg.data = state_machine_->getStateString();
    mission_state_pub_->publish(state_msg);
    
    response->success = true;
    response->message = "Mission stop initiated - drone will land and return to IDLE";
    
    RCLCPP_INFO(this->get_logger(), "Mission stopped for %s - transitioning to LANDING", drone_id_.c_str());
  }


  // Mission execution logic
  void MissionPlannerNode::executeMission() {
    MissionState current_state = state_machine_->getCurrentState();
    
    switch (current_state) {
      case MissionState::IDLE:
        // Do nothing - waiting for mission start
        break;
        
      case MissionState::TAKEOFF:
        // Monitor takeoff completion - this would normally check altitude
        // For now, simulate takeoff completion after a delay
        takeoff();
        break;
        
      case MissionState::WAYPOINT_NAVIGATION:
        // Check if current waypoint is reached
        waypointNavigation();
        break;
        
      case MissionState::HOVERING:
        // Maintain position - could wait for new waypoints or manual commands
        hovering();
        break;
        
      case MissionState::LANDING:
        // Monitor landing completion
        landing();
        break;
        
      case MissionState::MANUAL_CONTROL:
        // External control - monitor for return to autonomous mode
        manualControl();
        break;
        
      case MissionState::EMERGENCY:
        // Emergency handling - immediate landing
        emergency();
        break;

      case MissionState::WILDFIRE_REACTION:
        wildFireReaction();
        break;

      case MissionState::DEBRIS_OBSTRUCTION_REACTION:
        debrisReaction();
        break;

      case MissionState::STRANDED_HIKER_REACTION:
        strandedHikerReaction();
        break;

      case MissionState::ORBIT_INCIDENT:
        orbitIncident();
        break;
    }
  }

  void MissionPlannerNode::takeoff() {
    static auto takeoff_start = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::steady_clock::now() - takeoff_start;
    
    if (elapsed > std::chrono::seconds(5)) { // 5 second takeoff simulation
      RCLCPP_INFO(this->get_logger(), "Takeoff completed for %s - transitioning to waypoint navigation", drone_id_.c_str());
      
      // Check if we have waypoints to navigate
      if (path_planner_->hasNextWaypoint()) {
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      } else {
        state_machine_->setState(MissionState::HOVERING);
        RCLCPP_INFO(this->get_logger(), "No waypoints available - transitioning to HOVERING");
      }
      takeoff_start = std::chrono::steady_clock::now(); // Reset for next time
    }   
  }


  void MissionPlannerNode::hovering() {
    RCLCPP_DEBUG(this->get_logger(), "Hovering at current position");
  }

  void MissionPlannerNode::manualControl() {
    RCLCPP_DEBUG(this->get_logger(), "In manual control mode");
  }

  void MissionPlannerNode::emergency() {
    RCLCPP_WARN(this->get_logger(), "Emergency state active - initiating emergency landing");
    state_machine_->setState(MissionState::LANDING);
  }

  void MissionPlannerNode::wildFireReaction() {
  }

  void MissionPlannerNode::debrisReaction() {}
  void MissionPlannerNode::strandedHikerReaction() {}
  void MissionPlannerNode::orbitIncident() {
    hovering(); // stub for now
  }

  void MissionPlannerNode::publishMissionCommand() {
    if (state_machine_->getCurrentState() == MissionState::WAYPOINT_NAVIGATION && 
        path_planner_->hasNextWaypoint()) {
      
      // Get current waypoint and publish as target pose
      geometry_msgs::msg::PoseStamped current_waypoint = path_planner_->getCurrentWaypoint();
      current_waypoint.header.stamp = this->get_clock()->now();
      current_waypoint.header.frame_id = "map";
      
      target_pose_pub_->publish(current_waypoint);
      
      // Calculate velocity commands to reach the waypoint
      double dx = current_waypoint.pose.position.x - current_pose_.pose.position.x;
      double dy = current_waypoint.pose.position.y - current_pose_.pose.position.y;
      double dz = current_waypoint.pose.position.z - current_pose_.pose.position.z;
      
      // Simple proportional controller
      double kp_xy = 1.0;  // XY gain
      double kp_z = 0.5;   // Z gain
      double max_vel_xy = 2.0;  // Max horizontal velocity
      double max_vel_z = 1.0;   // Max vertical velocity
      
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = std::max(-max_vel_xy, std::min(max_vel_xy, kp_xy * dx));
      cmd_vel.linear.y = std::max(-max_vel_xy, std::min(max_vel_xy, kp_xy * dy));
      cmd_vel.linear.z = std::max(-max_vel_z, std::min(max_vel_z, kp_z * dz));
      
      // Publish velocity command
      cmd_vel_pub_->publish(cmd_vel);
      
      RCLCPP_DEBUG(this->get_logger(), "Published waypoint: [%.2f, %.2f, %.2f], vel: [%.2f, %.2f, %.2f]",
                  current_waypoint.pose.position.x,
                  current_waypoint.pose.position.y,
                  current_waypoint.pose.position.z,
                  cmd_vel.linear.x,
                  cmd_vel.linear.y,
                  cmd_vel.linear.z);
    } else {
        // Only publish a stop if *we* own motion in this state
        auto s = state_machine_->getCurrentState();
        bool we_control = (s == MissionState::IDLE || s == MissionState::HOVERING || s == MissionState::EMERGENCY);
        if (we_control) {
          // Stop the drone if not navigating
          geometry_msgs::msg::Twist stop_cmd;
          stop_cmd.linear.x = 0.0;
          stop_cmd.linear.y = 0.0;
          stop_cmd.linear.z = 0.0;
          stop_cmd.angular.x = 0.0;
          stop_cmd.angular.y = 0.0;
          stop_cmd.angular.z = 0.0;
          cmd_vel_pub_->publish(stop_cmd);
        }


    }
  }

  // Helper methods
  bool MissionPlannerNode::isWaypointReached() const {
    if (!path_planner_->hasNextWaypoint()) {
      return true;
    }
    
    double distance = path_planner_->getDistanceToWaypoint(current_pose_);
    return distance < waypoint_tolerance_;
  }

  void MissionPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert odometry to pose for mission planning
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
  }

  void MissionPlannerNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    current_velocity_ = *msg;
  }

  void MissionPlannerNode::waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received waypoint command for %s: [%.2f, %.2f, %.2f]", 
                drone_id_.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // Add waypoint to path planner
    std::vector<geometry_msgs::msg::PoseStamped> new_waypoint = {*msg};
    path_planner_->setWaypoints(new_waypoint);
    
    // If drone is idle, start mission automatically
    if (state_machine_->getCurrentState() == MissionState::IDLE) {
      RCLCPP_INFO(this->get_logger(), "Auto-starting mission for waypoint command");
      state_machine_->setState(MissionState::TAKEOFF);
    }
    // If drone is hovering, switch to waypoint navigation
    else if (state_machine_->getCurrentState() == MissionState::HOVERING) {
      RCLCPP_INFO(this->get_logger(), "Switching to waypoint navigation mode");
      state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
    }
  }

  // ---------- tiny string/parse helpers (local-only) ----------
  static inline std::string trimCopy(std::string s) {
    auto notSpace = [](unsigned char c){ return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
  }

  bool MissionPlannerNode::parseKeyVal(const std::string& tok, std::string& key, std::string& val) {
    auto p = tok.find(':');
    if (p == std::string::npos) return false;
    key = trimCopy(tok.substr(0, p));
    val = trimCopy(tok.substr(p + 1));
    return !key.empty();
  }

  MissionState MissionPlannerNode::stateFromString(const std::string& s) {
    // map the strings you already use in getStateString()
    if (s == "IDLE") return MissionState::IDLE;
    if (s == "TAKEOFF") return MissionState::TAKEOFF;
    if (s == "WAYPOINT_NAVIGATION") return MissionState::WAYPOINT_NAVIGATION;
    if (s == "HOVERING") return MissionState::HOVERING;
    if (s == "LANDING") return MissionState::LANDING;
    if (s == "MANUAL_CONTROL") return MissionState::MANUAL_CONTROL;
    if (s == "EMERGENCY") return MissionState::EMERGENCY;
    if (s == "WILDFIRE_REACTION") return MissionState::WILDFIRE_REACTION;
    if (s == "ORBIT_INCIDENT") return MissionState::ORBIT_INCIDENT;
    if (s == "DEBRIS_OBSTRUCTION_REACTION") return MissionState::DEBRIS_OBSTRUCTION_REACTION;
    if (s == "STRANDED_HIKER_REACTION") return MissionState::STRANDED_HIKER_REACTION;
    return MissionState::IDLE;
  }

  static inline std::vector<std::string> splitCSV(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
      if (c == ',') { out.push_back(cur); cur.clear(); }
      else { cur.push_back(c); }
    }
    out.push_back(cur);
    return out;
  }

  static inline bool parseDouble(const std::string& s, double& out) {
    try {
      size_t idx = 0;
      out = std::stod(s, &idx);
      return idx == s.size() && std::isfinite(out);
    } catch (...) { return false; }
  }

  static inline bool parseRespondFlag(const std::string& token, bool& out_flag) {
    // Accept: respond:1|0|true|false|yes|no (case-insensitive)
    auto pos = token.find(':');
    if (pos == std::string::npos) return false;
    auto key = trimCopy(token.substr(0, pos));
    auto val = trimCopy(token.substr(pos + 1));
    std::string v = val;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    if (key != "respond") return false;
    if (v == "1" || v == "true" || v == "yes") { out_flag = true;  return true; }
    if (v == "0" || v == "false"|| v == "no")  { out_flag = false; return true; }
    return false;
  }

  // ---------- mapper ----------
  Scenario MissionPlannerNode::scenarioFromString(const std::string& s) {
    // Exact matches published by perception
    if (s == "STRANDED_HIKER")     return Scenario::STRANDED_HIKER;
    if (s == "WILDFIRE")           return Scenario::WILDFIRE;
    if (s == "DEBRIS_OBSTRUCTION") return Scenario::DEBRIS_OBSTRUCTION;

    return Scenario::UNKNOWN;
  }

  static std::string missionStateToString(MissionState state) {
      switch (state) {
          case MissionState::IDLE: return "IDLE";
          case MissionState::TAKEOFF: return "TAKEOFF";
          case MissionState::WAYPOINT_NAVIGATION: return "WAYPOINT_NAVIGATION";
          case MissionState::HOVERING: return "HOVERING";
          case MissionState::LANDING: return "LANDING";
          case MissionState::MANUAL_CONTROL: return "MANUAL_CONTROL";
          case MissionState::EMERGENCY: return "EMERGENCY";
          case MissionState::WILDFIRE_REACTION: return "WILDFIRE_REACTION";
          case MissionState::ORBIT_INCIDENT: return "ORBIT_INCIDENT";
          case MissionState::STRANDED_HIKER_REACTION: return "STRANDED_HIKER_REACTION";
          case MissionState::DEBRIS_OBSTRUCTION_REACTION: return "DEBRIS_OBSTRUCTION_REACTION";
          default: return "UNKNOWN";
      }
  }

  MissionState MissionPlannerNode::targetStateForScenario(Scenario scenario) {
    switch (scenario) {
      case Scenario::STRANDED_HIKER:
        // Navigate to the reported location to assist / inspect
        return MissionState::STRANDED_HIKER_REACTION;
      case Scenario::WILDFIRE:
        // Hold position and observe / report
        return MissionState::WILDFIRE_REACTION; // This could be changed later on.
      case Scenario::DEBRIS_OBSTRUCTION:
        // Navigate to inspect / clear obstruction
        return MissionState::DEBRIS_OBSTRUCTION_REACTION;
      case Scenario::UNKNOWN:
      default:
        // Unknown scenarios: remain idle / do not autonomously act
        return MissionState::IDLE;
    }
  }

  /**
   * @brief Parse scenario detection message from perception package
   * 
   * Takes a string in the format:
   *   "SCENARIO_NAME,severity,x,y,z,yaw,respond:1"
   * 
   * Example input:
   *   "STRANDED_HIKER,4,10.5,5.2,2.1,1.57,respond:1"
   * 
   * @param message_data The raw string from /scenario_detection topic
   * @return ScenarioData struct with parsed values (check .valid field for success)
   */
  ScenarioData parseScenarioMessage(const std::string& message_data) {
    ScenarioData result;
    result.valid = false;  // Assume failure until we succeed
    
    // Split the message by commas
    std::vector<std::string> parts;
    std::string current_part;
    for (char c : message_data) {
      if (c == ',') {
        parts.push_back(current_part);
        current_part.clear();
      } else {
        current_part += c;
      }
    }
    parts.push_back(current_part);  // Add the last part
    
    // Check we have exactly 7 parts
    if (parts.size() != 7) {
      RCLCPP_WARN(rclcpp::get_logger("scenario_parser"),
                  "Expected 7 fields, got %zu in message: '%s'",
                  parts.size(), message_data.c_str());
      return result;
    }
    
    // Parse each field with error checking
    try {
      // Field 0: Scenario name (string)
      result.scenario_name = parts[0];
      
      // Field 1: Severity (integer)
      result.severity = std::stoi(parts[1]);
      
      // Field 2: X position (double)
      result.x = std::stod(parts[2]);
      
      // Field 3: Y position (double)
      result.y = std::stod(parts[3]);
      
      // Field 4: Z position (double)
      result.z = std::stod(parts[4]);
      
      // Field 5: Yaw angle (double, in radians)
      result.yaw = std::stod(parts[5]);
      
      // Field 6: Respond flag (format: "respond:1" or "respond:0")
      std::string respond_field = parts[6];
      if (respond_field.find("respond:") == 0) {
        std::string value = respond_field.substr(8);  // Skip "respond:"
        result.can_respond = (value == "1" || value == "true");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("scenario_parser"),
                    "Invalid respond field: '%s'", respond_field.c_str());
        return result;
      }
      
      // If we made it here, parsing succeeded
      result.valid = true;
      
      RCLCPP_INFO(rclcpp::get_logger("scenario_parser"),
                  "Parsed scenario: %s at [%.2f, %.2f, %.2f], severity=%d, respond=%s",
                  result.scenario_name.c_str(), result.x, result.y, result.z,
                  result.severity, result.can_respond ? "yes" : "no");
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("scenario_parser"),
                  "Failed to parse scenario message: %s", e.what());
      result.valid = false;
    }
    
    return result;
  }


  // ---------- subscriber callback ----------
  void MissionPlannerNode::scenarioDetectionCallback(const std_msgs::msg::String::SharedPtr msg) {
    // Parse the incoming message
    ScenarioData scenario = parseScenarioMessage(msg->data);
    if (!scenario.valid) {
      RCLCPP_WARN(this->get_logger(), "Received invalid scenario message");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Scenario detected: %s at [%.2f, %.2f, %.2f]",
                scenario.scenario_name.c_str(), scenario.x, scenario.y, scenario.z);

    // Hand off the (potentially blocking) coordination work to a background thread
    // so the container’s executor stays free to run subscription callbacks.
    std::thread([this, scenario]() {
      if (scenario.scenario_name == "STRANDED_HIKER" ||
          scenario.scenario_name == "WILDFIRE") {
        sendMissionToLowestDrone(scenario);
      } else if (scenario.scenario_name == "DEBRIS_OBSTRUCTION") {
        RCLCPP_INFO(this->get_logger(), "Debris detected - notifying GUI only");
      }
    }).detach();
  }


  DroneInfo MissionPlannerNode::parseInfoManifest(const std::string& manifest_data) {
    DroneInfo result;
    result.valid = false;
    result.battery_level = 0.0;
    result.drone_id = -1;
    
    auto toks = splitCSV(manifest_data);
    
    try {
      for (auto& t : toks) {
        std::string k, v;
        if (!parseKeyVal(trimCopy(t), k, v)) continue;
        
        if (k == "id") result.drone_id = std::stoi(v);
        else if (k == "battery") result.battery_level = std::stod(v);
        else if (k == "state") result.mission_state = v;
        else if (k == "x") result.x = std::stod(v);
        else if (k == "y") result.y = std::stod(v);
        else if (k == "z") result.z = std::stod(v);
        else if (k == "t") result.timestamp = rclcpp::Time(std::stoll(v));
      }
      
      if (result.drone_id > 0) result.valid = true;
      
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse info manifest: %s", e.what());
      result.valid = false;
    }
    
    return result;
  }

  int MissionPlannerNode::selectBestResponderDrone(
      const std::vector<int>& all_drone_ids, MissionState required_state) {
    
    RCLCPP_INFO(this->get_logger(), 
                "Selecting best responder for state: %s",
                missionStateToString(required_state).c_str());
    
    auto drone_data = pingDronesForInfo(all_drone_ids, 5000);
    // auto fut = std::async(std::launch::async, [this, &all_drone_ids](){
    //   return pingDronesForInfo(all_drone_ids, 5000);
    // });
    
    // auto drone_data = fut.get(); 

    struct Candidate { int id; double distance; double battery; std::string state; };
    std::vector<Candidate> candidates;
    
    for (const auto& [drone_id, info] : drone_data) {
      if (!info.valid) {
        RCLCPP_DEBUG(this->get_logger(), "Drone %d: No response (skipping)", drone_id);
        continue;
      }
      
      if (info.battery_level < 0.5) {
        RCLCPP_DEBUG(this->get_logger(), "Drone %d: Low battery %.0f%% (skipping)", drone_id, info.battery_level * 100.0);
        continue;
      }
      
      MissionState current_state = stateFromString(info.mission_state);
      if (!canStateTransitionTo(current_state, required_state)) {
        RCLCPP_DEBUG(this->get_logger(), "Drone %d: Cannot transition from %s to required state (skipping)",
                    drone_id, info.mission_state.c_str());
        continue;
      }
      
      double dx = info.x - helipad_location_.x;
      double dy = info.y - helipad_location_.y;
      double dz = info.z - helipad_location_.z;
      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      candidates.push_back({drone_id, distance, info.battery_level, info.mission_state});
      
      RCLCPP_INFO(this->get_logger(), "Drone %d: CANDIDATE - battery=%.0f%%, state=%s, dist_to_helipad=%.2fm",
                  drone_id, info.battery_level * 100.0, info.mission_state.c_str(), distance);
    }
    
    if (candidates.empty()) {
      RCLCPP_WARN(this->get_logger(), "No suitable drones found! (all failed battery/state checks)");
      return -1;
    }
    
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.distance < b.distance; });
    
    int best_drone_id = candidates[0].id;
    
    RCLCPP_INFO(this->get_logger(), "✓ Selected drone %d: battery=%.0f%%, dist=%.2fm, state=%s",
                best_drone_id, candidates[0].battery * 100.0, candidates[0].distance, candidates[0].state.c_str());
    
    return best_drone_id;
  }

  void MissionPlannerNode::createPeerSubscriptionForId(int peer_id) {
    const std::string ns = "/rs1_drone_" + std::to_string(peer_id);

    // If we already wired the critical subscription for this peer, we're done.
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      if (info_manifest_subs_.count(peer_id)) {
        return;
      }
    }

    // QoS (match peers): reliable + volatile, keep_last(10)
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    // Create handles first (outside the lock)
    auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        ns + "/odom", 10,
        [this, peer_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> guard(peers_mutex_);
          peer_poses_[peer_id].header = msg->header;
          peer_poses_[peer_id].pose   = msg->pose.pose;
        });

    auto assignment_pub = this->create_publisher<std_msgs::msg::String>(
        ns + "/mission_assignment", reliable_qos);

    auto info_req_pub = this->create_publisher<std_msgs::msg::Empty>(
        ns + "/info_request", reliable_qos);

    auto info_sub = this->create_subscription<std_msgs::msg::String>(
        ns + "/info_manifest", reliable_qos,
        [this, peer_id](const std_msgs::msg::String::SharedPtr msg) {
          infoManifestCallback(peer_id, msg);
        });

    // Atomically install all endpoints (single lock)
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);

      // Another thread may have finished wiring while we were creating handles.
      if (!info_manifest_subs_.count(peer_id)) {
        peer_odom_subs_[peer_id]     = std::move(odom_sub);
        assignment_pubs_[peer_id]    = std::move(assignment_pub);
        info_request_pubs_[peer_id]  = std::move(info_req_pub);
        info_manifest_subs_[peer_id] = std::move(info_sub);

        RCLCPP_INFO(this->get_logger(),
          "Peer %d wired: SUB<- %s, PUB-> %s & %s",
          peer_id,
          (ns + "/info_manifest").c_str(),
          (ns + "/mission_assignment").c_str(),
          (ns + "/info_request").c_str());
      }
    }
  }

  // --- Minimal CSV helpers ---
  static inline std::string csv_escape(std::string s) {
    if (s.find_first_of(",\"\n") != std::string::npos) {
      for (size_t p = 0; (p = s.find('"', p)) != std::string::npos; p += 2) s.insert(p, 1, '"');
      return "\"" + s + "\"";
    }
    return s;
  }

  // Build exactly: drone_id,incident_id,title,severity,iso_time,x,y,z,description
  static inline std::string make_incident_csv(
      int drone_id, const std::string& incident_id,
      const std::string& title, int severity,
      const std::string& iso_time,
      double x, double y, double z,
      const std::string& description) {

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << drone_id << ","
      << incident_id << ","
      << csv_escape(title) << ","
      << std::max(1, std::min(3, severity)) << ","
      << iso_time << ","
      << x << "," << y << "," << z << ","
      << csv_escape(description);
    return ss.str();
  }

  void MissionPlannerNode::alertIncidentGui(const std::optional<ScenarioEvent>& event) {
    if (!event) return;
    const ScenarioEvent& e = *event;

    // Make ISO8601 (simple, thread-safe)
    auto to_iso_utc = [&](const rclcpp::Time& t) {
      using namespace std::chrono;
      auto ns = nanoseconds(t.nanoseconds());
      auto tp = time_point<std::chrono::system_clock>(duration_cast<std::chrono::system_clock::duration>(ns));
      std::time_t tt = std::chrono::system_clock::to_time_t(tp);
      char buf[32]{0};
  #if defined(_WIN32)
      std::tm g{}; gmtime_s(&g, &tt);
      std::strftime(buf, sizeof(buf), "%FT%TZ", &g);
  #else
      std::tm g{}; gmtime_r(&tt, &g);
      std::strftime(buf, sizeof(buf), "%FT%TZ", &g);
  #endif
      return std::string(buf[0] ? buf : "1970-01-01T00:00:00Z");
    };

    const int drone_num = drone_numeric_id_;              // MUST be numeric (see fix below)
    const std::string id = "INC-" + std::to_string(++incident_counter_);
    const std::string title = evTypeToString(e.type);     // see fix below
    const int severity = 1;                                // keep simple for now
    const std::string iso = to_iso_utc(e.stamp);

    std_msgs::msg::String msg;
    msg.data = make_incident_csv(
        drone_num, id, title, severity, iso,
        e.target.x, e.target.y, e.target.z,
        ""  // description (optional)
    );
    if (incident_pub_) incident_pub_->publish(msg);
  }

  const char* MissionPlannerNode::evTypeToString(Scenario s) const {
    switch (s) {
      case Scenario::STRANDED_HIKER:     return "STRANDED_HIKER";
      case Scenario::WILDFIRE:           return "WILDFIRE";
      case Scenario::DEBRIS_OBSTRUCTION: return "DEBRIS_OBSTRUCTION";
      default:                           return "UNKNOWN";
    }
  }

  void MissionPlannerNode::discoverPeerDrones() {
    // Get all topic names/types currently visible in the graph.
    const auto topic_names_and_types = this->get_topic_names_and_types();

    // We consider any topic matching /rs1_drone_<ID>/odom to indicate a peer drone.
    const std::regex odom_topic_regex(R"(^/rs1_drone_([0-9]+)/odom$)");

    // Collect the set of peer IDs we detect from the topic list (excluding ourselves).
    std::set<int> detected_peer_ids;

    for (const auto &entry : topic_names_and_types) {
      const std::string &topic_name = entry.first;
      std::smatch match;
      if (std::regex_match(topic_name, match, odom_topic_regex) && match.size() > 1) {
        try {
          const int peer_id = std::stoi(match[1].str());
          if (peer_id == drone_numeric_id_) continue;  // skip our own ID
          detected_peer_ids.insert(peer_id);
        } catch (...) {
          // Ignore parse errors and keep scanning
        }
      }
    }

    // Create subscriptions for discovered peers
    for (int peer_id : detected_peer_ids) {
      createPeerSubscriptionForId(peer_id);
    }
  }

  // Apparently removing inside of the lock could cause issues but idc for now, @jackson can look into this
  void MissionPlannerNode::removePeerSubscriptionForId(int id) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    peer_odom_subs_.erase(id);
    peer_poses_.erase(id);
    assignment_pubs_.erase(id);
    RCLCPP_INFO(this->get_logger(), "Peer drone %d removed", id);
  }

  void MissionPlannerNode::infoRequestPingCallback(const std_msgs::msg::Empty::SharedPtr) {
    std_msgs::msg::String out;
    out.data = buildInfoManifestCsv();
    info_manifest_pub_->publish(out);
    RCLCPP_DEBUG(this->get_logger(), "Published info manifest in response to ping");
  }
  
  std::string MissionPlannerNode::buildInfoManifestCsv() {
    // id:<n>,battery:<0-1>,state:<STATE>,x:..,y:..,z:..,t:<ns>
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "id:" << drone_numeric_id_
      << ",battery:" << std::clamp(battery_level_, 0.0, 1.0)
      << ",state:" << state_machine_->getStateString()
      << ",x:" << current_pose_.pose.position.x
      << ",y:" << current_pose_.pose.position.y
      << ",z:" << current_pose_.pose.position.z
      << ",t:" << this->now().nanoseconds();
    return ss.str();
  }

  
  // void MissionPlannerNode::infoManifestCallback(int peer_id, const std_msgs::msg::String::SharedPtr& msg) {
  //   RCLCPP_INFO(this->get_logger(), "🔵 CB fired: peer=%d msg=%s", peer_id, msg->data.c_str());
  //   // Expect: id:N,battery:0.80,state:STATE,x:..,y:..,z:..,t:..
  //   auto toks = splitCSV(msg->data);
  //   int id_from_msg = -1;
  //   PeerInfo pi;
  //   for (auto& t : toks) {
  //     std::string k, v;
  //     if (!parseKeyVal(trimCopy(t), k, v)) continue;
  //     if (k == "id")        { try { id_from_msg = std::stoi(v); } catch (...) {} }
  //     else if (k == "battery") { parseDouble(v, pi.battery); }
  //     else if (k == "state")   { pi.state = stateFromString(v); }
  //     else if (k == "x")       { pi.pose.pose.position.x = std::stod(v); }
  //     else if (k == "y")       { pi.pose.pose.position.y = std::stod(v); }
  //     else if (k == "z")       { pi.pose.pose.position.z = std::stod(v); }
  //   }
  //   pi.stamp = this->now();
  //   if (id_from_msg > 0 && id_from_msg == peer_id) {
  //     std::lock_guard<std::mutex> lock(peers_mutex_);
  //     peer_info_[peer_id] = pi;
  //   }
  // }

  void MissionPlannerNode::infoManifestCallback(int peer_id, const std_msgs::msg::String::SharedPtr& msg) {
    RCLCPP_INFO(this->get_logger(), "🔵 CB fired: peer=%d msg=%s", peer_id, msg->data.c_str());  // <— add

    auto toks = splitCSV(msg->data);
    int id_from_msg = -1;
    PeerInfo pi;

    for (auto& t : toks) {
      std::string k, v;
      if (!parseKeyVal(trimCopy(t), k, v)) continue;
      if (k == "id")        { try { id_from_msg = std::stoi(v); } catch (...) {} }
      else if (k == "battery") { parseDouble(v, pi.battery); }
      else if (k == "state")   { pi.state = stateFromString(v); }
      else if (k == "x")       { pi.pose.pose.position.x = std::stod(v); }
      else if (k == "y")       { pi.pose.pose.position.y = std::stod(v); }
      else if (k == "z")       { pi.pose.pose.position.z = std::stod(v); }
    }

    pi.stamp = this->now();
    

    // 🔧 TEMP: trust the subscription wiring (peer_id) over the self-reported id
    // If you keep the guard, at least log mismatches.
    if (id_from_msg > 0 && id_from_msg != peer_id) {
      RCLCPP_WARN(this->get_logger(), "Manifest id mismatch: msg_id=%d sub_peer=%d", id_from_msg, peer_id);
    }

    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      peer_info_[peer_id] = pi;  // <— don’t block on id match for now
    }

    RCLCPP_INFO(this->get_logger(), "✅ Stored peer_info[%d] stamp=%ld", peer_id, pi.stamp.nanoseconds());
  }
}  // namespace drone_swarm
