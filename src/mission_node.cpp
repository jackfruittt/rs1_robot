#include "mission_node.h"
#include <thread>

namespace drone_swarm
{
  //--- FORWARD DECLARATIONS for helper functions ---///
  static inline std::string trimCopy(std::string s);
  static inline std::vector<std::string> splitCSV(const std::string& s);
  static inline bool parseDouble(const std::string& s, double& out);
  ScenarioData parseScenarioMessage(const std::string& message_data);
  static std::string missionStateToString(MissionState state);

MissionPlannerNode::MissionPlannerNode(const rclcpp::NodeOptions& options, const std::string & name)
  : Node( (name.empty() ? std::string("mission_planner_") + std::to_string(getpid()) : name), rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
  {
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable(); // QoS (Quality of Service) to tell ROS2 to keep a 10 message buffer and re-send dropped messages so they always arrive

    // --- Parameter Loading (no declares) --- //
    this->get_parameter_or("drone_namespace", drone_namespace_, std::string("rs1_drone"));
    this->get_parameter_or("mission_update_rate", mission_update_rate_, 5.0);
    this->get_parameter_or("waypoint_tolerance",        waypoint_tolerance_,   0.5);
    this->get_parameter_or("helipad_location.x",        helipad_location_.x,   -40.0);
    this->get_parameter_or("helipad_location.y",        helipad_location_.y,   20.02);
    this->get_parameter_or("helipad_location.z",        helipad_location_.z,   19.7);
    this->get_parameter_or("battery_level",             battery_level_,        0.8);
    
    // Depot locations default to helipad/spawn location
    // Helipad is at fixed location [-40, 20.02, 19.7] for all drones
    // Fire retardant and medkits are stored at the helipad
    this->get_parameter_or("retardant_depot.x",         depot_xyz_.x,         helipad_location_.x);
    this->get_parameter_or("retardant_depot.y",         depot_xyz_.y,         helipad_location_.y);
    this->get_parameter_or("retardant_depot.z",         depot_xyz_.z,         helipad_location_.z);
    this->get_parameter_or("medkit_depot.x",            medkit_depot_xyz_.x,  helipad_location_.x);
    this->get_parameter_or("medkit_depot.y",            medkit_depot_xyz_.y,  helipad_location_.y);
    this->get_parameter_or("medkit_depot.z",            medkit_depot_xyz_.z,  helipad_location_.z);
    
    repeat_Waypoint_Path_ = true;

    //--- Component Initialisation ---///
    state_machine_ = std::make_unique<StateMachine>(); 
    path_planner_ = std::make_unique<WaypointPlanner>();
    mission_executor_ = std::make_unique<MissionExecutor>();
    
    // Configure mission executor with logger
    mission_executor_->setLogger(this->get_logger());

    // Set drone identifier from namespace
    drone_id_ = drone_namespace_;
    try {
        std::string num_part = drone_id_.substr(drone_id_.find_last_of('_') + 1);
        drone_numeric_id_ = std::stoi(num_part); // string to int
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "FATAL: Could not parse numeric ID from namespace: %s", drone_id_.c_str());
    }

    //--- Load custom waypoints for surveillance ---//
    // loadWaypointsFromParams();
    
    // Initialise takeoff-related variables
    current_sonar_range_ = 0.0;
    takeoff_in_progress_ = false;
    target_takeoff_altitude_ = 4.0; // 4 metres target altitude
    takeoff_complete_ = false;
    
    // Initialise landing-related variables
    landing_in_progress_ = false;
    target_landing_altitude_ = 0.2; // 0.2 metres (close to ground, sonar minimum)
    landing_complete_ = false;
    
    //--- Subs ---///
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + drone_namespace_ + "/odom", 10, std::bind(&MissionPlannerNode::odomCallback, this, std::placeholders::_1));
    sonar_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/" + drone_namespace_ + "/sonar", 10, std::bind(&MissionPlannerNode::sonarCallback, this, std::placeholders::_1));
    scenario_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/" + drone_namespace_ + "/scenario_detection", reliable_qos, std::bind(&MissionPlannerNode::scenarioDetectionCallback, this, std::placeholders::_1));
    info_request_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/" + drone_namespace_ + "/info_request", reliable_qos, std::bind(&MissionPlannerNode::infoRequestPingCallback, this, std::placeholders::_1));
    assignment_subs_ = this->create_subscription<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/mission_assignment", reliable_qos, 
      std::bind(&MissionPlannerNode::assignmentCallback, this, std::placeholders::_1));
    reset_mission_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/reset_mission", 10, std::bind(&MissionPlannerNode::resetMissioncallback, this, std::placeholders::_1));
    incident_dispatch_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fleet/incident_dispatch", reliable_qos, 
      std::bind(&MissionPlannerNode::incidentDispatchCallback, this, std::placeholders::_1));

    //--- Pubs ---//
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + drone_namespace_ + "/cmd_vel", 10);
    takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("/" + drone_namespace_ + "/takeoff", 10);
    land_pub_ = this->create_publisher<std_msgs::msg::Empty>("/" + drone_namespace_ + "/land", 10);
    
    // Configure mission executor with command velocity publisher
    mission_executor_->setCmdVelPublisher(cmd_vel_pub_);
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + drone_namespace_ + "/target_pose", 10);
    mission_state_pub_ = this->create_publisher<std_msgs::msg::String>("/" + drone_namespace_ + "/mission_state", reliable_qos);
    info_manifest_pub_ = this->create_publisher<std_msgs::msg::String>("/" + drone_namespace_ + "/info_manifest", reliable_qos);
    incident_pub_ = this->create_publisher<std_msgs::msg::String>("/" + drone_namespace_ + "/incident", 10);
    incident_dispatch_pub_ = this->create_publisher<std_msgs::msg::String>("/fleet/incident_dispatch", reliable_qos);

    //--- Srvs ---//
    start_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/start_mission", std::bind(&MissionPlannerNode::startMissionCallback, this, std::placeholders::_1, std::placeholders::_2)); // Tiny service: empty request, and response {bool success, string message}
    stop_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/stop_mission", std::bind(&MissionPlannerNode::stopMissionCallback, this, std::placeholders::_1, std::placeholders::_2));
    takeoff_drone_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/takeoff_drone", std::bind(&MissionPlannerNode::takeoffDroneCallback, this, std::placeholders::_1, std::placeholders::_2));
    land_drone_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/land_drone", std::bind(&MissionPlannerNode::landDroneCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Scenario reaction services
    wildfire_service_ = this->create_service<rs1_robot::srv::ReactToWildfire>(
      "/" + drone_namespace_ + "/react_to_wildfire", 
      std::bind(&MissionPlannerNode::wildfireReactionCallback, this, std::placeholders::_1, std::placeholders::_2));
    hiker_service_ = this->create_service<rs1_robot::srv::ReactToHiker>(
      "/" + drone_namespace_ + "/react_to_hiker",
      std::bind(&MissionPlannerNode::hikerRescueCallback, this, std::placeholders::_1, std::placeholders::_2));
    debris_service_ = this->create_service<rs1_robot::srv::ReactToDebris>(
      "/" + drone_namespace_ + "/react_to_debris",
      std::bind(&MissionPlannerNode::debrisReactionCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Scenario reaction service clients (for calling our own or peer services)
    wildfire_client_ = this->create_client<rs1_robot::srv::ReactToWildfire>(
      "/" + drone_namespace_ + "/react_to_wildfire");
    hiker_client_ = this->create_client<rs1_robot::srv::ReactToHiker>(
      "/" + drone_namespace_ + "/react_to_hiker");
    debris_client_ = this->create_client<rs1_robot::srv::ReactToDebris>(
      "/" + drone_namespace_ + "/react_to_debris");

    //--- Tims ---//
    auto mission_timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / mission_update_rate_));
    mission_timer_ = this->create_wall_timer(mission_timer_period, std::bind(&MissionPlannerNode::missionTimerCallback, this));
    discovery_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&MissionPlannerNode::discoverPeerDrones, this));
    
    // Broadcast status every 1 second for peer discovery (non-blocking for composed mode)
    status_broadcast_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          std_msgs::msg::String manifest;
          manifest.data = buildInfoManifestCsv();
          info_manifest_pub_->publish(manifest);
          RCLCPP_DEBUG(this->get_logger(), "Broadcasted status: %s", manifest.data.c_str());
        });
    
    waypoint_load_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
          this->loadWaypointsFromParams();
          waypoint_load_timer_->cancel();
        });
    mission_params_timer_ = this->create_wall_timer(std::chrono::milliseconds(750),[this]() {this->loadMissionParams();mission_params_timer_->cancel();});

    RCLCPP_INFO(this->get_logger(), "Mission Planner Node initialised for %s", drone_id_.c_str()); 

    //--- Finds drones that exist ---//
    discoverPeerDrones();
  }

  /************************** MATTHEW  ***************************************/
  std::map<int, DroneInfo> MissionPlannerNode::pingDronesForInfo(
      const std::vector<int>& drone_ids) {
    
    std::map<int, DroneInfo> results;
    // Separate self from peers
    std::vector<int> peers_to_ping;
    for (int id : drone_ids) {
      if (id == drone_numeric_id_) {
        DroneInfo self_info = parseInfoManifest(buildInfoManifestCsv());
        results[id] = self_info;
      } else {
        peers_to_ping.push_back(id);
        results[id] = {id, 0,0,0,0.0,"",this->now(),false}; // pre-fill results with invalid data
      }
    }

    if (peers_to_ping.empty()) {
      RCLCPP_INFO(this->get_logger(), "Ping complete: 1/1 responses (self)");
      return results;
    }
    
    // Ensure peer wiring exists before pinging
    for (int drone_id : peers_to_ping) {
      // ensure wiring exists
      createPeerSubscriptionForId(drone_id);

      // wait (briefly) for the DDS (Data Distribution Service, the pub/sub middleware under ROS 2) match so a one-shot ping won’t be dropped
      if (!waitForPeerPingSubscriber(drone_id, std::chrono::milliseconds(300))) {
        RCLCPP_WARN(this->get_logger(),
                    "Peer %d /info_request not matched yet; will send a few spaced pings as fallback.",
                    drone_id);
      }
    }
    // Record when we sent pings so we only accept responses after this
    auto ping_start_time = this->now();

    auto send_ping = [&](int id){
      std_msgs::msg::Empty e;
      auto it = info_request_pubs_.find(id);
      if (it != info_request_pubs_.end() && it->second) it->second->publish(e);
    };

    for (int id : peers_to_ping) send_ping(id);
    // rclcpp::sleep_for(std::chrono::milliseconds(120)); // NEED TO TEST IF NECESSARY OR NOT
    // for (int id : peers_to_ping) send_ping(id);
    // rclcpp::sleep_for(std::chrono::milliseconds(120));  
    // for (int id : peers_to_ping) send_ping(id);

    RCLCPP_INFO(this->get_logger(), "Reading cached peer info for %zu peers (non-blocking for composed mode)...", peers_to_ping.size()); 
    
    // COMPOSED MODE OPTIMISATION by Jackson: Skip ping requests, just use cached info
    // The info_manifest subscriptions continuously update peer_info_ in background
    // Sending ping + waiting blocks the executor in composed nodes
    
    // Immediately read cached peer info without waiting
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      for (int id : peers_to_ping) {
        auto it = peer_info_.find(id);
        if (it != peer_info_.end() && it->second.stamp.seconds() > 0) {
          // Found valid cached info
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
    
    // Log results
    int valid_count = 0;
    for (const auto& [id, info] : results) {
      if (info.valid) {
        valid_count++;
        RCLCPP_DEBUG(this->get_logger(), "Drone %d: Valid cached info (battery=%.0f%%, state=%s)", 
                    id, info.battery_level * 100.0, info.mission_state.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Drone %d: No cached info available", id);
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Ping complete: %d/%zu cached responses", 
                valid_count, peers_to_ping.size());
    
    return results;
  }

  /*************** JACKSON *******************/
  bool MissionPlannerNode::shouldSuppressIncident(const ScenarioData& s) {
    const auto now = steady_clock_.now();
    std::lock_guard<std::mutex> lk(dispatch_mutex_);
    auto it = dispatch_cooldown_.find(s.scenario_name);
    if (it == dispatch_cooldown_.end()) return false;

    auto& cd = it->second;
    
    // Same-incident proximity check first (2D only, ignore Z noise)
    const double dx = cd.target.x - s.x;
    const double dy = cd.target.y - s.y;
    const double d2_2d = dx*dx + dy*dy;  // 2D distance squared
    const double r2 = incident_merge_radius_m_ * incident_merge_radius_m_;
    if (d2_2d > r2) return false;  // far enough away: treat as new incident

    // If the responder is still non-IDLE, keep suppressing indefinitely (and slide the cooldown)
    {
      std::lock_guard<std::mutex> pl(peers_mutex_); // peer lock
      auto pit = peer_info_.find(cd.responder_id); // peer iterator
      if (pit != peer_info_.end() && pit->second.state != MissionState::IDLE) {
        cd.until = now + coordination_cooldown_;  // slide window forward while busy
        RCLCPP_INFO(this->get_logger(),
          "Suppressing duplicate %s (responder=%d still busy) within %.1fm (2D)",
          s.scenario_name.c_str(), cd.responder_id, incident_merge_radius_m_);
        return true;
      }
    }

    // Otherwise fall back to time-based cooldown
    if (now < cd.until) {
      RCLCPP_INFO(this->get_logger(),
        "Suppressing duplicate %s (responder=%d) within %.1fm (2D) & cooldown %.1fs remaining",
        s.scenario_name.c_str(), cd.responder_id, incident_merge_radius_m_,
        (cd.until - now).seconds());
      return true;
    }

    // Cooldown expired AND responder idle -> allow new coordination
    return false;
  }

  /****************** JACKSON AND MATTHEW ******************/
  void MissionPlannerNode::missionTimerCallback() {
      executeMission();

      // Hiker rescue
      if (in_hiker_rescue_ && medkit_collected_) {
          if (this->get_clock()->now() - medkit_collect_stamp_ > rclcpp::Duration::from_seconds(2.0)) {
              RCLCPP_INFO(this->get_logger(), "Medkit collected, proceeding to hiker location.");
              geometry_msgs::msg::PoseStamped wp_hiker;
              wp_hiker.header.frame_id = "map";
              wp_hiker.pose.position = hiker_target_xyz_;
              wp_hiker.pose.orientation.w = 1.0;
              path_planner_->setWaypoints({wp_hiker});
              state_machine_->setState(MissionState::TAKEOFF);
              medkit_collected_ = false;
              in_hiker_rescue_awaiting_takeoff_ = true;
          }
      }

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

    if (in_hiker_rescue_ && !medkit_collected_ && !in_hiker_rescue_awaiting_takeoff_) {
      RCLCPP_INFO(this->get_logger(), "Arrived at medkit depot. Landing to collect.");
      state_machine_->setState(MissionState::LANDING);
      return; 
    }

    (void)path_planner_->getNextWaypoint();
    if (!path_planner_->hasNextWaypoint()) {
      if (in_hiker_rescue_) {
        // We're in a managed multi-phase mission; phase logic/timers will advance us.
        return;
      }
      
      // Check if we just completed a scenario reaction mission
      if (in_scenario_reaction_) {
        RCLCPP_INFO(get_logger(), "Scenario reaction mission complete (%s). Restoring previous mission.", 
                    active_scenario_type_.c_str());
        
        // Mark incident as resolved by removing it from the fleet registry
        if (!active_scenario_incident_id_.empty()) {
          std::lock_guard<std::mutex> lock(registry_mutex_);
          auto it = fleet_incident_registry_.find(active_scenario_incident_id_);
          if (it != fleet_incident_registry_.end()) {
            RCLCPP_INFO(get_logger(), "Marking incident %s as RESOLVED and removing from registry",
                        active_scenario_incident_id_.c_str());
            
            // Add to recently resolved incidents list to prevent re-detection
            {
              std::lock_guard<std::mutex> resolved_lock(resolved_incidents_mutex_);
              ResolvedIncident resolved;
              resolved.location = it->second.location;
              resolved.scenario_name = it->second.scenario_name;
              resolved.resolved_at = this->get_clock()->now();
              recently_resolved_incidents_.push_back(resolved);
              
              RCLCPP_INFO(get_logger(), 
                         "Added %s at [%.2f, %.2f, %.2f] to ignore list for %.0f seconds",
                         resolved.scenario_name.c_str(),
                         resolved.location.x, resolved.location.y, resolved.location.z,
                         resolved_incident_ignore_duration_.seconds());
            }
            
            fleet_incident_registry_.erase(it);
          }
        }
        
        // Restore the saved mission state
        mission_executor_->restoreMissionState(path_planner_.get());
        
        // Clear the scenario reaction flag
        in_scenario_reaction_ = false;
        active_scenario_type_.clear();
        active_scenario_incident_id_.clear();  // Clear the incident ID
        
        // Decide what to do based on pre-mission state
        if (was_idle_before_reaction_) {
          // Drone was IDLE before mission - land at helipad and return to IDLE
          was_idle_before_reaction_ = false;  // Reset flag
          state_machine_->setState(MissionState::LANDING);
          RCLCPP_INFO(get_logger(), "Scenario mission complete. Drone was IDLE before, landing at helipad and returning to IDLE.");
        } else if (path_planner_->hasNextWaypoint()) {
          // Drone was on patrol - resume original waypoint navigation
          state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
          RCLCPP_INFO(get_logger(), "Resumed original mission from saved state");
        } else {
          // No waypoints to restore, go idle
          state_machine_->setState(MissionState::IDLE);
          RCLCPP_INFO(get_logger(), "Original mission was empty, returning to IDLE");
        }
        return;
      }
      
      if (repeat_Waypoint_Path_ == true) {
        // Generate new random waypoints for autonomous patrol
        // Using larger distances (8-15m) for better area coverage and exploration
        RCLCPP_INFO(get_logger(), "Final waypoint reached. Generating new random patrol route...");
        /**** JACKSONS TEMU RRT  ******/
        auto new_waypoints = path_planner_->generateRandomWaypoints(
            current_pose_,  // Start from current position
            3,              // Generate 3 waypoints (fewer but farther apart)
            8.0,            // Minimum 8m between points (explore wider area)
            15.0,           // Maximum 15m between points (better coverage)
            current_pose_.pose.position.z  // Maintain current altitude
        );
        
        if (!new_waypoints.empty()) {
          path_planner_->setWaypoints(new_waypoints);
          RCLCPP_INFO(get_logger(), "Generated %zu new waypoints for autonomous patrol", 
                     new_waypoints.size());
          
          // Log each waypoint coordinate for visibility
          for (size_t i = 0; i < new_waypoints.size(); ++i) {
            const auto& wp = new_waypoints[i];
            RCLCPP_INFO(get_logger(), "  Waypoint %zu: [%.2f, %.2f, %.2f]",
                       i + 1, wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
          }
        } else {
          // Fallback: just reset to beginning if generation failed
          path_planner_->reset();
          RCLCPP_WARN(get_logger(), "Random waypoint generation failed, resetting to original route");
        }
        return;
      }
      RCLCPP_INFO(get_logger(), "Final waypoint reached. Mission complete. Hovering.");
      state_machine_->setState(MissionState::HOVERING);
    } else {
      // Check if we just reached depot waypoint in a scenario mission (waypoint index 0)
      // Need to descend and collect payload (retardant or medkit)
      if (in_scenario_reaction_ && !payload_collected_ && 
          path_planner_->getCurrentWaypointIndex() == 0) {
        RCLCPP_INFO(get_logger(), "Reached depot location - descending to collect payload (checking sonar < 1m)");
        
        // Check if we've descended enough (sonar reading < 1m = close to ground)
        double current_altitude = 0.0;
        {
          std::lock_guard<std::mutex> lock(sonar_mutex_);
          current_altitude = current_sonar_range_;
        }
        
        if (current_altitude < 1.0) {
          // Close enough to ground - payload collected
          payload_collected_ = true;
          RCLCPP_INFO(get_logger(), "Payload collected at %.2fm altitude - ascending and continuing mission", 
                     current_altitude);
          // Continue to next waypoint
          (void)path_planner_->getNextWaypoint();
          RCLCPP_INFO(get_logger(), "Waypoint reached - moving to next waypoint");
        } else {
          // Still need to descend - don't advance waypoint yet
          RCLCPP_DEBUG(get_logger(), "Descending to depot: altitude %.2fm (need < 1.0m)", current_altitude);
        }
        return;
      }
      
      // Check if we just reached the fire/hiker waypoint (waypoint index 1, after depot)
      if (in_scenario_reaction_ && active_scenario_type_ == "WILDFIRE" && 
          path_planner_->getCurrentWaypointIndex() == 1) {
        RCLCPP_INFO(get_logger(), "Reached fire location - extinguishing fire for 2 seconds");
        
        // Brief pause to simulate extinguishing fire
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        RCLCPP_INFO(get_logger(), "Fire extinguished - continuing mission");
      }
      
      RCLCPP_INFO(get_logger(), "Waypoint reached - moving to next waypoint");
    }
  }

  /******************* JACKSON ********************************/
  void MissionPlannerNode::landing() {
    // Mission planner delegates actual flight execution to drone controller
    // We only monitor completion and handle mission-specific logic
    
    if (landing_in_progress_) {
      // Monitor altitude to determine when landing is complete
      double current_altitude = 0.0;
      {
        std::lock_guard<std::mutex> lock(sonar_mutex_);
        current_altitude = current_sonar_range_;
      }
      
      // Check if landing is complete (drone_controller handles the actual flight)
      if (current_altitude <= target_landing_altitude_ + 0.2) {
        landing_complete_ = true;
        landing_in_progress_ = false;
        
        // Transition to IDLE when landing complete
        state_machine_->setState(MissionState::IDLE);
        path_planner_->reset();
        // Reset flags
        in_hiker_rescue_ = false;
        in_hiker_rescue_awaiting_takeoff_ = false;
        RCLCPP_INFO(this->get_logger(), "Landing complete - transitioning to IDLE");
        return;
      }
      return; // Exit early for sonar-based landing
    }
    
    // Fallback to original timer-based landing for mission scenarios
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
      else {
        // mission-ending landing.
        state_machine_->setState(MissionState::IDLE);
        path_planner_->reset();
        // Reset flags
        in_hiker_rescue_ = false;
        in_hiker_rescue_awaiting_takeoff_ = false;
      }
      landing_timers.erase(drone_id_);
    }
  }

  /******************** MATTHEW **************************/
  void MissionPlannerNode::assignmentCallback(const std_msgs::msg::String::SharedPtr msg) {
    //--- Decode data ---///
    const std::string payload = trimCopy(msg->data);
    auto tokens = splitCSV(payload); 

    //--- Manage data (set wp, states, etc) ---///
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
      hiker_target_xyz_.z = hz + 1; // so drones don't crash

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

      if (tokens.size() == 5) {
        // set a repeat waypoint variable to false
        RCLCPP_INFO(get_logger(), "Single waypoint set, repeat_Waypoint_Path_ false");
        repeat_Waypoint_Path_ = false;
      } else {
        // set a repeat waypoint variable to true
        RCLCPP_INFO(get_logger(), "Multiple waypoints set, repeat_Waypoint_Path_ true");
        repeat_Waypoint_Path_ = true;
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
        waypoint.pose.position.z = current_pose_.pose.position.z;
        waypoint.pose.orientation.w = 1.0;
        
        new_waypoints.push_back(waypoint);
      }

      // Set the full list of waypoints in the planner
      path_planner_->setWaypoints(new_waypoints);
      {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        route_waypoints_cached_ = new_waypoints;
      }
      
      if (canStateTransitionTo(state_machine_->getCurrentState(), MissionState::WAYPOINT_NAVIGATION)) {
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      }
    }
  }
  
  std::vector<int> MissionPlannerNode::getKnownDroneIds() {
    std::vector<int> ids;
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      ids.reserve(info_manifest_subs_.size() + 1);
      for (const auto& kv : info_manifest_subs_) {
        ids.push_back(kv.first);        // discovered peer ids
      }
    }
    ids.push_back(drone_numeric_id_);   // include self
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
  }


  /*************************** MATTHEW (MOSTLY MATTHEW) *****************************/
  void MissionPlannerNode::performCoordination(const ScenarioData& scenario) {
    RCLCPP_INFO(this->get_logger(), "Coordinating response for %s", scenario.scenario_name.c_str());
    
    MissionState required_state = targetStateForScenario(scenarioFromString(scenario.scenario_name));

    // Make sure we’ve done a recent discovery pass
    discoverPeerDrones();
    
    // DEBUG: Log peer_info_ cache state
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      RCLCPP_INFO(this->get_logger(), "Peer cache: %zu entries", peer_info_.size());
      for (const auto& [id, info] : peer_info_) {
        RCLCPP_INFO(this->get_logger(), "  Drone %d: bat=%.0f%% state=%s age=%.1fs", 
                   id, info.battery * 100.0, missionStateToString(info.state).c_str(),
                   (this->now() - info.stamp).seconds());
      }
    }
    
    auto all_drones = getKnownDroneIds();        // dynamic list
    
    geometry_msgs::msg::Point incident;
    incident.x = scenario.x; incident.y = scenario.y; incident.z = scenario.z;
    std::vector<int> peer_drones = all_drones;
    peer_drones.erase(std::remove(peer_drones.begin(), peer_drones.end(), drone_numeric_id_),
                      peer_drones.end());

    // First, try to delegate to an IDLE peer
    int responder_id = selectBestResponderDrone(peer_drones, required_state, incident);
    
    // If no peer available, check if self can respond
    if (responder_id < 0) {
      RCLCPP_WARN(this->get_logger(), "No suitable peer drones available for scenario response");
      
      auto current_state = state_machine_->getCurrentState();
      
      // Self can respond if IDLE or WAYPOINT_NAVIGATION
      // WAYPOINT_NAVIGATION allows interrupting patrol to handle emergency scenarios
      if (current_state == MissionState::IDLE || 
          current_state == MissionState::WAYPOINT_NAVIGATION) {
        responder_id = drone_numeric_id_;
        RCLCPP_INFO(this->get_logger(), 
                   "Self-assigning mission: Drone %d (state: %s) will respond to scenario", 
                   responder_id, state_machine_->getStateString().c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), 
                    "Failed to find suitable responder! No available drones (including self).");
        RCLCPP_ERROR(this->get_logger(),
                    "Self state: %s (cannot interrupt for scenario response)",
                    state_machine_->getStateString().c_str());
        return;
      }
    }

    // Update dispatch cooldown with final responder_id
    {
      std::lock_guard<std::mutex> lk(dispatch_mutex_);
      auto it = dispatch_cooldown_.find(scenario.scenario_name);
      if (it != dispatch_cooldown_.end()) {
        // Update the existing cooldown entry with the selected responder
        it->second.responder_id = responder_id;
        it->second.until = steady_clock_.now() + coordination_cooldown_;
        RCLCPP_DEBUG(this->get_logger(), 
                    "Updated cooldown for %s with responder %d",
                    scenario.scenario_name.c_str(), responder_id);
      } else {
        // Fallback: create new entry if somehow not found
        DispatchCooldown cd; 
        cd.until        = steady_clock_.now() + coordination_cooldown_;
        cd.responder_id = responder_id;
        cd.target.x     = scenario.x;
        cd.target.y     = scenario.y;
        cd.target.z     = scenario.z;
        dispatch_cooldown_[scenario.scenario_name] = cd;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Generating incident ID for %s", scenario.scenario_name.c_str());
    std::string incident_id = generateIncidentId(scenario);  // e.g., "WILDFIRE_105_52_21"
    RCLCPP_INFO(this->get_logger(), "Generated incident ID: %s", incident_id.c_str());
    
    // Record incident dispatch for tracking
    RCLCPP_INFO(this->get_logger(), "Recording incident dispatch");
    recordIncidentDispatch(incident_id, scenario, responder_id);
    RCLCPP_INFO(this->get_logger(), "Incident dispatch recorded");


    /************* JACKSONS SR BASED ON SERVICE CALLS ******************/
    // Call appropriate service based on scenario type
    if (scenario.scenario_name == "STRANDED_HIKER") {
      RCLCPP_INFO(this->get_logger(), 
                 "Dispatching hiker rescue to drone %d via service call", responder_id);
      try {
        callHikerService(responder_id, scenario, incident_id);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Exception calling hiker service: %s", e.what());
      }
    }
    else if (scenario.scenario_name == "WILDFIRE") {
      RCLCPP_INFO(this->get_logger(),
                 "Dispatching wildfire response to drone %d via service call", responder_id);
      try {
        callWildfireService(responder_id, scenario, incident_id);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Exception calling wildfire service: %s", e.what());
      }
    }
    else if (scenario.scenario_name == "DEBRIS_OBSTRUCTION") {
      RCLCPP_INFO(this->get_logger(),
                 "Notifying drone %d about debris via service call", responder_id);
      try {
        callDebrisService(responder_id, scenario, incident_id);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Exception calling debris service: %s", e.what());
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), 
                 "Unknown scenario type: %s", scenario.scenario_name.c_str());
    }
  }

/************************* MATTHEW ***************************/
  std::string MissionPlannerNode::generateIncidentId(const ScenarioData& scenario) {
    // Create deterministic ID based on scenario type and rounded location
    // Format: "WILDFIRE_105_52_21" (type_x_y_z with 0.1m precision)
    // This ensures all drones generate the same ID for the same incident
    
    // Round coordinates to 1 decimal place (0.1m precision)
    // This groups nearby detections into the same incident
    auto round_coord = [](double val) -> int {
      return static_cast<int>(std::round(val * 10.0));  // 10.53 -> 105
    };
    
    int x_rounded = round_coord(scenario.x);
    int y_rounded = round_coord(scenario.y);
    int z_rounded = round_coord(scenario.z);
    
    std::ostringstream ss;
    ss << scenario.scenario_name << "_"
      << x_rounded << "_"
      << y_rounded << "_"
      << z_rounded;
    
    return ss.str();
    // Examples:
    // "WILDFIRE_105_52_21"
    // "STRANDED_HIKER_-283_140_140" (handles negatives)
  }

  void MissionPlannerNode::recordIncidentDispatch(
      const std::string& incident_id,
      const ScenarioData& scenario, 
      int responder_id) {
    
    auto now = this->now();
    
    // Create incident record
    ActiveIncident incident;
    incident.scenario_name = scenario.scenario_name;
    incident.location.x = scenario.x;
    incident.location.y = scenario.y;
    incident.location.z = scenario.z;
    incident.responder_id = responder_id;
    incident.dispatch_time = now;
    incident.expires_at = now + rclcpp::Duration::from_seconds(300);  // 5 min expiry
    
    // Update fleet-wide registry
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      fleet_incident_registry_[incident_id] = incident;
      
      RCLCPP_INFO(this->get_logger(),
                  "Recorded dispatch: %s (id=%s) → drone %d, expires in 300s",
                  scenario.scenario_name.c_str(), 
                  incident_id.c_str(),
                  responder_id);
    }
    
    // Broadcast incident dispatch to all drones for fleet-wide awareness
    std_msgs::msg::String dispatch_msg;
    std::ostringstream oss;
    oss << "DISPATCH," << incident_id << "," << scenario.scenario_name << ","
        << scenario.x << "," << scenario.y << "," << scenario.z << ","
        << responder_id << "," << now.nanoseconds();
    dispatch_msg.data = oss.str();
    incident_dispatch_pub_->publish(dispatch_msg);
    
    RCLCPP_INFO(this->get_logger(),
                "Broadcasted incident dispatch to fleet: %s",
                dispatch_msg.data.c_str());
    
    // Also update local dispatch cooldown for this drone's own tracking
    {
      std::lock_guard<std::mutex> lk(dispatch_mutex_);
      DispatchCooldown cd;
      cd.until = steady_clock_.now() + coordination_cooldown_;
      cd.responder_id = responder_id;
      cd.target.x = scenario.x;
      cd.target.y = scenario.y;
      cd.target.z = scenario.z;
      dispatch_cooldown_[scenario.scenario_name] = cd;
    }
  }

  void MissionPlannerNode::incidentDispatchCallback(
      const std_msgs::msg::String::SharedPtr msg) {
    
    auto tokens = splitCSV(msg->data);
    if (tokens.size() < 8 || tokens[0] != "DISPATCH") return;
    
    ActiveIncident incident;
    incident.scenario_name = tokens[2];
    incident.location.x = std::stod(tokens[3]);
    incident.location.y = std::stod(tokens[4]);
    incident.location.z = std::stod(tokens[5]);
    incident.responder_id = std::stoi(tokens[6]);
    incident.dispatch_time = rclcpp::Time(std::stoll(tokens[7]));
    incident.expires_at = incident.dispatch_time + rclcpp::Duration::from_seconds(300); // 5 min
    
    std::lock_guard<std::mutex> lock(registry_mutex_);
    fleet_incident_registry_[tokens[1]] = incident;  // tokens[1] is incident_id
    
    RCLCPP_INFO(this->get_logger(), 
                "Registry updated: %s at [%.1f,%.1f,%.1f] → drone %d",
                incident.scenario_name.c_str(), incident.location.x, 
                incident.location.y, incident.location.z, incident.responder_id);
  }

  bool MissionPlannerNode::isIncidentAlreadyManaged(const ScenarioData& scenario) {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    
    auto now = this->now();
    
    RCLCPP_INFO(this->get_logger(),
                "Checking if %s at [%.2f, %.2f, %.2f] is already managed (registry has %zu entries)",
                scenario.scenario_name.c_str(), scenario.x, scenario.y, scenario.z,
                fleet_incident_registry_.size());
    
    for (auto it = fleet_incident_registry_.begin(); 
        it != fleet_incident_registry_.end(); ) {
      
      // Clean up expired entries (with time source safety)
      try {
        if (now > it->second.expires_at) {
          RCLCPP_INFO(this->get_logger(), "Removing expired incident: %s", it->first.c_str());
          it = fleet_incident_registry_.erase(it);
          continue;
        }
      } catch (const std::runtime_error& e) {
        // Time sources don't match - skip expiry check for this entry
        RCLCPP_WARN(this->get_logger(), 
                    "Time comparison error for incident %s: %s (skipping expiry check)",
                    it->first.c_str(), e.what());
      }
      
      const auto& incident = it->second;
      
      // Check if same scenario type within merge radius
      if (incident.scenario_name == scenario.scenario_name) {
        double dx = incident.location.x - scenario.x;
        double dy = incident.location.y - scenario.y;
        // Ignore Z coordinate for incident matching due to perception noise
        double dist_sq_2d = dx*dx + dy*dy;
        double dist_2d = std::sqrt(dist_sq_2d);
        
        RCLCPP_INFO(this->get_logger(),
                    "Comparing with incident %s at [%.2f, %.2f, %.2f] handled by drone %d - 2D distance: %.2fm (threshold: %.2fm)",
                    it->first.c_str(), incident.location.x, incident.location.y, incident.location.z,
                    incident.responder_id, dist_2d, incident_merge_radius_m_);
        
        if (dist_sq_2d < (incident_merge_radius_m_ * incident_merge_radius_m_)) {
          RCLCPP_INFO(this->get_logger(),
                      "Incident already managed by drone %d (%.1fm away in 2D)",
                      incident.responder_id, dist_2d);
          return true;
        }
      }
      ++it;
    }
    
    RCLCPP_INFO(this->get_logger(), "No matching incident found in registry");
    return false;
  }

  // For manager drones to determine whether a fleet member's current state can switch to the target state
  bool MissionPlannerNode::canStateTransitionTo(MissionState current_state, MissionState target_state) {
    // Simplified state transitions - scenario reactions are services, not states
    switch (current_state) {
      case MissionState::IDLE:
        return  target_state == MissionState::TAKEOFF || 
                target_state == MissionState::MANUAL_CONTROL;

      case MissionState::TAKEOFF:
        return  target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::RESPONSE_NAVIGATION ||
                target_state == MissionState::HOVERING ||
                target_state == MissionState::EMERGENCY;

      case MissionState::WAYPOINT_NAVIGATION:
        return  target_state == MissionState::HOVERING || 
                target_state == MissionState::LANDING ||
                target_state == MissionState::EMERGENCY;
      
      case MissionState::RESPONSE_NAVIGATION:
        return  target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::HOVERING || 
                target_state == MissionState::LANDING ||
                target_state == MissionState::IDLE ||
                target_state == MissionState::EMERGENCY;

      case MissionState::HOVERING:
        return  target_state == MissionState::WAYPOINT_NAVIGATION ||
                target_state == MissionState::RESPONSE_NAVIGATION ||
                target_state == MissionState::LANDING ||
                target_state == MissionState::EMERGENCY;

      case MissionState::LANDING:
        return  target_state == MissionState::IDLE || 
                target_state == MissionState::EMERGENCY;

      case MissionState::MANUAL_CONTROL:
        return  target_state == MissionState::IDLE || 
                target_state == MissionState::EMERGENCY;

      case MissionState::EMERGENCY:
        return  target_state == MissionState::IDLE;
    }
    return false;
  }
  
  /******************* JACKSON ******************************/
  void MissionPlannerNode::loadWaypointsFromParams() {
    RCLCPP_INFO(get_logger(), "Loading waypoint params (enumerate-style) for %s", drone_id_.c_str());

    // List parameter names we can see under the "waypoints." prefix
    auto result = this->list_parameters({"waypoints"}, 10000); // depth large enough
    const auto& names = result.names;

    if (names.empty()) {
      RCLCPP_WARN(get_logger(), "No 'waypoints.*' parameters visible - using fallback");
      loadFallbackWaypoints();
      // loadMissionParams();    // still attempt to read mission params
      return;
    }

    // Extract indices present (waypoints.<idx>.*)
    std::set<int> indices;
    std::regex re(R"(waypoints\.([0-9]+)\.)");
    for (const auto& n: names) {
      std::smatch m; 
      if (std::regex_search(n, m, re) && m.size() > 1) {
        indices.insert(std::stoi(m[1].str()));
      }
    }

    std::vector<geometry_msgs::msg::PoseStamped> wps;
    for (int i : indices) {
      double x=0,y=0,z=0;
      // use get_parameter_or so undeclared params don’t throw
      this->get_parameter_or("waypoints." + std::to_string(i) + ".position.x", x, x);
      this->get_parameter_or("waypoints." + std::to_string(i) + ".position.y", y, y);
      this->get_parameter_or("waypoints." + std::to_string(i) + ".position.z", z, z);

      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = "map";
      wp.pose.orientation.w = 1.0;
      wp.pose.position.x = x; wp.pose.position.y = y; wp.pose.position.z = z;
      wps.push_back(wp);

      RCLCPP_INFO(get_logger(), "WP %d -> [%.2f, %.2f, %.2f]", i, x, y, z);
    }

    if (wps.empty()) {
      RCLCPP_WARN(get_logger(), "Waypoints keys existed but no positions parsed - using fallback");
      loadFallbackWaypoints();
    } else {
      path_planner_->setWaypoints(wps);
      {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        original_waypoints_cached_ = wps;
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints via enumeration", wps.size());
    }

    // Try mission params regardless
    // loadMissionParams();
  }

/****************************** JACKSONS MISSION PLANNING CALLBACK STUFF STARTS HERE **********/
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

  // Fallback function in case YAML loading fails
  void MissionPlannerNode::loadFallbackWaypoints() {
      std::vector<geometry_msgs::msg::PoseStamped> waypoints;
      
      // Simple fallback waypoint for each drone
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.header.stamp = this->get_clock()->now();
      waypoint.pose.orientation.w = 1.0;
      
      // OG waypoints, but kinda far
      if (drone_id_ == "rs1_drone_1") {
          waypoint.pose.position.x = 4.0;
          waypoint.pose.position.y = 16.0;
          waypoint.pose.position.z = 16.0;
      } else if (drone_id_ == "rs1_drone_2") {
          waypoint.pose.position.x = -5.0;
          waypoint.pose.position.y = 5.0;
          waypoint.pose.position.z = 15.0;
      } else if (drone_id_ == "rs1_drone_3") {
          waypoint.pose.position.x = -5.0;
          waypoint.pose.position.y = -5.0;
          waypoint.pose.position.z = 15.0;
      } else if (drone_id_ == "rs1_drone_4") {
          waypoint.pose.position.x = 5.0;
          waypoint.pose.position.y = -5.0;
          waypoint.pose.position.z = 8.0;
      } else {
          waypoint.pose.position.x = 6.0;
          waypoint.pose.position.y = -6.0;
          waypoint.pose.position.z = 15.0;
      }
      
      waypoints.push_back(waypoint);
      path_planner_->setWaypoints(waypoints);
      {
        std::lock_guard<std::mutex> lk(cache_mutex_);
        original_waypoints_cached_ = waypoints;
      }
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

  void MissionPlannerNode::takeoffDroneCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // Suppress unused parameter warning
    
    RCLCPP_INFO(this->get_logger(), "Takeoff service called for %s", drone_id_.c_str());
    
    // Check if we can takeoff (must be in IDLE state)
    if (state_machine_->getCurrentState() != MissionState::IDLE) {
      response->success = false;
      response->message = "Cannot takeoff - drone not in IDLE state. Current state: " + 
                        state_machine_->getStateString();
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    // Initialise takeoff state
    {
      std::lock_guard<std::mutex> lock(sonar_mutex_);
      takeoff_in_progress_ = true;
      takeoff_complete_ = false;
      takeoff_start_time_ = std::chrono::steady_clock::now();
    }
    
    // Publish takeoff command multiple times to ensure reception
    std_msgs::msg::Empty takeoff_msg;
    for (int i = 0; i < 5; i++) {
      takeoff_pub_->publish(takeoff_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Transition to takeoff state
    state_machine_->setState(MissionState::TAKEOFF);
    
    // Publish mission state
    std_msgs::msg::String state_msg;
    state_msg.data = state_machine_->getStateString();
    mission_state_pub_->publish(state_msg);
    
    response->success = true;
    response->message = "Takeoff initiated - drone will climb to 4m using sonar feedback";
    
    RCLCPP_INFO(this->get_logger(), "Takeoff initiated for %s - climbing to %.1fm", 
                drone_id_.c_str(), target_takeoff_altitude_);
  }

  void MissionPlannerNode::landDroneCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // Suppress unused parameter warning
    
    RCLCPP_INFO(this->get_logger(), "Landing service called for %s", drone_id_.c_str());
    
    // Check if we can land (must not be in IDLE or LANDING state already)
    MissionState current_state = state_machine_->getCurrentState();
    if (current_state == MissionState::IDLE) {
      response->success = false;
      response->message = "Cannot land - drone already in IDLE state";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    if (current_state == MissionState::LANDING) {
      response->success = false;
      response->message = "Cannot land - drone already in LANDING state";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    // Initialise landing state
    {
      std::lock_guard<std::mutex> lock(sonar_mutex_);
      landing_in_progress_ = true;
      landing_complete_ = false;
      landing_start_time_ = std::chrono::steady_clock::now();
    }
    
    // Publish landing command multiple times to ensure reception
    std_msgs::msg::Empty land_msg;
    for (int i = 0; i < 5; i++) {
      land_pub_->publish(land_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Transition to landing state
    state_machine_->setState(MissionState::LANDING);
    
    // Publish mission state
    std_msgs::msg::String state_msg;
    state_msg.data = state_machine_->getStateString();
    mission_state_pub_->publish(state_msg);
    
    response->success = true;
    response->message = "Landing initiated - drone will descend to 0.2m using sonar feedback";
    
    RCLCPP_INFO(this->get_logger(), "Landing initiated for %s - descending to %.1fm", 
                drone_id_.c_str(), target_landing_altitude_);
  }

  void MissionPlannerNode::wildfireReactionCallback(
      const std::shared_ptr<rs1_robot::srv::ReactToWildfire::Request> request,
      std::shared_ptr<rs1_robot::srv::ReactToWildfire::Response> response) {
    
    RCLCPP_INFO(this->get_logger(), "Wildfire reaction service called for incident: %s", 
                request->incident_id.c_str());
    
    // Save current mission state before starting reaction
    mission_executor_->saveMissionState(path_planner_.get());
    RCLCPP_INFO(this->get_logger(), "Saved current mission state for later restoration");
    
    // Build fire and depot locations from request
    geometry_msgs::msg::Point fire_location;
    fire_location.x = request->fire_x;
    fire_location.y = request->fire_y;
    fire_location.z = request->fire_z;
    
    geometry_msgs::msg::Point depot_location;
    depot_location.x = request->depot_x;
    depot_location.y = request->depot_y;
    depot_location.z = request->depot_z;
    
    // Execute wildfire reaction mission (sets waypoints, doesn't block)
    bool success = mission_executor_->executeWildfireReaction(
        fire_location, depot_location, helipad_location_, path_planner_.get());
    
    if (success) {
      // Mark that we're in a scenario reaction mission
      in_scenario_reaction_ = true;
      active_scenario_type_ = "WILDFIRE";
      active_scenario_incident_id_ = request->incident_id;  // Store incident ID for resolution
      
      // Track if drone was IDLE before mission (so we can return to IDLE after)
      was_idle_before_reaction_ = (state_machine_->getCurrentState() == MissionState::IDLE);
      
      // If drone is IDLE, need to initiate takeoff first
      if (state_machine_->getCurrentState() == MissionState::IDLE) {
        // Initiate takeoff
        {
          std::lock_guard<std::mutex> lock(sonar_mutex_);
          takeoff_in_progress_ = true;
          takeoff_complete_ = false;
          takeoff_start_time_ = std::chrono::steady_clock::now();
        }
        
        // Publish takeoff command (non-blocking for composed nodes)
        std_msgs::msg::Empty takeoff_msg;
        takeoff_pub_->publish(takeoff_msg);
        
        state_machine_->setState(MissionState::TAKEOFF);
        RCLCPP_INFO(this->get_logger(), "Initiated takeoff for wildfire response");
      } else if (state_machine_->getCurrentState() != MissionState::RESPONSE_NAVIGATION) {
        // Already airborne, just transition to response navigation
        state_machine_->setState(MissionState::RESPONSE_NAVIGATION);
        RCLCPP_INFO(this->get_logger(), "Transitioned to RESPONSE_NAVIGATION for wildfire response");
      }
      
      // Service returns immediately - actual mission execution happens asynchronously
      // Mission state will be restored when the waypoints are completed
      response->success = true;
      response->message = "Wildfire reaction mission started";
      response->completion_time = 0.0;  // Mission is async, not yet complete
      
      RCLCPP_INFO(this->get_logger(), "Wildfire reaction mission started - executing asynchronously");
    } else {
      // Failed to set up mission, restore immediately
      mission_executor_->restoreMissionState(path_planner_.get());
      
      response->success = false;
      response->message = "Failed to start wildfire reaction mission";
      response->completion_time = 0.0;
      
      RCLCPP_ERROR(this->get_logger(), "Wildfire reaction failed to start");
    }
  }

  void MissionPlannerNode::hikerRescueCallback(
      const std::shared_ptr<rs1_robot::srv::ReactToHiker::Request> request,
      std::shared_ptr<rs1_robot::srv::ReactToHiker::Response> response) {
    
    RCLCPP_INFO(this->get_logger(), "Hiker rescue service called for incident: %s",
                request->incident_id.c_str());
    
    // Save current mission state before starting reaction
    mission_executor_->saveMissionState(path_planner_.get());
    RCLCPP_INFO(this->get_logger(), "Saved current mission state for later restoration");
    
    // Build hiker and depot locations from request
    geometry_msgs::msg::Point hiker_location;
    hiker_location.x = request->hiker_x;
    hiker_location.y = request->hiker_y;
    hiker_location.z = request->hiker_z;
    
    geometry_msgs::msg::Point depot_location;
    depot_location.x = request->depot_x;
    depot_location.y = request->depot_y;
    depot_location.z = request->depot_z;
    
    // Execute hiker rescue mission (sets waypoints, doesn't block)
    bool success = mission_executor_->executeHikerRescue(
        hiker_location, depot_location, path_planner_.get());
    
    if (success) {
      // Mark that we're in a scenario reaction mission
      in_scenario_reaction_ = true;
      active_scenario_type_ = "STRANDED_HIKER";
      active_scenario_incident_id_ = request->incident_id;  // Store incident ID for resolution
      
      // Track if drone was IDLE before mission (so we can return to IDLE after)
      was_idle_before_reaction_ = (state_machine_->getCurrentState() == MissionState::IDLE);
      
      // If drone is IDLE, need to initiate takeoff first
      if (state_machine_->getCurrentState() == MissionState::IDLE) {
        // Initiate takeoff
        {
          std::lock_guard<std::mutex> lock(sonar_mutex_);
          takeoff_in_progress_ = true;
          takeoff_complete_ = false;
          takeoff_start_time_ = std::chrono::steady_clock::now();
        }
        
        // Publish takeoff command (non-blocking for composed nodes)
        std_msgs::msg::Empty takeoff_msg;
        takeoff_pub_->publish(takeoff_msg);
        
        state_machine_->setState(MissionState::TAKEOFF);
        RCLCPP_INFO(this->get_logger(), "Initiated takeoff for hiker rescue");
      } else if (state_machine_->getCurrentState() != MissionState::RESPONSE_NAVIGATION) {
        // Already airborne, just transition to response navigation
        state_machine_->setState(MissionState::RESPONSE_NAVIGATION);
        RCLCPP_INFO(this->get_logger(), "Transitioned to RESPONSE_NAVIGATION for hiker rescue");
      }
      
      // Service returns immediately - actual mission execution happens asynchronously
      // Mission state will be restored when the waypoints are completed
      response->success = true;
      response->message = "Hiker rescue mission started";
      response->completion_time = 0.0;  // Mission is async, not yet complete
      
      RCLCPP_INFO(this->get_logger(), "Hiker rescue mission started - executing asynchronously");
    } else {
      // Failed to set up mission, restore immediately
      mission_executor_->restoreMissionState(path_planner_.get());
      
      response->success = false;
      response->message = "Failed to start hiker rescue mission";
      response->completion_time = 0.0;
      
      RCLCPP_ERROR(this->get_logger(), "Hiker rescue failed to start");
    }
  }

  void MissionPlannerNode::debrisReactionCallback(
      const std::shared_ptr<rs1_robot::srv::ReactToDebris::Request> request,
      std::shared_ptr<rs1_robot::srv::ReactToDebris::Response> response) {
    
    RCLCPP_INFO(this->get_logger(), "Debris notification service called for incident: %s",
                request->incident_id.c_str());
    
    auto start_time = this->now();
    
    // Build debris location from request
    geometry_msgs::msg::Point debris_location;
    debris_location.x = request->debris_x;
    debris_location.y = request->debris_y;
    debris_location.z = request->debris_z;
    
    // Execute debris reaction (notification only)
    bool success = mission_executor_->executeDebrisReaction(debris_location);
    
    response->success = success;
    response->message = success ? "Debris notification acknowledged" : "Debris notification failed";
    response->completion_time = (this->now() - start_time).seconds();
    
    RCLCPP_INFO(this->get_logger(), "Debris notification processed in %.2f seconds",
                response->completion_time);
  }

  // Service-based coordination helper methods
  
  rclcpp::Client<rs1_robot::srv::ReactToWildfire>::SharedPtr 
  MissionPlannerNode::getOrCreateWildfireClient(int drone_id) {
    if (drone_id == drone_numeric_id_) {
      RCLCPP_DEBUG(this->get_logger(), "Using self wildfire client");
      return wildfire_client_;  // Use our own client
    }
    
    auto it = peer_wildfire_clients_.find(drone_id);
    if (it != peer_wildfire_clients_.end()) {
      RCLCPP_DEBUG(this->get_logger(), "Reusing existing wildfire client for drone %d", drone_id);
      return it->second;
    }
    
    // Create new client for this peer
    std::string peer_namespace = "rs1_drone_" + std::to_string(drone_id);
    std::string service_name = "/" + peer_namespace + "/react_to_wildfire";
    
    RCLCPP_INFO(this->get_logger(), 
                "Creating wildfire service client for drone %d: %s", 
                drone_id, service_name.c_str());
    
    try {
      auto client = this->create_client<rs1_robot::srv::ReactToWildfire>(service_name);
      peer_wildfire_clients_[drone_id] = client;
      RCLCPP_INFO(this->get_logger(), "Successfully created wildfire service client for drone %d", drone_id);
      return client;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Exception creating wildfire client for drone %d: %s", 
                   drone_id, e.what());
      throw;
    }
  }

  rclcpp::Client<rs1_robot::srv::ReactToHiker>::SharedPtr 
  MissionPlannerNode::getOrCreateHikerClient(int drone_id) {
    if (drone_id == drone_numeric_id_) {
      return hiker_client_;
    }
    
    auto it = peer_hiker_clients_.find(drone_id);
    if (it != peer_hiker_clients_.end()) {
      return it->second;
    }
    
    std::string peer_namespace = "rs1_drone_" + std::to_string(drone_id);
    auto client = this->create_client<rs1_robot::srv::ReactToHiker>(
      "/" + peer_namespace + "/react_to_hiker");
    peer_hiker_clients_[drone_id] = client;
    
    RCLCPP_INFO(this->get_logger(), "Created hiker rescue service client for drone %d", drone_id);
    return client;
  }

  rclcpp::Client<rs1_robot::srv::ReactToDebris>::SharedPtr 
  MissionPlannerNode::getOrCreateDebrisClient(int drone_id) {
    if (drone_id == drone_numeric_id_) {
      return debris_client_;
    }
    
    auto it = peer_debris_clients_.find(drone_id);
    if (it != peer_debris_clients_.end()) {
      return it->second;
    }
    
    std::string peer_namespace = "rs1_drone_" + std::to_string(drone_id);
    auto client = this->create_client<rs1_robot::srv::ReactToDebris>(
      "/" + peer_namespace + "/react_to_debris");
    peer_debris_clients_[drone_id] = client;
    
    RCLCPP_INFO(this->get_logger(), "Created debris notification service client for drone %d", drone_id);
    return client;
  }

  void MissionPlannerNode::callWildfireService(
      int responder_id, 
      const ScenarioData& scenario, 
      const std::string& incident_id) {
    
    RCLCPP_INFO(this->get_logger(), "Getting wildfire client for drone %d", responder_id);
    
    auto client = getOrCreateWildfireClient(responder_id);
    if (!client) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get wildfire client for drone %d", responder_id);
      return;
    }
    
    // Check if service is available (non-blocking check)
    if (!client->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), 
                   "Wildfire service not ready for drone %d, sending request anyway...", 
                   responder_id);
    }
    
    // Create service request
    auto request = std::make_shared<rs1_robot::srv::ReactToWildfire::Request>();
    request->fire_x = scenario.x;
    request->fire_y = scenario.y;
    request->fire_z = scenario.z;
    // Use helipad location as depot (fire retardant stored at helipad)
    request->depot_x = helipad_location_.x;
    request->depot_y = helipad_location_.y;
    request->depot_z = helipad_location_.z;
    request->severity = scenario.severity;
    request->incident_id = incident_id;
    
    RCLCPP_INFO(this->get_logger(), "Sending wildfire service request to drone %d", responder_id);
    
    // Send async request - fire and forget pattern (no callback needed for composed mode)
    // The responder drone will handle the mission and update the registry when complete
    try {
      client->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), 
                 "Successfully sent wildfire service request to drone %d for incident %s",
                 responder_id, incident_id.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                  "Exception sending wildfire service request: %s", e.what());
    }
  }

  void MissionPlannerNode::callHikerService(
      int responder_id,
      const ScenarioData& scenario,
      const std::string& incident_id) {
    
    auto client = getOrCreateHikerClient(responder_id);
    
    // Check if service is available (non-blocking check)
    if (!client->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(),
                   "Hiker rescue service not ready for drone %d, sending request anyway...", 
                   responder_id);
    }
    
    auto request = std::make_shared<rs1_robot::srv::ReactToHiker::Request>();
    request->hiker_x = scenario.x;
    request->hiker_y = scenario.y;
    request->hiker_z = scenario.z;
    request->depot_x = medkit_depot_xyz_.x;
    request->depot_y = medkit_depot_xyz_.y;
    request->depot_z = medkit_depot_xyz_.z;
    request->severity = scenario.severity;
    request->incident_id = incident_id;
    
    // Send async request - fire and forget pattern (no callback needed for composed mode)
    client->async_send_request(request);
    
    RCLCPP_INFO(this->get_logger(),
               "Called hiker rescue service on drone %d for incident %s",
               responder_id, incident_id.c_str());
  }

  void MissionPlannerNode::callDebrisService(
      int responder_id,
      const ScenarioData& scenario,
      const std::string& incident_id) {
    
    auto client = getOrCreateDebrisClient(responder_id);
    
    // Check if service is available (non-blocking check)
    if (!client->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(),
                   "Debris notification service not ready for drone %d, sending request anyway...", 
                   responder_id);
    }
    
    auto request = std::make_shared<rs1_robot::srv::ReactToDebris::Request>();
    request->debris_x = scenario.x;
    request->debris_y = scenario.y;
    request->debris_z = scenario.z;
    request->severity = scenario.severity;
    request->incident_id = incident_id;
    
    // Send async request - fire and forget pattern (no callback needed for composed mode)
    client->async_send_request(request);
    
    RCLCPP_INFO(this->get_logger(),
               "Called debris notification service on drone %d for incident %s",
               responder_id, incident_id.c_str());
  }

  // Mission execution logic
  void MissionPlannerNode::executeMission() {
    MissionState current_state = state_machine_->getCurrentState();
    
    switch (current_state) {
      case MissionState::IDLE:
        // IDLE state handled by drone_controller - no action needed here
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
      
      case MissionState::RESPONSE_NAVIGATION:
        // Scenario response navigation - uses same waypoint navigation logic
        // but prevents loading default waypoints from YAML
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
      
      // Scenario reactions now handled via services, not states
    }
  }

  void MissionPlannerNode::takeoff() {
    // Mission planner delegates actual flight execution to drone controller
    // We only monitor completion via time-based approach (not altitude)
    // This prevents issues with invalid sonar readings during ground drooping
    
    if (!takeoff_in_progress_) {
      return; // Takeoff not initiated via service
    }
    
    // Use time-based takeoff completion
    // Sonar readings can be unreliable (inf, >5m) when drone droops on ground
    // Give drone enough time to reach target altitude, then transition regardless
    const double TAKEOFF_DURATION_SECONDS = 3.0; // Allow 3 seconds for 8m climb at 5m/s
    
    auto elapsed = std::chrono::steady_clock::now() - takeoff_start_time_;
    double elapsed_seconds = std::chrono::duration<double>(elapsed).count();
    
    // After takeoff duration, transition to navigation
    // Drone will hover at whatever altitude it reached (ideally ~8m)
    if (elapsed_seconds >= TAKEOFF_DURATION_SECONDS) {
      takeoff_complete_ = true;
      takeoff_in_progress_ = false;
      
      // Transition to next state when takeoff complete
      if (path_planner_->hasNextWaypoint()) {
        // If responding to a scenario, go to RESPONSE_NAVIGATION
        // Otherwise, go to normal WAYPOINT_NAVIGATION
        if (in_scenario_reaction_) {
          state_machine_->setState(MissionState::RESPONSE_NAVIGATION);
          RCLCPP_INFO(this->get_logger(), "Takeoff complete (%.1fs) - transitioning to RESPONSE_NAVIGATION for %s", 
                     elapsed_seconds, active_scenario_type_.c_str());
        } else {
          state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
          RCLCPP_INFO(this->get_logger(), "Takeoff complete (%.1fs) - transitioning to waypoint navigation", 
                     elapsed_seconds);
        }
      } else {
        state_machine_->setState(MissionState::HOVERING);
        RCLCPP_INFO(this->get_logger(), "Takeoff complete (%.1fs) - no waypoints available, transitioning to HOVERING",
                   elapsed_seconds);
      }
    }
  }

/***************** STUFF BELOW NOT JACKSON *****************/



  void MissionPlannerNode::hovering() {
    // Mission planner monitors hovering state
    // Actual hovering control is handled by drone_controller
    // We just maintain state here - drone_controller will keep position
  }

  void MissionPlannerNode::manualControl() {
    RCLCPP_DEBUG(this->get_logger(), "In manual control mode");
  }

  void MissionPlannerNode::emergency() {
    RCLCPP_WARN(this->get_logger(), "Emergency state active - drone_controller executing emergency descent");
    // Mission planner just monitors - actual emergency descent is handled by drone_controller
    // Transition to landing state for continued emergency descent
    state_machine_->setState(MissionState::LANDING);
  }


  /*********************** IS JACKSONS   ***************** */
  void MissionPlannerNode::publishMissionCommand() {
    // Publish waypoints for both normal navigation and scenario response missions
    if ((state_machine_->getCurrentState() == MissionState::WAYPOINT_NAVIGATION ||
         state_machine_->getCurrentState() == MissionState::RESPONSE_NAVIGATION) && 
        path_planner_->hasNextWaypoint()) {
      
      // Get current waypoint and publish as target pose for drone_controller
      geometry_msgs::msg::PoseStamped current_waypoint = path_planner_->getCurrentWaypoint();
      current_waypoint.header.stamp = this->get_clock()->now();
      current_waypoint.header.frame_id = "map";
      
      // Publish target pose - drone_controller will handle the actual flight control
      target_pose_pub_->publish(current_waypoint);
      
      RCLCPP_DEBUG(this->get_logger(), "Published target waypoint: [%.2f, %.2f, %.2f] to drone_controller",
                  current_waypoint.pose.position.x,
                  current_waypoint.pose.position.y,
                  current_waypoint.pose.position.z);
    } else {
        // Mission planner doesn't publish velocity commands - that's drone_controller's job
        // We only manage state transitions here
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

  void MissionPlannerNode::sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(sonar_mutex_);
    current_sonar_range_ = msg->range;
    
    // Debug output during takeoff
    if (takeoff_in_progress_) {
      RCLCPP_DEBUG(this->get_logger(), "Sonar reading: %.2fm (target: %.2fm)", 
                   current_sonar_range_, target_takeoff_altitude_);
    }
  }

  void MissionPlannerNode::resetMissioncallback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string payload = trimCopy(msg->data);
    auto tokens = splitCSV(payload); 
    if (tokens.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Invalid reset command: '%s'", payload.c_str());
      return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> route_waypoints_cached;
    std::vector<geometry_msgs::msg::PoseStamped> original_waypoints_cached;

    {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      route_waypoints_cached = route_waypoints_cached_;
      original_waypoints_cached = original_waypoints_cached_;
    }

    const std::string& mode = tokens[1];
    if (mode == "ORIGINAL_MISSION") {
      if (original_waypoints_cached.empty()) {
        RCLCPP_WARN(this->get_logger(), "No original mission cached; ignoring reset.");
        return;
      }
      path_planner_->setWaypoints(original_waypoints_cached);
      RCLCPP_INFO(this->get_logger(), "Reset to ORIGINAL mission waypoints.");
      if (canStateTransitionTo(state_machine_->getCurrentState(), MissionState::WAYPOINT_NAVIGATION)) {
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      }
    }
    else if (mode == "ROUTE_MISSION") {
      if (route_waypoints_cached.empty()) {
        RCLCPP_WARN(this->get_logger(), "No route mission cached; ignoring reset.");
        return;
      }
      path_planner_->setWaypoints(route_waypoints_cached);
      RCLCPP_INFO(this->get_logger(), "Reset to ROUTE mission waypoints.");
      if (canStateTransitionTo(state_machine_->getCurrentState(), MissionState::WAYPOINT_NAVIGATION)) {
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Unknown reset mission mode: '%s'", mode.c_str());
    }
  }


  /**************************** MATTHEW AND MARCUS *****************************/
  inline float normalizeAngle(float a) {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;
    while (a >= PI)  a -= TWO_PI;
    while (a < -PI)  a += TWO_PI;
    return a;
  }

  /**
   * @brief Generate circular orbit waypoints around a point of interest
   * 
   * Creates a circular trajectory of waypoints that orbit around a central point,
   * with each waypoint oriented to face the centre (useful for inspection/monitoring).
   * 
   * @param center_x X coordinate of orbit centre
   * @param center_y Y coordinate of orbit centre  
   * @param center_z Z coordinate (altitude) for all waypoints
   * @param radius Orbit radius in metres
   * @param point_count Number of waypoints to generate around the circle
   * @return Vector of PoseStamped waypoints forming a circular orbit
   */
  std::vector<geometry_msgs::msg::PoseStamped> generateOrbitWaypoints(
      double center_x, double center_y, double center_z,
      double radius, int point_count)
  {
    const int N = std::max(1, point_count);
    const double R = std::fabs(radius);

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    waypoints.reserve(N);

    // Edge case: zero radius means hovering at the centre point
    if (R < 0.01)  // Small epsilon for floating point comparison
    {
      geometry_msgs::msg::PoseStamped hover_point;
      hover_point.header.frame_id = "map";
      hover_point.pose.position.x = center_x;
      hover_point.pose.position.y = center_y;
      hover_point.pose.position.z = center_z;
      hover_point.pose.orientation.w = 1.0;  // Identity quaternion
      
      for (int i = 0; i < N; ++i) {
        waypoints.push_back(hover_point);
      }
      return waypoints;
    }

    constexpr double PI = 3.14159265358979323846;
    const double angle_step = 2.0 * PI / static_cast<double>(N);

    for (int i = 0; i < N; ++i)
    {
      const double theta = angle_step * static_cast<double>(i);

      // Calculate position on circle
      const double px = center_x + R * std::cos(theta);
      const double py = center_y + R * std::sin(theta);

      // Calculate yaw to face the centre point
      double yaw = std::atan2(center_y - py, center_x - px);
      
      // Convert yaw to quaternion (rotation around Z axis)
      // q = [cos(yaw/2), 0, 0, sin(yaw/2)]
      const double half_yaw = yaw * 0.5;
      const double qw = std::cos(half_yaw);
      const double qz = std::sin(half_yaw);

      // Create waypoint
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.pose.position.x = px;
      waypoint.pose.position.y = py;
      waypoint.pose.position.z = center_z;
      waypoint.pose.orientation.w = qw;
      waypoint.pose.orientation.x = 0.0;
      waypoint.pose.orientation.y = 0.0;
      waypoint.pose.orientation.z = qz;

      waypoints.push_back(waypoint);
    }

    return waypoints;
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

    RCLCPP_INFO(this->get_logger(), "Waypoint stored. Awaiting /start_mission or manual state change.");
  }

  //--- tiny string/parse helpers (local-only) ---//
  // Parses a key:value token, trims both sides, returns false if no colon or empty key.
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
    // Map state strings to enum values (simplified - scenario reactions removed)
    if (s == "IDLE") return MissionState::IDLE;
    if (s == "TAKEOFF") return MissionState::TAKEOFF;
    if (s == "WAYPOINT_NAVIGATION") return MissionState::WAYPOINT_NAVIGATION;
    if (s == "RESPONSE_NAVIGATION") return MissionState::RESPONSE_NAVIGATION;
    if (s == "HOVERING") return MissionState::HOVERING;
    if (s == "LANDING") return MissionState::LANDING;
    if (s == "MANUAL_CONTROL") return MissionState::MANUAL_CONTROL;
    if (s == "EMERGENCY") return MissionState::EMERGENCY;
    // Scenario reaction states removed - now handled via services
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

  static std::string missionStateToString(MissionState state) {
      switch (state) {
          case MissionState::IDLE: return "IDLE";
          case MissionState::TAKEOFF: return "TAKEOFF";
          case MissionState::WAYPOINT_NAVIGATION: return "WAYPOINT_NAVIGATION";
          case MissionState::RESPONSE_NAVIGATION: return "RESPONSE_NAVIGATION";
          case MissionState::HOVERING: return "HOVERING";
          case MissionState::LANDING: return "LANDING";
          case MissionState::MANUAL_CONTROL: return "MANUAL_CONTROL";
          case MissionState::EMERGENCY: return "EMERGENCY";
          // Scenario reaction states removed - now handled via services
          default: return "UNKNOWN";
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
    
    // Check we have exactly 6 parts (updated from 7)
    if (parts.size() != 6) {
      RCLCPP_WARN(rclcpp::get_logger("scenario_parser"),
                  "Expected 6 fields, got %zu in message: '%s'",
                  parts.size(), message_data.c_str());
      return result;
    }
    
    // Parse each field with error checking
    try {
      // Field 0: Scenario name (string)
      result.scenario_name = parts[0];
      
      // Field 1: X position (double) - MOVED FROM INDEX 2
      result.x = std::stod(parts[1]);
      
      // Field 2: Y position (double) - MOVED FROM INDEX 3
      result.y = std::stod(parts[2]);
      
      // Field 3: Z position (double) - MOVED FROM INDEX 4
      result.z = std::stod(parts[3]);
      
      // Field 4: Yaw angle (double, in radians) - MOVED FROM INDEX 5
      result.yaw = std::stod(parts[4]);
      
      // Field 5: Respond flag - MOVED FROM INDEX 6
      std::string respond_field = parts[5];
      if (respond_field.find("respond:") == 0) {
        std::string value = respond_field.substr(8);  // Skip "respond:"
        result.can_respond = (value == "1" || value == "true");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("scenario_parser"),
                    "Invalid respond field: '%s'", respond_field.c_str());
        return result;
      }
      
      // Set severity to a default value since it's not provided
      result.severity = 1;  // Default severity
      
      // If we made it here, parsing succeeded
      result.valid = true;
      
      RCLCPP_INFO(rclcpp::get_logger("scenario_parser"),
                  "Parsed scenario: %s at [%.2f, %.2f, %.2f], yaw=%.2f, respond=%s",
                  result.scenario_name.c_str(), result.x, result.y, result.z,
                  result.yaw, result.can_respond ? "yes" : "no");
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("scenario_parser"),
                  "Failed to parse scenario message: %s", e.what());
      result.valid = false;
    }
    
    return result;
  }

  //--- mapper ---//
  Scenario MissionPlannerNode::scenarioFromString(const std::string& s) {
    // Exact matches published by perception
    if (s == "STRANDED_HIKER")     return Scenario::STRANDED_HIKER;
    if (s == "WILDFIRE")           return Scenario::WILDFIRE;
    if (s == "DEBRIS_OBSTRUCTION") return Scenario::DEBRIS_OBSTRUCTION;

    return Scenario::UNKNOWN;
  }

  MissionState MissionPlannerNode::targetStateForScenario(Scenario scenario) {
    // Scenarios are now handled via services, not states
    // This function returns the required state for a drone to be eligible to accept the service
    switch (scenario) {
      case Scenario::STRANDED_HIKER:
      case Scenario::WILDFIRE:
      case Scenario::DEBRIS_OBSTRUCTION:
        // Drones in WAYPOINT_NAVIGATION or HOVERING can accept scenario service calls
        // Return WAYPOINT_NAVIGATION as the target "active mission" state
        return MissionState::WAYPOINT_NAVIGATION;
      case Scenario::UNKNOWN:
      default:
        // Unknown scenarios: remain idle
        return MissionState::IDLE;
    }
  }

  void MissionPlannerNode::scenarioDetectionCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Scenario detection callback received message: %s", msg->data.c_str());
    
    // Parse the incoming message
    ScenarioData scenario = parseScenarioMessage(msg->data);
    if (!scenario.valid) {
      RCLCPP_WARN(this->get_logger(), "Received invalid scenario message: %s", msg->data.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Parsed scenario: %s at [%.2f, %.2f, %.2f]", 
                scenario.scenario_name.c_str(), scenario.x, scenario.y, scenario.z);

    // Check if this incident was recently resolved - prevents re-detection after resuming patrol
    {
      std::lock_guard<std::mutex> lock(resolved_incidents_mutex_);
      auto now = this->get_clock()->now();
      
      // Clean up expired resolved incidents
      recently_resolved_incidents_.erase(
        std::remove_if(recently_resolved_incidents_.begin(), 
                      recently_resolved_incidents_.end(),
                      [&](const ResolvedIncident& resolved) {
                        return (now - resolved.resolved_at) > resolved_incident_ignore_duration_;
                      }),
        recently_resolved_incidents_.end()
      );
      
      // Check if current detection matches a recently resolved incident
      for (const auto& resolved : recently_resolved_incidents_) {
        double dx = scenario.x - resolved.location.x;
        double dy = scenario.y - resolved.location.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // Check if same type and within spatial threshold
        if (scenario.scenario_name == resolved.scenario_name && 
            distance < resolved_incident_match_radius_) {
          double time_since_resolved = (now - resolved.resolved_at).seconds();
          RCLCPP_INFO(this->get_logger(),
                     "Ignoring %s detection - recently resolved %.1fs ago (%.2fm away). "
                     "Prevents re-detection after resuming patrol.",
                     scenario.scenario_name.c_str(), time_since_resolved, distance);
          return;
        }
      }
    }

    // Don't interrupt drones executing scenario missions (check BEFORE registry to avoid time comparison issues)
    if (in_scenario_reaction_) {
      RCLCPP_INFO(this->get_logger(),
                  "Ignoring scenario detection - currently executing %s mission (incident: %s)",
                  active_scenario_type_.c_str(), active_scenario_incident_id_.c_str());
      return;
    }

    // Check fleet-wide registry
    if (isIncidentAlreadyManaged(scenario)) {
      RCLCPP_INFO(this->get_logger(), 
                  "Ignoring %s - already managed by another drone",
                  scenario.scenario_name.c_str());
      return;
    }

    if (shouldSuppressIncident(scenario)) {
      RCLCPP_INFO(this->get_logger(), 
                  "Suppressing %s due to cooldown",
                  scenario.scenario_name.c_str());
      return;  // no hover/ping/selection
    }

    if (isBusyWithAssignedMission()) {
      RCLCPP_INFO(this->get_logger(),
                  "Ignoring scenario while executing assigned mission "
                  "(in_hiker_rescue=%d)",
                  in_hiker_rescue_);
      return;
    }

    // Smart tie-breaker: Only delay if potential simultaneous detection by lower-ID drone
    // Check if this could be a race condition (multiple drones detecting at nearly same time)
    // Strategy: Only apply delay if:
    //   1. Multiple active patrollers exist
    //   2. This drone is NOT the lowest ID
    //   3. No other drone has claimed this incident yet (registry check already passed)
    // 
    // This allows "first detector wins" for non-simultaneous detections while preventing
    // race conditions when multiple drones detect the same incident within ~50-100ms
    
    auto all_known_drones = getKnownDroneIds();
    int num_active_patrollers = 0;
    int lowest_patroller_id = std::numeric_limits<int>::max();
    
    // Count actively patrolling drones (WAYPOINT_NAVIGATION only - not IDLE)
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      for (const auto& [peer_id, peer_info] : peer_info_) {
        if (peer_info.state == MissionState::WAYPOINT_NAVIGATION) {
          num_active_patrollers++;
          lowest_patroller_id = std::min(lowest_patroller_id, peer_id);
        }
      }
    }
    
    // Include self if we're actively patrolling
    auto my_state = state_machine_->getCurrentState();
    if (my_state == MissionState::WAYPOINT_NAVIGATION) {
      num_active_patrollers++;
      lowest_patroller_id = std::min(lowest_patroller_id, drone_numeric_id_);
    }
    
    RCLCPP_INFO(this->get_logger(),
                "Active patrollers: %d, Lowest patroller ID: %d, My ID: %d, My state: %s",
                num_active_patrollers, lowest_patroller_id, drone_numeric_id_,
                state_machine_->getStateString().c_str());
    
    // Apply tie-breaker ONLY if:
    // 1. Multiple patrollers exist (potential for simultaneous detection)
    // 2. This drone has higher ID than lowest patroller
    // 3. Small delay to let lower-ID drone broadcast if it detected simultaneously
    // 
    // Key: The delay is SHORT (50ms) - just enough for broadcast propagation
    // If lower-ID drone detected it, we'll see the broadcast during the delay
    // If lower-ID drone hasn't detected it yet, this drone becomes "first detector"
    if (num_active_patrollers > 1 && drone_numeric_id_ > lowest_patroller_id) {
      // Use minimal delay (50ms per ID difference) to allow broadcast propagation
      int delay_ms = (drone_numeric_id_ - lowest_patroller_id) * 50;
      RCLCPP_INFO(this->get_logger(),
                  "Potential simultaneous detection: Delaying %dms to check if lower-ID drone detected first",
                  delay_ms);
        
      // Use non-blocking one-shot timer instead of sleep (safe for composed nodes)
      // Store timer as member to keep it alive until it fires
      tiebreaker_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(delay_ms),
        [this, scenario, msg]() {
          // Cancel timer after single execution (one-shot behavior)
          if (tiebreaker_timer_) {
            tiebreaker_timer_->cancel();
            tiebreaker_timer_.reset();
          }
          
          RCLCPP_INFO(this->get_logger(),
                      "Tie-breaker timer fired for drone %d after %dms delay",
                      drone_numeric_id_, (drone_numeric_id_ - 1) * 50);
          
          // Recheck registry after delay - lower ID drone may have claimed it
          if (isIncidentAlreadyManaged(scenario)) {
            RCLCPP_INFO(this->get_logger(),
                        "After delay: %s now managed by lower-ID drone",
                        scenario.scenario_name.c_str());
            return;
          }
          
          RCLCPP_INFO(this->get_logger(),
                      "After delay: Registry still empty - proceeding as first detector");
          
          // Recheck other conditions
          if (in_scenario_reaction_) {
            RCLCPP_INFO(this->get_logger(),
                        "After delay: Already in scenario reaction, ignoring");
            return;
          }
          
          if (isBusyWithAssignedMission()) {
            RCLCPP_INFO(this->get_logger(),
                        "After delay: Busy with assigned mission, ignoring");
            return;
          }
          
          // Set coordination flag
          {
            std::lock_guard<std::mutex> lock(coordination_mutex_);
            if (is_coordinating_) {
              RCLCPP_INFO(this->get_logger(), "After delay: Already coordinating, ignoring");
              return;
            }
            is_coordinating_ = true;
            active_coordination_scenario_ = scenario;
          }
          
          // Set cooldown
          {
            std::lock_guard<std::mutex> lk(dispatch_mutex_);
            DispatchCooldown cd;
            cd.until = steady_clock_.now() + coordination_cooldown_;
            cd.responder_id = -1;
            cd.target.x = scenario.x;
            cd.target.y = scenario.y;
            cd.target.z = scenario.z;
            dispatch_cooldown_[scenario.scenario_name] = cd;
          }
          
          // Notify GUI
          alertIncidentGui(this->parseScenarioDetection(*msg));
          
          RCLCPP_INFO(this->get_logger(),
                      "Drone %d detected: %s at [%.2f, %.2f, %.2f]", drone_numeric_id_,
                      scenario.scenario_name.c_str(), scenario.x, scenario.y, scenario.z);
          
          RCLCPP_INFO(this->get_logger(), "Coordinating response for %s", scenario.scenario_name.c_str());
          
          // Perform coordination
          try {
            if (scenario.scenario_name == "STRANDED_HIKER" ||
                scenario.scenario_name == "WILDFIRE") {
              performCoordination(scenario);
            } else if (scenario.scenario_name == "DEBRIS_OBSTRUCTION") {
              RCLCPP_INFO(this->get_logger(), "Debris detected - notifying GUI only");
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Coordination exception: %s", e.what());
          }
          
          // Reset coordination flags
          {
            std::lock_guard<std::mutex> lock(coordination_mutex_);
            is_coordinating_ = false;
            active_coordination_scenario_.reset();
            RCLCPP_INFO(this->get_logger(), "Coordination complete. Ready for new scenarios.");
          }
        }
      );
      
      // Timer will fire once and then be destroyed automatically
      return;
    }    {
      std::lock_guard<std::mutex> lock(coordination_mutex_);
      if (is_coordinating_ && 
          active_coordination_scenario_.has_value() && 
          active_coordination_scenario_->scenario_name == scenario.scenario_name) {
          RCLCPP_INFO(this->get_logger(), 
                      "Already coordinating %s - ignoring duplicate detection",
                      scenario.scenario_name.c_str());
          return;
      }
      
      // Set coordination flag
      is_coordinating_ = true;  
      active_coordination_scenario_ = scenario;
    }
    
    // Set dispatch cooldown IMMEDIATELY to prevent duplicate processing
    // This prevents multiple detections from racing through before the background thread starts
    {
      std::lock_guard<std::mutex> lk(dispatch_mutex_);
      DispatchCooldown cd; 
      cd.until        = steady_clock_.now() + coordination_cooldown_;
      cd.responder_id = -1;  // Will be updated after responder selection
      cd.target.x     = scenario.x;
      cd.target.y     = scenario.y;
      cd.target.z     = scenario.z;
      dispatch_cooldown_[scenario.scenario_name] = cd;
      RCLCPP_DEBUG(this->get_logger(), 
                  "Set immediate cooldown for %s to prevent duplicate processing",
                  scenario.scenario_name.c_str());
    }
    
    // NOTIFY GUI
    alertIncidentGui(this->parseScenarioDetection(*msg));

    auto all_drones = getKnownDroneIds();
    
    // Single drone can self-respond if in IDLE or WAYPOINT_NAVIGATION state
    // Multi-drone coordination requires at least 2 drones
    if (all_drones.size() == 1) {
      RCLCPP_INFO(this->get_logger(), 
                  "Single drone scenario - will attempt self-response if available");
      // Continue to coordination logic to allow self-assignment
    }

    RCLCPP_INFO(this->get_logger(),
                "Drone %d detected: %s at [%.2f, %.2f, %.2f]", drone_numeric_id_,
                scenario.scenario_name.c_str(), scenario.x, scenario.y, scenario.z);

    RCLCPP_INFO(this->get_logger(), "Coordinating response for %s", scenario.scenario_name.c_str());

    // Call coordination directly (already fully async via service calls)
    // No need for detached thread which causes crashes in composed nodes
    try {
      if (scenario.scenario_name == "STRANDED_HIKER" ||
          scenario.scenario_name == "WILDFIRE") {
        performCoordination(scenario);
      } else if (scenario.scenario_name == "DEBRIS_OBSTRUCTION") {
        RCLCPP_INFO(this->get_logger(), "Debris detected - notifying GUI only");
      }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Coordination exception: %s", e.what());
    }

    // Reset coordination flags after coordination completes
    {
      std::lock_guard<std::mutex> lock(coordination_mutex_);
      is_coordinating_ = false;
      active_coordination_scenario_.reset();
      RCLCPP_INFO(this->get_logger(), "Coordination complete. Ready for new scenarios.");
    }
  }

  std::optional<ScenarioEvent> MissionPlannerNode::parseScenarioDetection(const std_msgs::msg::String& msg) {
    // Expected: "SCENARIO_NAME,x,y,z,heading,respond:1"
    const std::string data = trimCopy(msg.data);
    auto tokens = splitCSV(data);
    for (auto& t : tokens) t = trimCopy(t);

    // Shape check
    if (tokens.size() < 6) {
      RCLCPP_WARN(this->get_logger(),
                  "Scenario parse failed (fields=%zu < 6): '%s'", tokens.size(), data.c_str());
      return std::nullopt;
    }

    // 1) Scenario type
    const std::string scenario_name = tokens[0];
    Scenario scenario = scenarioFromString(scenario_name);
    if (scenario == Scenario::UNKNOWN) {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown scenario name '%s' in message '%s'", scenario_name.c_str(), data.c_str());
      return std::nullopt;
    }

    // 2–4) Target position x,y,z
    double x{}, y{}, z{};
    if (!parseDouble(tokens[1], x) ||
        !parseDouble(tokens[2], y) ||
        !parseDouble(tokens[3], z)) {
      RCLCPP_WARN(this->get_logger(), "Scenario position parse failed in '%s'", data.c_str());
      return std::nullopt;
    }

    // 5) Heading (radians)
    double heading{};
    if (!parseDouble(tokens[4], heading)) {
      RCLCPP_WARN(this->get_logger(), "Scenario heading parse failed in '%s'", data.c_str());
      return std::nullopt;
    }

    // 6) respond flag
    bool can_respond = false;
    if (!parseRespondFlag(tokens[5], can_respond)) {
      RCLCPP_WARN(this->get_logger(), "Scenario respond flag parse failed in '%s'", data.c_str());
      return std::nullopt;
    }

    ScenarioEvent ev;
    ev.type = scenario;
    ev.target.x = x; ev.target.y = y; ev.target.z = z;
    ev.heading = heading;            // radians, as published
    ev.can_respond = can_respond;
    ev.stamp = this->now();          // when we received it
    ev.raw = data;

    return ev;
  }

  bool MissionPlannerNode::waitForPeerPingSubscriber(int peer_id, std::chrono::milliseconds timeout)
  {
    auto deadline = this->now() + rclcpp::Duration(timeout);
    while (this->now() < deadline) {
      std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> pub;
      {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        auto it = info_request_pubs_.find(peer_id);
        if (it != info_request_pubs_.end()) pub = it->second;
      }
      if (pub && pub->get_subscription_count() > 0) {
        return true; // the peer’s /info_request sub is matched
      }
      rclcpp::sleep_for(std::chrono::milliseconds(25));
    }
    return false;
  }
  
  DroneInfo MissionPlannerNode::parseInfoManifest(const std::string& manifest_data) {
    DroneInfo result;
    result.valid = false;
    result.battery_level = 0.0;
    result.drone_id = -1;
    
    auto toks = splitCSV(manifest_data);
    
    try {
      for (auto& t : toks) { // for token in tokens
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
      const std::vector<int>& all_drone_ids,
      MissionState required_state,
      const geometry_msgs::msg::Point& incident_xyz) {

    RCLCPP_INFO(this->get_logger(),
                "Selecting best IDLE responder for scenario (required state: %s)",
                missionStateToString(required_state).c_str());

    auto drone_data = pingDronesForInfo(all_drone_ids);

    struct Candidate { int id; double distance; double battery; std::string state; };
    std::vector<Candidate> candidates;

    for (const auto& [drone_id, info] : drone_data) {
      if (!info.valid) continue;
      
      // Only consider drones with sufficient battery
      if (info.battery_level < 0.5) {
        RCLCPP_DEBUG(this->get_logger(),
                    "Drone %d: REJECTED - low battery (%.0f%%)",
                    drone_id, info.battery_level * 100.0);
        continue;
      }

      MissionState current_state = stateFromString(info.mission_state);
      
      // SIMPLIFIED: Only consider IDLE drones as available responders
      if (current_state != MissionState::IDLE) {
        RCLCPP_DEBUG(this->get_logger(),
                    "Drone %d: REJECTED - not IDLE (state: %s)",
                    drone_id, info.mission_state.c_str());
        continue;
      }

      double dx = info.x - incident_xyz.x;
      double dy = info.y - incident_xyz.y;
      double dz = info.z - incident_xyz.z;
      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

      candidates.push_back({drone_id, distance, info.battery_level, info.mission_state});
      RCLCPP_INFO(this->get_logger(),
                  "Drone %d: CANDIDATE - IDLE, battery=%.0f%%, dist_to_incident=%.2fm",
                  drone_id, info.battery_level * 100.0, distance);
    }

    if (candidates.empty()) {
      RCLCPP_WARN(this->get_logger(), "No suitable IDLE peer drones found!");
      return -1;
    }

    // Sort by distance (closest drone responds)
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b){ return a.distance < b.distance; });

    const auto& best = candidates.front();
    RCLCPP_INFO(this->get_logger(),
                "Selected drone %d: IDLE, battery=%.0f%%, dist_to_incident=%.2fm",
                best.id, best.battery * 100.0, best.distance);
    return best.id;
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
      // Fallback: use current time - 10 seconds instead of epoch (1970)
      if (!buf[0]) {
        auto fallback_time = this->now() - rclcpp::Duration::from_seconds(10.0);
        auto fallback_ns = nanoseconds(fallback_time.nanoseconds());
        auto fallback_tp = time_point<std::chrono::system_clock>(duration_cast<std::chrono::system_clock::duration>(fallback_ns));
        std::time_t fallback_tt = std::chrono::system_clock::to_time_t(fallback_tp);
        gmtime_r(&fallback_tt, &g);
        std::strftime(buf, sizeof(buf), "%FT%TZ", &g);
      }
      return std::string(buf[0] ? buf : "ERROR-TIMESTAMP");
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

  void MissionPlannerNode::infoManifestCallback(int peer_id, const std_msgs::msg::String::SharedPtr& msg) {
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
    
    // If you keep the guard, at least log mismatches.
    
    if (id_from_msg > 0 && id_from_msg != peer_id) {
      RCLCPP_WARN(this->get_logger(), "Manifest id mismatch: msg_id=%d sub_peer=%d", id_from_msg, peer_id);
    }

    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      peer_info_[peer_id] = pi;  // <— don’t block on id match for now
    }
  }

}  // namespace drone_swarm
