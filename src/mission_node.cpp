#include "mission_node.h"


namespace drone_swarm
{

  MissionPlannerNode::MissionPlannerNode(const rclcpp::NodeOptions& options)
    : Node("mission_planner", options)
  {
    // Initialise ROS parameters for mission configuration
    this->declare_parameter("drone_namespace", std::string("rs1_drone"));
    this->declare_parameter<double>("mission_update_rate", 5.0);
    this->declare_parameter<double>("waypoint_tolerance", 0.5);

    this->declare_parameter<double>("battery_level", 0.8);
    this->declare_parameter<int>("wildfire_collect_window_ms", 400);
    this->declare_parameter<double>("retardant_depot.x", -28.3);
    this->declare_parameter<double>("retardant_depot.y",  14.0);
    this->declare_parameter<double>("retardant_depot.z",  14.0);

    drone_namespace_ = this->get_parameter("drone_namespace").as_string();
    mission_update_rate_ = this->get_parameter("mission_update_rate").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();

    battery_level_       = this->get_parameter("battery_level").as_double();
    collect_window_ms_   = this->get_parameter("wildfire_collect_window_ms").as_int();
    depot_xyz_.x         = this->get_parameter("retardant_depot.x").as_double();
    depot_xyz_.y         = this->get_parameter("retardant_depot.y").as_double();
    depot_xyz_.z         = this->get_parameter("retardant_depot.z").as_double();

    wildfire_phase_ = ReactionPhase::NONE;


    // Initialise mission management components
    state_machine_ = std::make_unique<StateMachine>();
    path_planner_ = std::make_unique<PathPlanner>();
    mission_executor_ = std::make_unique<MissionExecutor>();
    incident_counter_ = 0;

    // Set drone identifier from namespace
    drone_id_ = drone_namespace_;

    drone_numeric_id_ = 1;
    auto pos = drone_id_.find_last_not_of("0123456789");
    if (pos != std::string::npos && pos + 1 < drone_id_.size()) {
      try { drone_numeric_id_ = std::stoi(drone_id_.substr(pos + 1)); } catch (...) {}
    }


    // Load waypoints from YAML parameters
    loadWaypointsFromParams();
    
    // --- SUBS --- ///
    // Create ROS subscriptions for drone state monitoring
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + drone_namespace_ + "/odom", 10,
      std::bind(&MissionPlannerNode::odomCallback, this, std::placeholders::_1));

    // IMU data is handled by drone_node for orientation control
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/" + drone_namespace_ + "/velocity", 10,
      std::bind(&MissionPlannerNode::velocityCallback, this, std::placeholders::_1));

    // Subscribe to external waypoint commands
    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/" + drone_namespace_ + "/waypoint_command", 10,
      std::bind(&MissionPlannerNode::waypointCallback, this, std::placeholders::_1));

    // Subscribe to perception scenario detection topic
    scenario_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/" + drone_id_ + "/scenario_detection", 10,
        std::bind(&MissionPlannerNode::scenarioDetectionCallback, this, std::placeholders::_1));

    // sub to topic management drones can ping
    info_request_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/" + drone_namespace_ + "/info_request", 10,
        std::bind(&MissionPlannerNode::infoRequestPingCallback, this, std::placeholders::_1));

    assignment_subs_ = this->create_subscription<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/mission_assignment", 10,
      std::bind(&MissionPlannerNode::assignmentCallback, this, std::placeholders::_1));

    // --- PUBS --- ///
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/" + drone_namespace_ + "/cmd_vel", 10);

    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + drone_namespace_ + "/target_pose", 10);

    mission_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/mission_state", 10);

    // Publish incident to gui
    incident_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/incident", 10);

    info_manifest_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/info_manifest", 10);
      
    // --- SRV --- ///
    start_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/start_mission",
      std::bind(&MissionPlannerNode::startMissionCallback, this,
                std::placeholders::_1, std::placeholders::_2));
                
    stop_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/" + drone_namespace_ + "/stop_mission",
      std::bind(&MissionPlannerNode::stopMissionCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Create timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / mission_update_rate_));
    mission_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&MissionPlannerNode::missionTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Mission Planner Node initialised for %s", drone_id_.c_str());
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
          waypoint.pose.position.x = 5.0;
          waypoint.pose.position.y = 5.0;
          waypoint.pose.position.z = 15.0;
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

  void MissionPlannerNode::missionTimerCallback() {
    // Execute mission state machine logic
    executeMission();

    // FETCH_RT dwell logic: after 2s, TAKEOFF and set wildfire leg
    if (in_fetch_rt_ && fetch_landed_) {
      using clock = std::chrono::steady_clock;
      if (clock::now() - fetch_land_steady_ >= std::chrono::seconds(2)) {
        // Prepare second leg (wildfire) and take off
        geometry_msgs::msg::PoseStamped wp_fire;
        wp_fire.header.frame_id = "map"; wp_fire.header.stamp = this->get_clock()->now();
        wp_fire.pose.orientation.w = 1.0;
        wp_fire.pose.position.x = fetch_fire_target_.x;
        wp_fire.pose.position.y = fetch_fire_target_.y;
        wp_fire.pose.position.z = fetch_fire_target_.z;

        path_planner_->setWaypoints({wp_fire});
        state_machine_->setState(MissionState::TAKEOFF);
        fetch_landed_ = false; // proceed with second leg
      }
    }

    // Publish current mission state
    std_msgs::msg::String state_msg;
    state_msg.data = state_machine_->getStateString();
    mission_state_pub_->publish(state_msg);
    
    // Always publish mission commands (including stop commands)
    publishMissionCommand();
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

  std::vector<OrbitPoint> orbitTrajectory(float x, float y, float z,
                                          float radius, int pointCount)
  {
    const int N = std::max(1, pointCount);
    const float R = std::fabs(radius);

    std::vector<OrbitPoint> pts;
    pts.reserve(N);

    if (R == 0.0f)
    {
      for (int i = 0; i < N; ++i)
        pts.push_back({x, y, z, 0.0f});
      return pts;
    }

    constexpr float PI = 3.14159265358979323846f;
    const float step = 2.0f * PI / static_cast<float>(N);

    for (int i = 0; i < N; ++i)
    {
      const float theta = step * static_cast<float>(i);

      const float px = x + R * std::cos(theta);
      const float py = y + R * std::sin(theta);

      float yaw = std::atan2(y - py, x - px); // point back to center
      yaw = normalizeAngle(yaw);

      pts.push_back({px, py, z, yaw});
    }

    return pts;
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

  void MissionPlannerNode::waypointNavigation() {
    if (!path_planner_->hasNextWaypoint()) { 
      state_machine_->setState(MissionState::HOVERING);
      return;
    }
    if (!isWaypointReached()) return;

    // ---- FETCH_RT first leg: land at depot ----
    if (in_fetch_rt_ && !fetch_landed_) {
      state_machine_->setState(MissionState::LANDING);
      return; // landing() + missionTimerCallback will handle dwell + leg 2
    }

    // ---- Normal multi-waypoint progression ----
    (void)path_planner_->getNextWaypoint(); // advances index
    if (!path_planner_->hasNextWaypoint()) {
      RCLCPP_INFO(get_logger(), "All waypoints completed - transitioning to HOVERING");
      state_machine_->setState(MissionState::HOVERING);
    } else {
      RCLCPP_INFO(get_logger(), "Waypoint reached - moving to next waypoint");
    }
  }

  void MissionPlannerNode::landing() {
    static auto landing_start = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::steady_clock::now() - landing_start;

    // Existing timed landing sim
    if (elapsed > std::chrono::seconds(8)) {
      // If this is FETCH_RT and this was the depot landing, mark landed and start the 2s wait
      if (in_fetch_rt_ && !fetch_landed_) {
        fetch_landed_ = true;
        fetch_land_stamp_ = this->now();  // start 2s dwell
        // Stay in LANDING state while we wait; we’ll bump to TAKEOFF in missionTimer
      } else {
        // normal landing back to IDLE
        state_machine_->setState(MissionState::IDLE);
        path_planner_->reset();
      }
      landing_start = std::chrono::steady_clock::now();
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
    static bool did_discover_for_this_incident = false;

    std::optional<ScenarioEvent> event;
    { std::lock_guard<std::mutex> lock(incident_mutex_); event = active_incident_event_; }
    if (!event) { RCLCPP_WARN(get_logger(), "WILDFIRE_REACTION with no active incident"); return; }

    if (!did_discover_for_this_incident) {
      discoverPeerDrones();
      did_discover_for_this_incident = true;
    } else {
      discoverPeerDrones();
    }

    const int closest_peer_id = findClosestPeerToOrigin();
    if (closest_peer_id > 0) {
      std::ostringstream ss;
      ss << "ASSIGN,FETCH_RT,"
        << std::fixed << std::setprecision(2)
        // depot from params
        << depot_xyz_.x << "," << depot_xyz_.y << "," << depot_xyz_.z << ","
        // wildfire FROM THE MESSAGE
        << event->target.x << "," << event->target.y << "," << event->target.z
        << ",mission_id:" << (incident_counter_ + 1);

      const std::string cmd = ss.str();
      {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        auto it = assignment_pubs_.find(closest_peer_id);
        if (it != assignment_pubs_.end()) {
          std_msgs::msg::String out; out.data = cmd;
          it->second->publish(out);
          RCLCPP_INFO(this->get_logger(),
            "Assigned peer %d FETCH_RT: depot[%.2f, %.2f, %.2f] -> fire[%.2f, %.2f, %.2f]",
            closest_peer_id,
            depot_xyz_.x, depot_xyz_.y, depot_xyz_.z,
            event->target.x, event->target.y, event->target.z);
        } else {
          RCLCPP_WARN(this->get_logger(), "No assignment publisher cached for peer %d", closest_peer_id);
        }
      }

      if (state_machine_->canTransition(MissionState::ORBIT_INCIDENT))
        state_machine_->setState(MissionState::ORBIT_INCIDENT);
    } else {
      RCLCPP_WARN(this->get_logger(), "No suitable peer found to assign to incident");
    }
  }

  void MissionPlannerNode::debrisReaction() {}
  void MissionPlannerNode::strandedHikerReaction() {}
  void MissionPlannerNode::orbitIncident() {}

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

  // ---------- parser ----------
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

    ScenarioEvent event;
    event.type = scenario;
    event.target.x = x; event.target.y = y; event.target.z = z;
    event.heading = heading;            // radians, as published
    event.can_respond = can_respond;
    event.stamp = this->now();          // when we received it
    event.raw = data;

    return event;
  }

  // ---------- subscriber callback ----------
  void MissionPlannerNode::scenarioDetectionCallback(const std_msgs::msg::String::SharedPtr msg) {
    auto event = parseScenarioDetection(*msg);
    if (!event) return;

    bool is_duplicate = false;
    {
      std::lock_guard<std::mutex> lock(incident_mutex_);
      if (active_incident_event_ && active_incident_event_->raw == event->raw) {
        // same incident repeatedly published → refresh and bail
        active_incident_event_->stamp = this->now();
        is_duplicate = true;
      } else {
        // new incident → store and reset per-incident phase
        active_incident_event_ = event;
        wildfire_phase_ = ReactionPhase::INIT;
      }
    }
    if (is_duplicate) return;

    // notify GUI outside the lock
    alertIncidentGui(event);

    // enter the reaction state
    MissionState desired = targetStateForScenario(event->type);
    if (state_machine_->canTransition(desired)) {
      state_machine_->setState(desired);
    }
  }

  int MissionPlannerNode::findClosestPeerToOrigin() const {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    if (peer_poses_.empty()) return -1;

    double best_distance = std::numeric_limits<double>::infinity();
    int best_peer_id = -1;
    for (const auto &keyValuePair : peer_poses_) {
      const int peer_id = keyValuePair.first;
      const auto &pose = keyValuePair.second.pose.position;
      double dist = std::hypot(pose.x, pose.y); // planar distance to origin
      if (dist < best_distance) {
        best_distance = dist;
        best_peer_id = peer_id;
      }
    }
    return best_peer_id;
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

  // Creates (or no-ops if already exists) the subscription to /odom and publisher to /mission_assignment for a peer.
  void MissionPlannerNode::createPeerSubscriptionForId(int peer_id) {
    // Redundant check if exist
    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      if (peer_odom_subs_.count(peer_id)) return;
    }

    // namespace strings
    const std::string ns             = "/rs1_drone_" + std::to_string(peer_id);
    const std::string odom_topic     = ns + "/odom";
    const std::string assign_topic   = ns + "/mission_assignment";
    const std::string info_req_topic = ns + "/info_request";
    const std::string info_manifest  = ns + "/info_manifest";

    auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        [this, peer_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
          geometry_msgs::msg::PoseStamped latest_pose;
          latest_pose.header = msg->header;
          latest_pose.pose   = msg->pose.pose;
          std::lock_guard<std::mutex> guard(peers_mutex_);
          peer_poses_[peer_id] = latest_pose;
        });

    auto assignment_pub = this->create_publisher<std_msgs::msg::String>(assign_topic, 1);
    auto info_req_pub   = this->create_publisher<std_msgs::msg::Empty>(info_req_topic, 10);
    auto info_sub       = this->create_subscription<std_msgs::msg::String>(
        info_manifest, 10,
        [this, peer_id](const std_msgs::msg::String::SharedPtr msg) {
          infoManifestCallback(peer_id, msg);
        });

    {
      std::lock_guard<std::mutex> lock(peers_mutex_);
      if (peer_odom_subs_.count(peer_id)) return;
      peer_odom_subs_[peer_id] = std::move(odom_sub);
      assignment_pubs_[peer_id] = std::move(assignment_pub);
      info_request_pubs_[peer_id] = std::move(info_req_pub);
      info_manifest_subs_[peer_id] = std::move(info_sub);
    }

    RCLCPP_INFO(this->get_logger(),
                "Discovered peer %d (odom:%s, assign:%s, info_req:%s, info:%s)",
                peer_id, odom_topic.c_str(), assign_topic.c_str(),
                info_req_topic.c_str(), info_manifest.c_str());
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

  void MissionPlannerNode::assignmentCallback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string payload = trimCopy(msg->data);
    auto tokens = splitCSV(payload);
    for (auto &t : tokens) t = trimCopy(t);
    if (tokens.size() < 2 || tokens[0] != "ASSIGN") return;

    if (tokens[1] == "ORBIT") {
      if (tokens.size() < 5) { RCLCPP_WARN(get_logger(), "Bad ORBIT payload: %s", payload.c_str()); return; }
      double x{}, y{}, z{};
      if (!parseDouble(tokens[2], x) || !parseDouble(tokens[3], y) || !parseDouble(tokens[4], z)) {
        RCLCPP_WARN(get_logger(), "ORBIT coords parse failed: %s", payload.c_str()); return;
      }
      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = "map"; wp.header.stamp = this->get_clock()->now();
      wp.pose.orientation.w = 1.0; wp.pose.position.x = x; wp.pose.position.y = y; wp.pose.position.z = z;
      path_planner_->setWaypoints({wp});
      if (state_machine_->canTransition(MissionState::WAYPOINT_NAVIGATION))
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      return;
    }

    if (tokens[1] == "FETCH_RT" || tokens[1] == "FETCH_RETARDANT") {
      // Expect: ASSIGN,FETCH_RT,dep_x,dep_y,dep_z,fire_x,fire_y,fire_z,mission_id:NN
      if (tokens.size() < 9) { RCLCPP_WARN(get_logger(), "Bad FETCH_RT payload: %s", payload.c_str()); return; }
      double dx{}, dy{}, dz{}, fx{}, fy{}, fz{};
      if (!parseDouble(tokens[2], dx) || !parseDouble(tokens[3], dy) || !parseDouble(tokens[4], dz) ||
          !parseDouble(tokens[5], fx) || !parseDouble(tokens[6], fy) || !parseDouble(tokens[7], fz)) {
        RCLCPP_WARN(get_logger(), "FETCH_RT coords parse failed: %s", payload.c_str()); return;
      }

      in_fetch_rt_ = true;
      fetch_landed_ = false;
      fetch_fire_target_.x = fx; fetch_fire_target_.y = fy; fetch_fire_target_.z = fz;

      geometry_msgs::msg::PoseStamped wp_depot, wp_fire;
      wp_depot.header.frame_id = wp_fire.header.frame_id = "map";
      wp_depot.header.stamp = wp_fire.header.stamp = this->get_clock()->now();
      wp_depot.pose.orientation.w = wp_fire.pose.orientation.w = 1.0;
      wp_depot.pose.position.x = dx; wp_depot.pose.position.y = dy; wp_depot.pose.position.z = dz;
      wp_fire.pose.position.x  = fx; wp_fire.pose.position.y  = fy; wp_fire.pose.position.z  = fz;

      // FIRST LEG: go to depot only
      path_planner_->setWaypoints({wp_depot});
      if (state_machine_->canTransition(MissionState::WAYPOINT_NAVIGATION)) {
        state_machine_->setState(MissionState::WAYPOINT_NAVIGATION);
      }
      return;
    }
  }

void MissionPlannerNode::infoManifestCallback(int peer_id, const std_msgs::msg::String::SharedPtr& msg) {
  // Expect: id:N,battery:0.80,state:STATE,x:..,y:..,z:..,t:..
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
  if (id_from_msg > 0 && id_from_msg == peer_id) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    peer_info_[peer_id] = pi;
  }
}

}  // namespace drone_swarm
