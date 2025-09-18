#include "mission_node.h"

namespace drone_swarm
{

  MissionPlannerNode::MissionPlannerNode(const rclcpp::NodeOptions& options)
    : Node("mission_planner", options)
  {
    // Initialise ROS parameters for mission configuration
    this->declare_parameter("drone_namespace", std::string("rs1_drone"));
    this->declare_parameter<double>("mission_update_rate", 10.0);
    this->declare_parameter<double>("waypoint_tolerance", 0.5);

    drone_namespace_ = this->get_parameter("drone_namespace").as_string();
    mission_update_rate_ = this->get_parameter("mission_update_rate").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();

    // Initialise mission management components
    state_machine_ = std::make_unique<StateMachine>();
    path_planner_ = std::make_unique<PathPlanner>();
    mission_executor_ = std::make_unique<MissionExecutor>();

    // Set drone identifier from namespace
    drone_id_ = drone_namespace_;

    // Load waypoints from YAML parameters
    loadWaypointsFromParams();
    
    // Create ROS subscriptions for drone state monitoring
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + drone_namespace_ + "/odom", 10,
      std::bind(&MissionPlannerNode::odomCallback, this, std::placeholders::_1));

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/" + drone_namespace_ + "/velocity", 10,
      std::bind(&MissionPlannerNode::velocityCallback, this, std::placeholders::_1));

    // Subscribe to external waypoint commands
    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/" + drone_namespace_ + "/waypoint_command", 10,
      std::bind(&MissionPlannerNode::waypointCallback, this, std::placeholders::_1));

    // Create publishers  
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/" + drone_namespace_ + "/cmd_vel", 10);
      
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + drone_namespace_ + "/target_pose", 10);

    mission_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/" + drone_namespace_ + "/mission_state", 10);

    // Create services
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
        {
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
        break;
        
      case MissionState::WAYPOINT_NAVIGATION:
        
        // Check if current waypoint is reached
        if (isWaypointReached()) {
          if (path_planner_->hasNextWaypoint()) {
            RCLCPP_INFO(this->get_logger(), "Waypoint reached - moving to next waypoint");
          } else {
            RCLCPP_INFO(this->get_logger(), "All waypoints completed - transitioning to HOVERING");
            state_machine_->setState(MissionState::HOVERING);
          }
        }
        break;
        
      case MissionState::HOVERING:
        // Maintain position - could wait for new waypoints or manual commands
        RCLCPP_DEBUG(this->get_logger(), "Hovering at current position");
        break;
        
      case MissionState::LANDING:
        // Monitor landing completion
        {
          static auto landing_start = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::steady_clock::now() - landing_start;
          
          if (elapsed > std::chrono::seconds(8)) { // 8 second landing simulation
            RCLCPP_INFO(this->get_logger(), "Landing completed for %s - transitioning to IDLE", drone_id_.c_str());
            state_machine_->setState(MissionState::IDLE);
            path_planner_->reset(); // Reset waypoints for next mission
            landing_start = std::chrono::steady_clock::now(); // Reset for next time
          }
        }
        break;
        
      case MissionState::MANUAL_CONTROL:
        // External control - monitor for return to autonomous mode
        RCLCPP_DEBUG(this->get_logger(), "In manual control mode");
        break;
        
      case MissionState::EMERGENCY:
        // Emergency handling - immediate landing
        RCLCPP_WARN(this->get_logger(), "Emergency state active - initiating emergency landing");
        state_machine_->setState(MissionState::LANDING);
        break;
    }
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

}  // namespace drone_swarm
