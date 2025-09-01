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

    // TODO: ADD NAV2 RRT PLANNER PARAMETERS FOR AUTONOMOUS PATH PLANNING
    // this->declare_parameter<double>("replanning_frequency", 1.0);  // Hz
    // this->declare_parameter<double>("planning_timeout", 10.0);     // seconds
    // this->declare_parameter<std::string>("planner_id", "RRT*");    // RRT planner type
    // this->declare_parameter<std::string>("global_frame", "map");   // Global coordinate frame
    // Example: Declare parameters to configure RRT planning behavior and timeouts

    drone_namespace_ = this->get_parameter("drone_namespace").as_string();
    mission_update_rate_ = this->get_parameter("mission_update_rate").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();

    // TODO: GET NAV2 RRT PLANNER PARAMETERS FOR AUTONOMOUS PATH PLANNING
    // replanning_frequency_ = this->get_parameter("replanning_frequency").as_double();
    // planning_timeout_ = this->get_parameter("planning_timeout").as_double();
    // planner_id_ = this->get_parameter("planner_id").as_string();
    // Example: Read RRT planner configuration from parameters

    // Initialise mission management components
    state_machine_ = std::make_unique<StateMachine>();
    path_planner_ = std::make_unique<PathPlanner>();
    mission_executor_ = std::make_unique<MissionExecutor>();

    // TODO: INITIALISE NAV2 RRT PLANNER CLIENT FOR AUTONOMOUS PATH PLANNING
    // path_planner_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
    //   this, "compute_path_to_pose");
    // path_planning_active_ = false;
    // last_replan_time_ = this->get_clock()->now();
    // Example: Create action client to communicate with nav2 planner server
    // IMPORTANT: Ensure nav2 planner server is running before sending requests

    // Set drone identifier from namespace
    drone_id_ = drone_namespace_;

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

    // TODO: ADD NAV2 RRT PLANNER SUBSCRIPTIONS FOR AUTONOMOUS PATH PLANNING
    // Add a cost_map_sub_ and map_sub_
    //
    // Example: Subscribe to costmap and map updates to trigger replanning when obstacles change

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
    
    // TODO: ADD NAV2 RRT PLANNER GOAL SETTING FOR AUTONOMOUS PATH PLANNING
    // // Set mission goal pose (either from parameters or service request)
    // mission_goal_.header.frame_id = "map";  // Use global frame
    // mission_goal_.header.stamp = this->get_clock()->now();
    // mission_goal_.pose.position.x = 10.0;   // Example goal coordinates
    // mission_goal_.pose.position.y = 5.0;
    // mission_goal_.pose.position.z = 2.0;
    // mission_goal_.pose.orientation.w = 1.0; // No rotation
    // 
    // // Request initial path planning to mission goal
    // if (!requestPathPlanning(current_pose_, mission_goal_)) {
    //   response->success = false;
    //   response->message = "Failed to start path planning to mission goal";
    //   return;
    // }
    // Example: Set mission goal and request path planning instead of manual waypoints
    
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
            
            // TODO: REPLACE MANUAL WAYPOINT CHECK WITH NAV2 RRT PLANNER PATH EXECUTION
            // // After takeoff, ensure we have a planned path to the mission goal
            // 
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
        // TODO: ADD NAV2 RRT PLANNER REPLANNING CHECK FOR AUTONOMOUS PATH PLANNING
        
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

  void MissionPlannerNode::loadWaypointsFromParams() {
    // TODO: Implement parameter-based waypoint loading
    // This could load waypoints from ROS parameters for autonomous missions
    RCLCPP_DEBUG(this->get_logger(), "Waypoint parameter loading not yet implemented");
    
    /* Example implementation:
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    
    try {
      auto waypoint_list = this->get_parameter("waypoints").as_double_array();
      
      for (size_t i = 0; i + 2 < waypoint_list.size(); i += 3) {
        geometry_msgs::msg::PoseStamped wp;
        wp.header.frame_id = "map";
        wp.pose.position.x = waypoint_list[i];
        wp.pose.position.y = waypoint_list[i + 1];
        wp.pose.position.z = waypoint_list[i + 2];
        wp.pose.orientation.w = 1.0; // Default orientation
        waypoints.push_back(wp);
      }
      
      path_planner_->setWaypoints(waypoints);
      RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from parameters", waypoints.size());
      
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "No waypoints found in parameters: %s", e.what());
    }
    */
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

  // TODO: ADD NAV2 RRT PLANNER IMPLEMENTATION METHODS FOR AUTONOMOUS PATH PLANNING
  // 
  // bool MissionPlannerNode::requestPathPlanning(const geometry_msgs::msg::PoseStamped& start_pose,
  //                                              const geometry_msgs::msg::PoseStamped& goal_pose) {
  //   1. Check if nav2 planner server is available
  //   
  //   2. Create path planning request
  //   
  //   3. Set planning options
  //   
  //   4. Send planning request
  // }
  // 
  // void MissionPlannerNode::pathPlanningResultCallback(
  //   const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult& result) {
  //   
  //   path_planning_active_ = false;
  //   
  //   if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
  //     current_planned_path_ = result.result->path;
  //     convertPathToWaypoints(current_planned_path_);
  //     RCLCPP_INFO(this->get_logger(), "Path planning succeeded - %zu waypoints generated", 
  //                 current_planned_path_.poses.size());
  //   } else {
  //     RCLCPP_ERROR(this->get_logger(), "Path planning failed with code: %d", (int)result.code);
  //     // Handle planning failure - maybe try alternative goal or enter emergency mode
  //   }
  // }
  // 
  // void MissionPlannerNode::convertPathToWaypoints(const nav_msgs::msg::Path& path) {
  //   if (path.poses.empty()) {
  //     RCLCPP_WARN(this->get_logger(), "Cannot convert empty path to waypoints");
  //     return;
  //   }
  //   
  //   // Optionally subsample the path to reduce waypoint density
  //   std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  //   double min_waypoint_distance = 1.0;  // Minimum distance between waypoints
  //   
  //   waypoints.push_back(path.poses[0]);  // Always include first waypoint
  //   
  //   for (size_t i = 1; i < path.poses.size(); ++i) {
  // }
  // 
  // bool MissionPlannerNode::needsReplanning() const {
  //   // Check various conditions that would require replanning:
  //   // 1. Significant deviation from planned path
  //   // 2. New obstacles detected in costmap
  //   // 3. Goal position changed
  //   // 4. Path no longer valid due to environment changes
  //   
  //   
  //   // Example: Check if drone is too far from planned path
  //   // Find closest point on path and check distance
  // }
  // Example: These methods implement the core RRT path planning functionality

}  // namespace drone_swarm
