#include "drone_node.h"
#include <cmath>
#define _USE_MATH_DEFINES

namespace drone_swarm
{

  // DroneControllerNode Implementation
  DroneControllerNode::DroneControllerNode(const rclcpp::NodeOptions &options, const std::string & name)
      : Node( (name.empty() ? std::string("drone_controller_") + std::to_string(getpid()) : name), rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)),
        current_flight_mode_(FlightMode::DISARMED), current_sonar_range_(0.0), armed_(false)
  {
    // Initialise ROS parameters for drone configuration
    std::string drone_namespace;
    if (this->has_parameter("drone_namespace")) {
      drone_namespace = this->get_parameter("drone_namespace").as_string();
    } else {
      drone_namespace = "rs1_drone";  // fallback default
    }
    
    // Store the namespace (without leading slash since we're already namespaced)
    drone_namespace_ = drone_namespace;

    // Initialise control components for flight management
    drone_control_ = std::make_unique<DroneControl>();
    
    last_control_update_ = std::chrono::steady_clock::now();

    // Subscribe to topics using explicit namespace like mission_node
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + drone_namespace_ + "/odom", 10,
        std::bind(&DroneControllerNode::odomCallback, this, std::placeholders::_1));

    goals_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/" + drone_namespace_ + "/mission/goals", 10,
        std::bind(&DroneControllerNode::goalCallback, this, std::placeholders::_1));

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/" + drone_namespace_ + "/target_pose", 10,
        std::bind(&DroneControllerNode::targetPoseCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/" + drone_namespace_ + "/imu", 10,
        std::bind(&DroneControllerNode::imuCallback, this, std::placeholders::_1));

    // Subscribe to laser and sonar
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/" + drone_namespace_ + "/lidar", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          current_lidar_data_ = *msg;
        });

    sonar_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/" + drone_namespace_ + "/sonar", 10,
        [this](const sensor_msgs::msg::Range::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          current_sonar_range_ = msg->range;
        });

    mission_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/" + drone_namespace_ + "/mission_state", 10,
        std::bind(&DroneControllerNode::missionStateCallback, this, std::placeholders::_1));

    // Create publishers using explicit namespace like mission_node
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + drone_namespace_ + "/cmd_vel", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + drone_namespace_ + "/pose", 10);
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + drone_namespace_ + "/velocity", 10);
    flight_mode_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/" + drone_namespace_ + "/flight_mode", 10);

    // Configure DroneControl with publisher and logger
    drone_control_->setCmdVelPublisher(cmd_vel_pub_);
    drone_control_->setLogger(this->get_logger());

    // Load control parameters
    loadControlParams();

    // Create control timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&DroneControllerNode::controlLoopCallback, this));

    RCLCPP_INFO(this->get_logger(), "Drone Controller Node initialized for %s", drone_namespace_.c_str());
  }

  // Template implementations for callback functions
  void DroneControllerNode::controlLoopCallback() {
    // Main control loop - executed at control frequency

    // Check current mission state and execute appropriate control
    if (current_mission_state_ == "TAKEOFF")
    {
      executeTakeoffSequence();
    }
    else if (current_mission_state_ == "WAYPOINT_NAVIGATION" || 
             current_mission_state_ == "RESPONSE_NAVIGATION")
    {
      // Both normal waypoint navigation and scenario response use same navigation logic
      executeWaypointNavigation();
    }
    else if (current_mission_state_ == "LANDING")
    {
      executeLandingSequence();
    }
    else if (current_mission_state_ == "HOVERING")
    {
      executeHoverControl();
    }

    // Publish telemetry
    publishTelemetry();

    RCLCPP_DEBUG(this->get_logger(), "Control loop executed - Flight mode: %s, Mission state: %s",
                 flightModeToString(current_flight_mode_).c_str(), current_mission_state_.c_str());
  }

  void DroneControllerNode::loadControlParams() {
    // Load control parameters from ROS parameters
    // Use get_parameter_or instead of declare + get to avoid redeclaration errors
    this->get_parameter_or("control_frequency", control_frequency_, 10.0);
    this->get_parameter_or("telemetry_frequency", telemetry_frequency_, 5.0);

    // Validate frequencies
    if (control_frequency_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Invalid control frequency %.2f, using default 20.0 Hz", control_frequency_);
      control_frequency_ = 20.0;
    }

    // Configure drone control system
    drone_control_->setLogger(this->get_logger());
    drone_control_->setCmdVelPublisher(cmd_vel_pub_);

    RCLCPP_INFO(this->get_logger(), "Control parameters loaded - Control freq: %.1fHz, Telemetry freq: %.1fHz",
                control_frequency_, telemetry_frequency_);
  }

  void DroneControllerNode::missionStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string previous_state = current_mission_state_;
    current_mission_state_ = msg->data;

    if (previous_state != current_mission_state_)
    {
      RCLCPP_INFO(this->get_logger(), "Mission state changed from %s to %s",
                  previous_state.c_str(), current_mission_state_.c_str());

      // Handle state transitions
      if (current_mission_state_ == "TAKEOFF")
      {
        // Arm the drone and prepare for takeoff
        armed_ = true;
        current_flight_mode_ = FlightMode::GUIDED;
        takeoff_start_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Drone armed and ready for takeoff");
      }
      else if (current_mission_state_ == "LANDING")
      {
        // Prepare for landing
        current_flight_mode_ = FlightMode::GUIDED;
        landing_start_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Initiating landing sequence");
      }
      else if (current_mission_state_ == "IDLE")
      {
        // Keep drone armed and maintain altitude hold at ground level (~0m)
        // This prevents gravity-induced descent when idle
        armed_ = true;  // Keep armed to maintain altitude
        current_flight_mode_ = FlightMode::GUIDED;  // Use GUIDED for altitude hold
        
        // Set target to current horizontal position and current sonar altitude
        // Use sonar for altitude consistency with takeoff monitoring
        // Sonar returns invalid readings (>5m) when on ground, so clamp to ground level
        target_pose_.pose.position.x = current_odom_.pose.pose.position.x;
        target_pose_.pose.position.y = current_odom_.pose.pose.position.y;
        
        // Use sonar if valid (0-7m range), otherwise maintain ground level
        if (current_sonar_range_ >= 0.0 && current_sonar_range_ <= 7.0) {
          target_pose_.pose.position.z = current_sonar_range_;
          RCLCPP_DEBUG(this->get_logger(), "IDLE state - maintaining altitude hold at %.2fm (sonar)", 
                       current_sonar_range_);
        } else {
          target_pose_.pose.position.z = 0.0;  // Ground level default
          RCLCPP_DEBUG(this->get_logger(), "IDLE state - sonar out of range (%.2fm), holding at ground", 
                       current_sonar_range_);
        }
      }
    }
  }

  void DroneControllerNode::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_ = *msg;

    RCLCPP_DEBUG(this->get_logger(), "Target pose updated: [%.2f, %.2f, %.2f]",
                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void DroneControllerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Extract yaw angle from quaternion orientation
    double x = msg->orientation.x;
    double y = msg->orientation.y;
    double z = msg->orientation.z;
    double w = msg->orientation.w;
    
    // Convert quaternion to yaw angle (Euler Z rotation)
    double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    
    // Update drone control with current yaw
    drone_control_->updateCurrentYaw(yaw);
    
    RCLCPP_DEBUG(this->get_logger(), "IMU data received - Yaw: %.2f rad (%.1f deg)", 
                 yaw, yaw * 180.0 / M_PI);
  }

  void DroneControllerNode::goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // Future implementation for waypoint handling
    (void)msg; // Suppress unused parameter warning

    RCLCPP_INFO(this->get_logger(), "Received %zu waypoints for %s (waypoint system not yet implemented)",
                msg->poses.size(), drone_namespace_.c_str());

    // Log waypoints for debugging (future implementation)
    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
      const auto &wp = msg->poses[i];
      RCLCPP_INFO(this->get_logger(), "Waypoint %zu: [%.2f, %.2f, %.2f]",
                  i, wp.position.x, wp.position.y, wp.position.z);
    }

    // If drone is in waypoint navigation mode, start navigating immediately
    if (current_mission_state_ == "WAYPOINT_NAVIGATION")
    {
      RCLCPP_INFO(this->get_logger(), "Starting waypoint navigation immediately");
    }
  }

  void DroneControllerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store current odometry for navigation
    current_odom_ = *msg;

    // Update pose publisher with current position
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header = msg->header;
    current_pose.pose = msg->pose.pose;
    pose_pub_->publish(current_pose);

    RCLCPP_DEBUG(this->get_logger(), "Odometry updated: [%.2f, %.2f, %.2f]",
                 msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  // Mission execution methods
  void DroneControllerNode::executeTakeoffSequence()
  {
    if (!armed_)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot execute takeoff - drone not armed");
      return;
    }

    // Get current sonar altitude reading
    double current_altitude = 0.0;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      current_altitude = current_sonar_range_;
    }
    
    // Validate sonar reading - if invalid (inf, -inf, nan), assume ground level
    if (!std::isfinite(current_altitude) || current_altitude < 0.0) {
      current_altitude = 0.0;  // Assume ground level if sonar is invalid
      RCLCPP_DEBUG(this->get_logger(), "Invalid sonar reading, assuming ground level (0m)");
    }
    
    // Calculate elapsed time
    auto now = this->get_clock()->now();
    double elapsed_seconds = (now - takeoff_start_time_).seconds();
    
    // Use DroneControl for takeoff with sonar feedback
    // Target 8m to ensure drone climbs even if sonar reads >5m on ground (invalid reading)
    // Once in valid range (0-7m), actual altitude will be detected properly
    double target_altitude = 8.0;  // Higher target to overcome invalid ground readings
    this->get_parameter_or("takeoff_altitude", target_altitude, 8.0);
    
    bool takeoff_complete = drone_control_->takeoff(
        target_altitude, current_altitude, elapsed_seconds);
    
    if (takeoff_complete) {
      RCLCPP_INFO(this->get_logger(), "Takeoff sequence completed - altitude reached");
    }
  }

  void DroneControllerNode::executeWaypointNavigation() {
    // Check if target pose is available
    if (target_pose_.header.stamp.sec == 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "No target pose available for navigation");
      return;
    }

    // Get current pose from odometry
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.stamp = this->get_clock()->now();
    current_pose.pose = current_odom_.pose.pose;

    // Get sensor data for altitude control
    double current_sonar_altitude = 0.0;
    sensor_msgs::msg::LaserScan lidar_data;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      current_sonar_altitude = current_sonar_range_;
      lidar_data = current_lidar_data_;
    }

    // Use DroneControl with altitude control for terrain following
    geometry_msgs::msg::Twist cmd_vel = drone_control_->navigateToWaypointWithAltitudeControl(
        current_pose, target_pose_, current_sonar_altitude, lidar_data);

    cmd_vel_pub_->publish(cmd_vel);

    // Check if target is reached (2D distance only, altitude is maintained by terrain following)
    double dx = target_pose_.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    if (distance < 0.8) { // Target tolerance
      RCLCPP_DEBUG(this->get_logger(), "Target reached (distance: %.2fm)", distance);
    }

    RCLCPP_DEBUG(this->get_logger(), "Navigating to target - distance: %.2fm, altitude: %.2fm", 
                 distance, current_sonar_altitude);
  }

  void DroneControllerNode::executeLandingSequence() {
    // Get current sonar altitude reading
    double current_altitude = 0.0;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      current_altitude = current_sonar_range_;
    }
    
    // Calculate elapsed time
    auto now = this->get_clock()->now();
    double elapsed_seconds = (now - landing_start_time_).seconds();
    
    // Use DroneControl for landing with sonar feedback
    double target_landing_altitude = 0.2;  // 0.2m above ground
    this->get_parameter_or("landing_altitude", target_landing_altitude, 0.2);
    
    bool landing_complete = drone_control_->land(
        target_landing_altitude, current_altitude, elapsed_seconds);
    
    if (landing_complete) {
      RCLCPP_INFO(this->get_logger(), "Landing sequence completed - on ground");
    }
  }

  void DroneControllerNode::executeHoverControl() {
    // Hover at current position using target pose
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.stamp = this->get_clock()->now();
    current_pose.pose = current_odom_.pose.pose;

    // Use current position as target for hovering
    geometry_msgs::msg::PoseStamped hover_target = target_pose_;
    if (hover_target.pose.position.x == 0.0 && hover_target.pose.position.y == 0.0)
    {
      // Use current position if no target set
      hover_target = current_pose;
    }

    geometry_msgs::msg::Twist cmd_vel = drone_control_->calculateAdvancedPositionControl(
        current_pose, hover_target, 0.05);

    cmd_vel_pub_->publish(cmd_vel);

    RCLCPP_DEBUG(this->get_logger(), "Hovering at position [%.2f, %.2f, %.2f]",
                 hover_target.pose.position.x, hover_target.pose.position.y, hover_target.pose.position.z);
  }

  // Helper methods
  double DroneControllerNode::calculateDistanceToWaypoint(const geometry_msgs::msg::Pose &current,
                                                          const geometry_msgs::msg::Pose &target) const
  {
    double dx = target.position.x - current.position.x;
    double dy = target.position.y - current.position.y;
    double dz = target.position.z - current.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  std::string DroneControllerNode::flightModeToString(FlightMode mode) const {
    switch (mode)
    {
    case FlightMode::MANUAL:
      return "MANUAL";
    case FlightMode::STABILISE:
      return "STABILISE";
    case FlightMode::GUIDED:
      return "GUIDED";
    case FlightMode::AUTO:
      return "AUTO";
    case FlightMode::EMERGENCY_LAND:
      return "EMERGENCY_LAND";
    default:
      return "UNKNOWN";
    }
  }

  void DroneControllerNode::publishTelemetry() {
    // Publish flight mode
    std_msgs::msg::String flight_mode_msg;
    flight_mode_msg.data = flightModeToString(current_flight_mode_);
    flight_mode_pub_->publish(flight_mode_msg);

    // Publish velocity (from odometry)
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg = current_odom_.twist.twist;
    velocity_pub_->publish(velocity_msg);
  }

} // namespace drone_swarm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_swarm::DroneControllerNode)
