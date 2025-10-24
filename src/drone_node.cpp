#include "drone_node.h"
#include <cmath>
#define _USE_MATH_DEFINES

namespace drone_swarm
{

  // DroneControllerNode Implementation
  DroneControllerNode::DroneControllerNode(const rclcpp::NodeOptions &options)
      : Node("drone_controller", options), current_flight_mode_(FlightMode::DISARMED), armed_(false)
  {
    // Initialise ROS parameters for drone configuration
    this->declare_parameter("drone_namespace", std::string("rs1_drone"));

    drone_namespace_ = this->get_parameter("drone_namespace").as_string();

    // Initialise control components for flight management
    drone_control_ = std::make_unique<DroneControl>();
    // sensor_manager_ = std::make_unique<SensorManager>();  // Commented out - future implementation

    last_control_update_ = std::chrono::steady_clock::now();

    // Create ROS subscriptions using drone namespace pattern
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

    // gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    //     "/" + drone_namespace_ + "/gps", 10,
    //     std::bind(&DroneControllerNode::gpsCallback, this, std::placeholders::_1));

    // Subscribe to laser and sonar (future implementation)
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/" + drone_namespace_ + "/laserscan", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
          // TODO: Process laser data when sensor manager is implemented
          (void)msg;
        });

    sonar_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/" + drone_namespace_ + "/sonar", 10,
        [this](const sensor_msgs::msg::Range::SharedPtr msg)
        {
          // TODO: Process sonar data when sensor manager is implemented
          (void)msg;
        });

    mission_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/" + drone_namespace_ + "/mission_state", 10,
        std::bind(&DroneControllerNode::missionStateCallback, this, std::placeholders::_1));

    // Create publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + drone_namespace_ + "/cmd_vel", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + drone_namespace_ + "/pose", 10);
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + drone_namespace_ + "/velocity", 10);
    flight_mode_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/" + drone_namespace_ + "/flight_mode", 10);

    // Create services

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
    else if (current_mission_state_ == "WAYPOINT_NAVIGATION")
    {
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
    this->declare_parameter("control_frequency", 10.0);
    this->declare_parameter("telemetry_frequency", 5.0);
    this->declare_parameter("takeoff_altitude", 2.0);
    this->declare_parameter("landing_speed", 0.5);
    this->declare_parameter("hover_altitude_tolerance", 0.2);

    control_frequency_ = this->get_parameter("control_frequency").as_double();
    telemetry_frequency_ = this->get_parameter("telemetry_frequency").as_double();

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
        // Disarm the drone
        armed_ = false;
        current_flight_mode_ = FlightMode::DISARMED;
        // TODO: Clear waypoints when waypoint system is implemented

        RCLCPP_INFO(this->get_logger(), "Drone disarmed and in IDLE state");
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

  // void DroneControllerNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  // {
  //   (void)msg; // Suppress unused parameter warning
  //   // TODO: Process GPS data for position estimation
  //   // sensor_manager_->updateGPS(msg);
  //   RCLCPP_DEBUG(this->get_logger(), "GPS data received");
  // }

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

    // Simple takeoff control - ascend to target altitude
    double target_altitude = this->get_parameter("takeoff_altitude").as_double();
    double current_altitude = current_odom_.pose.pose.position.z;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0;

    if (current_altitude < target_altitude - 0.2)
    {
      // Ascend at controlled rate
      cmd_vel.linear.z = 0.5; // 0.5 m/s ascent rate
      RCLCPP_DEBUG(this->get_logger(), "Ascending - Current: %.2fm, Target: %.2fm",
                   current_altitude, target_altitude);
    }
    else
    {
      // Maintain altitude - takeoff complete
      cmd_vel.linear.z = 0.0;
      RCLCPP_DEBUG(this->get_logger(), "Takeoff altitude reached - hovering");
    }

    cmd_vel_pub_->publish(cmd_vel);
  }

  void DroneControllerNode::executeWaypointNavigation() {
    // Future implementation for waypoint navigation
    // 

    // Use target pose from drone node
    if (target_pose_.header.stamp.sec == 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "No target pose available for navigation");
      return;
    }

    // Calculate control command using drone control system
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.stamp = this->get_clock()->now();
    current_pose.pose = current_odom_.pose.pose;

    geometry_msgs::msg::Twist cmd_vel = drone_control_->calculateAdvancedPositionControl(
        current_pose, target_pose_, 0.05); // 20Hz control loop = 0.05s dt

    cmd_vel_pub_->publish(cmd_vel);

    // Check if target is reached
    double distance = calculateDistanceToWaypoint(current_pose.pose, target_pose_.pose);

    if (distance < 0.8)
    { // Target tolerance
      RCLCPP_DEBUG(this->get_logger(), "Target reached (distance: %.2fm)", distance);
    }

    RCLCPP_DEBUG(this->get_logger(), "Navigating to target - distance: %.2fm", distance);
  }

  void DroneControllerNode::executeLandingSequence() {
    // Simple landing control - descend at controlled rate
    double landing_speed = this->get_parameter("landing_speed").as_double();
    double current_altitude = current_odom_.pose.pose.position.z;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0;

    if (current_altitude > 0.3)
    {                                    // Land until 30cm above ground
      cmd_vel.linear.z = -landing_speed; // Descend
      RCLCPP_DEBUG(this->get_logger(), "Landing - Current altitude: %.2fm", current_altitude);
    }
    else
    {
      // Landing complete
      cmd_vel.linear.z = 0.0;
      RCLCPP_DEBUG(this->get_logger(), "Landing complete - altitude: %.2fm", current_altitude);
    }

    cmd_vel_pub_->publish(cmd_vel);
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

  // Commented out methods for future implementation
  /*
  void DroneControllerNode::handleEmergency()
  {
    // TODO: Implement emergency handling
    current_flight_mode_ = FlightMode::EMERGENCY_LAND;
    RCLCPP_ERROR(this->get_logger(), "Emergency situation detected - initiating emergency landing");
  }
  */

} // namespace drone_swarm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_swarm::DroneControllerNode)
