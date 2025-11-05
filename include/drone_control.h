/**
 * @file drone_control.h
 * @brief Advanced drone control class with PID controllers and flight navigation
 * @author Jackson Russell
 * OTHER AUTHORS ADD HERE AND BELOW
 * @date August-2025
 */

#ifndef DRONE_SWARM_DRONE_CONTROL_H_
#define DRONE_SWARM_DRONE_CONTROL_H_

#include <memory>
#include <mutex>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pid.h"

namespace drone_swarm
{

/**
 * @struct VehicleData
 * @brief Structure to hold vehicle data for drone state management
 * 
 * Contains odometry information and mission status for individual drones
 * in the swarm with thread-safe access patterns.
 */
struct VehicleData {
  std::string id;                     ///< Unique drone identifier
  nav_msgs::msg::Odometry current_odom;  ///< Current odometry data
  std::mutex odom_mutex;              ///< Thread-safe odometry access
  bool mission_active;                ///< Mission status flag
  
  /**
   * @brief Default constructor
   * Initialises mission_active to false
   */
  VehicleData() : mission_active(false) {}
};

/**
 * @class DroneControl
 * @brief Advanced drone control system with multiple control modes
 * 
 * Provides comprehensive flight control including position control,
 * velocity control, and high-level navigation capabilities with
 * enhanced flyToGoal functionality for autonomous missions.
 */
class DroneControl
{
public:
  /**
   * @brief Default constructor
   * Initialises PID controllers and sets default parameters
   */
  DroneControl();
  
  /**
   * @brief Initialise physical parameters
   * @param mass Drone mass in kilograms
   * @param max_thrust Maximum thrust capability in Newtons
   */
  void initialise(double mass, double max_thrust);
  
  /**
   * @brief Set ROS 2 logger for debugging output
   * @param logger ROS 2 logger instance
   */
  void setLogger(rclcpp::Logger logger);
  
  /**
   * @brief Set command velocity publisher
   * @param pub ROS 2 publisher for geometry_msgs::msg::Twist
   */
  void setCmdVelPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub);
  
  /**
   * @brief Calculate control output for position tracking
   * @param current_pose Current drone pose
   * @param target_pose Desired target pose
   * @param current_velocity Current velocity for feedforward control
   * @param dt Time step since last calculation
   * @return Twist command for drone actuation
   */
  geometry_msgs::msg::Twist calculateControlOutput(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    double dt);
  
  /**
   * @brief Update position PID controller gains
   * @param kp Proportional gain
   * @param ki Integral gain  
   * @param kd Derivative gain
   */
  void updatePositionPIDGains(double kp, double ki, double kd);
  
  /**
   * @brief Update velocity PID controller gains
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void updateVelocityPIDGains(double kp, double ki, double kd);
  
  /**
   * @brief Update attitude PID controller gains
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void updateAttitudePIDGains(double kp, double ki, double kd);
  
  /**
   * @brief Set control output limits
   * @param max_velocity Maximum linear velocity (m/s)
   * @param max_angular_velocity Maximum angular velocity (rad/s)
   */
  void setControlLimits(double max_velocity, double max_angular_velocity);
  
  /**
   * @brief Reset all PID controllers
   * Clears integral accumulators and error history
   */
  void resetControllers();
  
  /**
   * @brief Enhanced position control with advanced features
   * @param current_pose Current drone pose
   * @param target_pose Desired target pose  
   * @param dt Time step since last calculation
   * @param use_manual_altitude Enable manual altitude control mode
   * @return Twist command optimised for smooth flight
   */
  geometry_msgs::msg::Twist calculateAdvancedPositionControl(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose,
    double dt,
    bool use_manual_altitude = true);
  
  /**
   * @brief High-level navigation to goal pose
   * @param vehicle Vehicle data structure with current state
   * @param goal Target pose to reach
   * @return True if goal reached, false if still navigating
   * 
   * Implements advanced flyToGoal algorithm with obstacle avoidance
   * and smooth trajectory following for autonomous missions.
   */
  bool flyToGoal(VehicleData& vehicle, const geometry_msgs::msg::Pose& goal);
  
  /**
   * @brief Navigate to 2D waypoint using terrain-following altitude control
   * @param current_pose Current drone position from odometry
   * @param target_waypoint Target waypoint (only X,Y used, Z ignored)
   * @param sonar_range Current sonar altitude reading
   * @param lidar_data Current LiDAR scan for obstacle detection
   * @param dt Time step for control calculations
   * @return Twist command for waypoint navigation with terrain following
   * 
   * Navigates to X,Y coordinates while maintaining constant altitude above ground
   * using sonar feedback. Uses LiDAR to detect obstacles and climb over them.
   */
  geometry_msgs::msg::Twist navigateToWaypointWithAltitudeControl(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_waypoint,
    double sonar_range,
    const sensor_msgs::msg::LaserScan& lidar_data,
    double dt = 0.1);
  
  /**
   * @brief Set target altitude above ground for terrain following
   * @param altitude Target altitude in metres above ground (default 13m)
   */
  void setTerrainFollowingAltitude(double altitude);
  
  /**
   * @brief Set emergency climb parameters for obstacle avoidance
   * @param detection_distance Distance ahead to scan for obstacles (default 5m)
   * @param climb_altitude Additional altitude to climb over obstacles (default 5m)
   */
  void setEmergencyClimbParams(double detection_distance, double climb_altitude);
  
  /**
   * @brief Execute takeoff maneuver using sonar feedback
   * @param target_altitude Target altitude in metres
   * @param sonar_range Current sonar reading in metres
   * @param elapsed_seconds Time elapsed since takeoff started
   * @param max_timeout_seconds Maximum time allowed for takeoff
   * @return True if takeoff complete (success or timeout), false if still in progress
   * 
   * Handles complete takeoff logic including velocity command publishing,
   * progress monitoring, and completion detection using sonar feedback.
   * Publishes climb commands until target altitude is reached.
   */
  bool takeoff(double target_altitude, double sonar_range, 
               double elapsed_seconds, double max_timeout_seconds = 10.0);
  
  /**
   * @brief Execute landing maneuver using sonar feedback
   * @param target_altitude Target landing altitude in metres (typically 0.2m)
   * @param sonar_range Current sonar reading in metres
   * @param elapsed_seconds Time elapsed since landing started
   * @param max_timeout_seconds Maximum time allowed for landing
   * @return True if landing complete (success or timeout), false if still in progress
   * 
   * Handles complete landing logic including velocity command publishing,
   * progress monitoring, and completion detection using sonar feedback.
   * Publishes descent commands until target altitude is reached.
   */
  bool land(double target_altitude, double sonar_range,
            double elapsed_seconds, double max_timeout_seconds = 20.0);
  
  /**
   * @brief Basic position control mode
   * @param current_pose Current drone pose
   * @param target_pose Desired target pose
   * @param dt Time step since last calculation  
   * @return Twist command for position tracking
   */
  geometry_msgs::msg::Twist calculatePositionControl(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose,
    double dt);
  
  /**
   * @brief Velocity control mode
   * @param current_velocity Current measured velocity
   * @param target_velocity Desired target velocity
   * @param dt Time step since last calculation
   * @return Twist command for velocity tracking
   */
  geometry_msgs::msg::Twist calculateVelocityControl(
    const geometry_msgs::msg::Twist& current_velocity,
    const geometry_msgs::msg::Twist& target_velocity,
    double dt);
  
  // Individual axis control methods (useful for testing and debugging)
  /**
   * @brief X-axis position control
   * @param target_x Desired X position
   * @param current_x Current X position
   * @param dt Time step
   * @return X velocity command
   */
  double calculateXControl(double target_x, double current_x, double dt);
  
  /**
   * @brief Y-axis position control  
   * @param target_y Desired Y position
   * @param current_y Current Y position
   * @param dt Time step
   * @return Y velocity command
   */
  double calculateYControl(double target_y, double current_y, double dt);
  
  /**
   * @brief Z-axis position control
   * @param target_z Desired Z position
   * @param current_z Current Z position  
   * @param dt Time step
   * @return Z velocity command
   */
  double calculateZControl(double target_z, double current_z, double dt);
  
  /**
   * @brief Update current yaw angle from IMU data
   * @param yaw Current yaw angle in radians
   */
  void updateCurrentYaw(double yaw);
  
  /**
   * @brief Calculate desired yaw angle towards target
   * @param current_pos Current position
   * @param target_pos Target position
   * @return Desired yaw angle in radians
   */
  double calculateYawToTarget(const geometry_msgs::msg::Point& current_pos, const geometry_msgs::msg::Point& target_pos);
  
  /**
   * @brief Calculate yaw control command
   * @param target_yaw Desired yaw angle
   * @param current_yaw Current yaw angle
   * @param dt Time step
   * @return Angular velocity command
   */
  double calculateYawControl(double target_yaw, double current_yaw, double dt);
  
private:
  // PID controllers for position control (X, Y, Z)
  std::unique_ptr<PIDController> pid_x_;
  std::unique_ptr<PIDController> pid_y_;
  std::unique_ptr<PIDController> pid_z_;
  
  // PID controllers for attitude control
  // std::unique_ptr<PIDController> pid_roll_;
  // std::unique_ptr<PIDController> pid_pitch_;
  std::unique_ptr<PIDController> pid_yaw_;  ///< Yaw orientation controller
  
  // Velocity PID controllers (for future implementation)
  // std::unique_ptr<PIDController> pid_vel_x_;
  // std::unique_ptr<PIDController> pid_vel_y_;
  // std::unique_ptr<PIDController> pid_vel_z_;
  
  // Physical parameters
  double drone_mass_;                 ///< Drone mass in kg
  double max_thrust_;                 ///< Maximum thrust in N
  double max_velocity_;               ///< Maximum linear velocity limit
  double max_angular_velocity_;       ///< Maximum angular velocity limit
  
  // Command smoothing
  geometry_msgs::msg::Twist last_cmd_;  ///< Previous command for smoothing
  bool first_command_;                  ///< First command flag
  
  // Orientation tracking
  double current_yaw_;                  ///< Current yaw angle in radians
  bool imu_data_available_;             ///< Flag indicating IMU data validity
  
  // ROS 2 integration
  rclcpp::Logger logger_;               ///< ROS 2 logger for debugging
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  ///< Command publisher
  
  // Altitude control parameters
  double terrain_following_altitude_;   ///< Target altitude above ground (metres)
  double obstacle_detection_distance_;  ///< Distance ahead to scan for obstacles (metres)
  double emergency_climb_altitude_;     ///< Additional altitude for obstacle avoidance (metres)
  bool emergency_climb_active_;         ///< Flag indicating if emergency climb is in progress
  std::chrono::steady_clock::time_point clearance_hold_start_; ///< When clearance altitude was reached
  bool clearance_hold_active_;          ///< Holding at clearance altitude before resuming
  double clearance_target_altitude_;    ///< Target altitude to hold during clearance
  
  /**
   * @brief Apply velocity and angular limits to command
   * @param cmd_vel Command to limit (modified in-place)
   */
  void applyControlLimits(geometry_msgs::msg::Twist& cmd_vel);
  
  /**
   * @brief Apply smoothing filter to reduce command jitter
   * @param cmd_vel Output command (modified in-place)
   * @param raw_cmd Raw unfiltered command
   * @param alpha Smoothing factor (0.0 = no change, 1.0 = no smoothing)
   */
  void applySmoothingFilter(geometry_msgs::msg::Twist& cmd_vel, const geometry_msgs::msg::Twist& raw_cmd, double alpha = 0.4);
  
  /**
   * @brief Scale command for basic velocity control
   * @param cmd_vel Command to scale (modified in-place)
   */
  void scaleForBasicVelocityControl(geometry_msgs::msg::Twist& cmd_vel);
  
  /**
   * @brief Check if goal position is reached
   * @param current_pos Current position
   * @param goal Target goal pose
   * @param use_manual_altitude Whether to check altitude
   * @return True if goal reached within tolerance
   */
  bool isGoalReached(const geometry_msgs::msg::Point& current_pos, const geometry_msgs::msg::Pose& goal, bool use_manual_altitude);
  
  /**
   * @brief Normalize angle to [-pi, pi] range
   * @param angle Angle in radians
   * @return Normalized angle
   */
  double normalizeAngle(double angle);
  
  /**
   * @brief Check for obstacles in forward direction using LiDAR
   * @param lidar_data Current LiDAR scan data
   * @return Minimum distance to obstacle in forward sector (infinity if none)
   */
  double checkForObstaclesAhead(const sensor_msgs::msg::LaserScan& lidar_data);
};

}  // namespace drone_swarm

#endif  // DRONE_SWARM_DRONE_CONTROL_H_