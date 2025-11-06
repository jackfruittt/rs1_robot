#include "drone_control.h"
#include <algorithm>
#include <cmath>
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace drone_swarm
{

  DroneControl::DroneControl() 
    : drone_mass_(1.5), max_thrust_(8.0), max_velocity_(8.0), max_angular_velocity_(1.0),
      first_command_(true), current_yaw_(0.0), imu_data_available_(false),
      logger_(rclcpp::get_logger("drone_control")),
      terrain_following_altitude_(5.0), obstacle_detection_distance_(5.0), 
      emergency_climb_altitude_(5.0), emergency_climb_active_(false),
      clearance_hold_active_(false), clearance_target_altitude_(0.0),
      panic_climb_active_(false)
  {
    // Initialise PID controllers with optimised gains from flyToGoal testing
    pid_x_ = std::make_unique<PIDController>(0.8, 0.05, 0.15);  // Horizontal X control
    pid_y_ = std::make_unique<PIDController>(0.8, 0.05, 0.15);  // Horizontal Y control
    pid_z_ = std::make_unique<PIDController>(1.2, 0.05, 0.2);   // Vertical Z control - aggressive altitude correction
    pid_yaw_ = std::make_unique<PIDController>(2.0, 0.1, 0.3);  // Yaw orientation control
    
    // Configure integral windup protection for stable flight
    pid_x_->setIntegralLimits(-1.0, 1.0);
    pid_y_->setIntegralLimits(-1.0, 1.0);
    pid_z_->setIntegralLimits(-0.5, 0.5);  // More conservative for altitude
    pid_yaw_->setIntegralLimits(-0.5, 0.5);  // Conservative yaw integral
    
    // Initialise command smoothing history
    last_cmd_.linear.x = last_cmd_.linear.y = last_cmd_.linear.z = 0.0;
    last_cmd_.angular.x = last_cmd_.angular.y = last_cmd_.angular.z = 0.0;
  }

  void DroneControl::initialise(double mass, double max_thrust) {
    // Configure drone physical parameters for flight dynamics
    drone_mass_ = mass;
    max_thrust_ = max_thrust;
  }

  void DroneControl::setLogger(rclcpp::Logger logger) {
    // Update logger instance for debug and error reporting
    logger_ = logger;
  }

  void DroneControl::setCmdVelPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub) {
    // Set publisher for velocity commands to actuators
    cmd_vel_pub_ = pub;
  }

  void DroneControl::updatePositionPIDGains(double kp, double ki, double kd) {
    // Update position control gains for all horizontal and vertical controllers
    pid_x_->setGains(kp, ki, kd);
    pid_y_->setGains(kp, ki, kd);
    pid_z_->setGains(kp, ki, kd);
  }

  void DroneControl::setControlLimits(double max_velocity, double max_angular_velocity) {
    // Configure velocity limits for safe operation
    max_velocity_ = max_velocity;
    max_angular_velocity_ = max_angular_velocity;
  }

  void DroneControl::resetControllers() {
    // Reset all PID controllers for clean mission restart
    pid_x_->reset();
    pid_y_->reset();
    pid_z_->reset();
    pid_yaw_->reset();
    
    first_command_ = true;  // Reset smoothing filter
    imu_data_available_ = false;
  }

  geometry_msgs::msg::Twist DroneControl::calculateAdvancedPositionControl(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose,
    double dt,
    bool use_manual_altitude) {
    
      // Validate delta time input
    const double DEFAULT_DT = 0.1;
    if (dt <= 0.0) dt = DEFAULT_DT;
    
    // Calculate position errors for distance-based control scaling
    double error_x = target_pose.pose.position.x - current_pose.pose.position.x;
    double error_y = target_pose.pose.position.y - current_pose.pose.position.y;
    double horizontal_distance = std::sqrt(error_x*error_x + error_y*error_y);
    
    geometry_msgs::msg::Twist raw_cmd;
    
    // Forward flight control using yaw orientation
    if (imu_data_available_ && horizontal_distance > 0.3) {
      // Calculate desired yaw to face target
      double target_yaw = calculateYawToTarget(current_pose.pose.position, target_pose.pose.position);
      
      // Calculate yaw control command
      raw_cmd.angular.z = calculateYawControl(target_yaw, current_yaw_, dt);
      
      // Calculate forward velocity based on distance and orientation accuracy
      double yaw_error = normalizeAngle(target_yaw - current_yaw_);
      double orientation_accuracy = std::cos(yaw_error);  // 1.0 when perfectly aligned, 0.0 when perpendicular
      
      // Adaptive speed ramping based on distance to target
      double max_forward_vel; // Zoom
      if (horizontal_distance < 5.0) {
        max_forward_vel = 3.0;  // Higher speed when far from target
      } else if (horizontal_distance < 1.0) {
        max_forward_vel = 1.0;  // Medium speed when approaching target
      } else {
        max_forward_vel = 8.0;  // Slow speed for precision when close
      }
      
      // Forward velocity reduced by orientation error
      raw_cmd.linear.x = max_forward_vel * std::min(1.0, horizontal_distance / 1.0) * std::max(0.2, orientation_accuracy);
      raw_cmd.linear.y = 0.0;  // No lateral strafing in forward flight mode
      
      // Small position correction for fine positioning when close
      if (horizontal_distance < 1.0) {
        // Add small lateral corrections when close to target
        raw_cmd.linear.y = pid_y_->calculateWithWindupProtection(target_pose.pose.position.y, current_pose.pose.position.y, dt, 1.0) * 0.3;
      }
      
    } else {
      // Fallback to direct position control when IMU unavailable or very close
      raw_cmd.linear.x = pid_x_->calculateWithWindupProtection(target_pose.pose.position.x, current_pose.pose.position.x, dt, 2.0);
      raw_cmd.linear.y = pid_y_->calculateWithWindupProtection(target_pose.pose.position.y, current_pose.pose.position.y, dt, 2.0);
      raw_cmd.angular.z = 0.0;
      
      // Adaptive speed ramping for fallback mode
      double max_horizontal_vel = 10.0;
      if (horizontal_distance > 5.0) {
        max_horizontal_vel = 7.0;
      } else if (horizontal_distance > 1.0) {
        max_horizontal_vel = 1.5;
      } else {
        max_horizontal_vel = 1.0;
      }
      
      // Apply velocity limits with proportional scaling to maintain direction
      double cmd_magnitude = std::sqrt(raw_cmd.linear.x*raw_cmd.linear.x + raw_cmd.linear.y*raw_cmd.linear.y);
      if (cmd_magnitude > max_horizontal_vel) {
        raw_cmd.linear.x = (raw_cmd.linear.x / cmd_magnitude) * max_horizontal_vel;
        raw_cmd.linear.y = (raw_cmd.linear.y / cmd_magnitude) * max_horizontal_vel;
      }
    }
    
    // Altitude control with manual or terrain following modes
    if (use_manual_altitude) {
      // Manual altitude control with windup protection
      raw_cmd.linear.z = pid_z_->calculateWithWindupProtection(target_pose.pose.position.z, current_pose.pose.position.z, dt, 1.0);
      raw_cmd.linear.z = std::clamp(raw_cmd.linear.z, -0.3, 0.3);  // Conservative Z velocity limits
    } else {
      // Terrain following mode - future implementation
      raw_cmd.linear.z = 0.0;  // No terrain following yet
      
      // Safety altitude ceiling to prevent flyaway
      if (current_pose.pose.position.z > 8.0) {
        raw_cmd.linear.z = -0.1;  // Gentle descent if too high
      }
    }
    
    // Apply smoothing filter to reduce control jitter
    geometry_msgs::msg::Twist cmd_vel;
    applySmoothingFilter(cmd_vel, raw_cmd);
    
    // Keep roll and pitch zero for stability, allow yaw control
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    // cmd_vel.angular.z is preserved from raw_cmd for yaw control
    
    // Final scaling for compatibility with basic velocity control
    scaleForBasicVelocityControl(cmd_vel);
    
    return cmd_vel;
  }

  bool DroneControl::flyToGoal(VehicleData& vehicle, const geometry_msgs::msg::Pose& goal) {
    
    // Determine control mode based on goal altitude specification
    bool use_manual_altitude = (goal.position.z > 0.1);
    
    // Log mission start with appropriate mode
    if (use_manual_altitude) {
      RCLCPP_INFO(logger_, "Flying %s to goal: [%.2f, %.2f, %.2f] (MANUAL ALTITUDE)",
                  vehicle.id.c_str(), goal.position.x, goal.position.y, goal.position.z);
    } else {
      RCLCPP_INFO(logger_, "Flying %s to goal: [%.2f, %.2f, Z=terrain] (TERRAIN FOLLOWING)",
                  vehicle.id.c_str(), goal.position.x, goal.position.y);
    }

    rclcpp::Rate rate(10);  // 10 Hz control loop for smooth flight
    int max_attempts = 1800; // Maximum 3 minutes flight time for safety
    
    // Reset controllers for clean goal approach
    resetControllers();
    
    // Main control loop for goal navigation
    for (int i = 0; i < max_attempts; i++)
    {
      // Check if mission has been cancelled
      if (!vehicle.mission_active) {
        return false;
      }

      // Get current position with thread safety
      geometry_msgs::msg::Point current_pos;
      {
        std::lock_guard<std::mutex> lock(vehicle.odom_mutex);
        current_pos = vehicle.current_odom.pose.pose.position;
      }

      // Check if goal has been reached within tolerance
      if (isGoalReached(current_pos, goal, use_manual_altitude)) {
        RCLCPP_INFO(logger_, "Goal reached for %s", vehicle.id.c_str());
        
        // Execute stabilisation hover sequence
        geometry_msgs::msg::Twist hover_cmd;
        hover_cmd.linear.x = 0.0;
        hover_cmd.linear.y = 0.0;
        hover_cmd.angular.x = hover_cmd.angular.y = hover_cmd.angular.z = 0.0;
        
        // Fine altitude adjustment during hover
        if (use_manual_altitude) {
          double altitude_error = goal.position.z - current_pos.z;
          hover_cmd.linear.z = altitude_error * 0.15;  // Gentle altitude correction
          hover_cmd.linear.z = std::clamp(hover_cmd.linear.z, -0.1, 0.1);
        } else {
          hover_cmd.linear.z = 0.0;  // Hold current altitude
        }
        
        // Stabilisation period for precise positioning
        for (int hover_count = 0; hover_count < 15; hover_count++) {
          if (cmd_vel_pub_) {
            cmd_vel_pub_->publish(hover_cmd);
          }
          rate.sleep();
        }
        
        return true;  // Successfully reached goal
      }

      // Calculate control command using advanced position control
      geometry_msgs::msg::PoseStamped current_pose_stamped;
      current_pose_stamped.pose.position = current_pos;
      
      geometry_msgs::msg::PoseStamped target_pose_stamped;
      target_pose_stamped.pose = goal;
      
      geometry_msgs::msg::Twist cmd_vel = calculateAdvancedPositionControl(
        current_pose_stamped, target_pose_stamped, 0.1, use_manual_altitude);
      
      // Publish control command to actuators
      if (cmd_vel_pub_) {
        cmd_vel_pub_->publish(cmd_vel);
      }

      // Periodic debug output for monitoring flight progress
      if (i % 20 == 0) {  // Every 2 seconds
        double horizontal_distance = std::sqrt(
          std::pow(goal.position.x - current_pos.x, 2) + 
          std::pow(goal.position.y - current_pos.y, 2));
        
        if (use_manual_altitude) {
          RCLCPP_INFO(logger_, "PID: dist=%.2fm, alt_err=%.2fm, cmd=[%.2f,%.2f,%.2f]", 
                    horizontal_distance, goal.position.z - current_pos.z,
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
        } else {
          RCLCPP_INFO(logger_, "PID: dist=%.2fm, alt=%.2fm, cmd=[%.2f,%.2f,%.2f]", 
                    horizontal_distance, current_pos.z,
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
        }
      }

      rate.sleep();
    }

    RCLCPP_ERROR(logger_, "Failed to reach goal within time limit");
    return false;
  }

  // Utility methods
  void DroneControl::applySmoothingFilter(geometry_msgs::msg::Twist& cmd_vel, const geometry_msgs::msg::Twist& raw_cmd, double alpha) {
    
    if (first_command_) {
      cmd_vel = raw_cmd;
      first_command_ = false;
    } else {
      cmd_vel.linear.x = alpha * raw_cmd.linear.x + (1.0 - alpha) * last_cmd_.linear.x;
      cmd_vel.linear.y = alpha * raw_cmd.linear.y + (1.0 - alpha) * last_cmd_.linear.y;
      cmd_vel.linear.z = alpha * raw_cmd.linear.z + (1.0 - alpha) * last_cmd_.linear.z;
      cmd_vel.angular.x = alpha * raw_cmd.angular.x + (1.0 - alpha) * last_cmd_.angular.x;
      cmd_vel.angular.y = alpha * raw_cmd.angular.y + (1.0 - alpha) * last_cmd_.angular.y;
      cmd_vel.angular.z = alpha * raw_cmd.angular.z + (1.0 - alpha) * last_cmd_.angular.z;
    }
    
    last_cmd_ = cmd_vel;
  }

  void DroneControl::scaleForBasicVelocityControl(geometry_msgs::msg::Twist& cmd_vel) {
    cmd_vel.linear.x *= 0.8;
    cmd_vel.linear.y *= 0.8;
    cmd_vel.linear.z *= 0.9;
  }

  bool DroneControl::isGoalReached(const geometry_msgs::msg::Point& current_pos, const geometry_msgs::msg::Pose& goal, bool use_manual_altitude) {
    double error_x = goal.position.x - current_pos.x;
    double error_y = goal.position.y - current_pos.y;
    double horizontal_distance = std::sqrt(error_x*error_x + error_y*error_y);
    
    if (use_manual_altitude) {
      double altitude_error = std::abs(goal.position.z - current_pos.z);
      return (horizontal_distance < 0.8 && altitude_error < 0.5);
    } else {
      return (horizontal_distance < 0.8);
    }
  }

  void DroneControl::applyControlLimits(geometry_msgs::msg::Twist& cmd_vel) {
    // Apply velocity limits
    cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -max_velocity_, max_velocity_);
    cmd_vel.linear.y = std::clamp(cmd_vel.linear.y, -max_velocity_, max_velocity_);
    cmd_vel.linear.z = std::clamp(cmd_vel.linear.z, -max_velocity_, max_velocity_);
    
    cmd_vel.angular.x = std::clamp(cmd_vel.angular.x, -max_angular_velocity_, max_angular_velocity_);
    cmd_vel.angular.y = std::clamp(cmd_vel.angular.y, -max_angular_velocity_, max_angular_velocity_);
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_velocity_, max_angular_velocity_);
  }

  // Template implementations for remaining methods (to maintain compatibility)
  geometry_msgs::msg::Twist DroneControl::calculateControlOutput(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    double dt) {
    // Use the advanced position control as default
    (void)current_velocity;  // Suppress unused parameter warning
    return calculateAdvancedPositionControl(current_pose, target_pose, dt);
  }

  geometry_msgs::msg::Twist DroneControl::calculatePositionControl(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose,
    double dt) {
    return calculateAdvancedPositionControl(current_pose, target_pose, dt);
  }

  double DroneControl::calculateXControl(double target_x, double current_x, double dt) {
    return pid_x_->calculate(target_x, current_x, dt);
  }

  double DroneControl::calculateYControl(double target_y, double current_y, double dt) {
    return pid_y_->calculate(target_y, current_y, dt);
  }

  double DroneControl::calculateZControl(double target_z, double current_z, double dt) {
    return pid_z_->calculate(target_z, current_z, dt);
  }

  void DroneControl::updateCurrentYaw(double yaw) {
    current_yaw_ = normalizeAngle(yaw);
    imu_data_available_ = true;
  }

  double DroneControl::calculateYawToTarget(const geometry_msgs::msg::Point& current_pos, const geometry_msgs::msg::Point& target_pos) {
    double dx = target_pos.x - current_pos.x;
    double dy = target_pos.y - current_pos.y;
    return std::atan2(dy, dx);  // Returns angle in [-pi, pi]
  }

  double DroneControl::calculateYawControl(double target_yaw, double current_yaw, double dt) {
    // Normalize the yaw error to handle wraparound
    double yaw_error = normalizeAngle(target_yaw - current_yaw);
    return pid_yaw_->calculate(0.0, -yaw_error, dt);  // Negative error for proper direction
  }

  double DroneControl::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  bool DroneControl::takeoff(double target_altitude, double sonar_range, 
                             double elapsed_seconds, double max_timeout_seconds) {
    // Track initial altitude to implement relative climb behavior
    static double initial_sonar_reading = -1.0;
    static double computed_target_altitude = -1.0;
    
    // On first call, record starting altitude and compute absolute target
    if (initial_sonar_reading < 0.0) {
      initial_sonar_reading = sonar_range;
      computed_target_altitude = initial_sonar_reading + target_altitude;
      RCLCPP_INFO(logger_, "Takeoff initiated: current=%.2fm, climbing %.2fm to %.2fm", 
                  initial_sonar_reading, target_altitude, computed_target_altitude);
    }
    
    // Timeout check
    if (elapsed_seconds > max_timeout_seconds) {
      RCLCPP_WARN(logger_, "Takeoff timeout - reached %.2fm (wanted %.2fm, climbed %.2fm)", 
                  sonar_range, computed_target_altitude, sonar_range - initial_sonar_reading);
      
      // Stop climbing and reset state
      if (cmd_vel_pub_) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
      }
      initial_sonar_reading = -1.0;
      computed_target_altitude = -1.0;
      return true; // Complete due to timeout
    }
    
    // Check if target altitude reached
    if (sonar_range >= computed_target_altitude - 0.2) {
      RCLCPP_INFO(logger_, "Takeoff completed - reached %.2fm (climbed %.2fm from %.2fm)", 
                  sonar_range, sonar_range - initial_sonar_reading, initial_sonar_reading);
      
      // Stop climbing and reset state
      if (cmd_vel_pub_) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
      }
      initial_sonar_reading = -1.0;
      computed_target_altitude = -1.0;
      return true; // Successfully completed
    }
    
    // Continue climbing - send climb commands
    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.z = 5.0; // 5 m/s upward velocity
      cmd_vel_pub_->publish(cmd_vel);
    }
    
    // Periodic progress logging (every ~2 seconds if called at 10Hz)
    static int takeoff_log_counter = 0;
    if (takeoff_log_counter % 20 == 0) {
      RCLCPP_INFO(logger_, "Takeoff progress: %.2fm / %.2fm (climbed %.2fm)", 
                  sonar_range, computed_target_altitude, sonar_range - initial_sonar_reading);
    }
    takeoff_log_counter++;
    
    // Still in progress
    return false;
  }

  bool DroneControl::land(double target_altitude, double sonar_range,
                          double elapsed_seconds, double max_timeout_seconds) {
    // Timeout check
    if (elapsed_seconds > max_timeout_seconds) {
      RCLCPP_WARN(logger_, "Landing timeout - reached %.2fm (wanted %.2fm)", 
                  sonar_range, target_altitude);
      
      // Stop descending
      if (cmd_vel_pub_) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
      }
      return true; // Complete due to timeout
    }
    
    // Check if target altitude reached (or if sonar indicates ground proximity)
    // Note: sonar may return ~5m when very close to ground due to minimum range limits
    if (sonar_range <= target_altitude || sonar_range >= 4.5) {
      RCLCPP_INFO(logger_, "Landing completed - reached %.2fm", sonar_range);
      
      // Stop descending
      if (cmd_vel_pub_) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
      }
      return true; // Successfully completed
    }
    
    // Continue descending - send descent commands
    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.z = -5.0; // 5 m/s downward velocity
      cmd_vel_pub_->publish(cmd_vel);
    }
    
    // Periodic progress logging (every ~2 seconds if called at 10Hz)
    static int landing_log_counter = 0;
    if (landing_log_counter % 20 == 0) {
      RCLCPP_INFO(logger_, "Landing progress: %.2fm (target: %.2fm)", 
                  sonar_range, target_altitude);
    }
    landing_log_counter++;
    
    // Still in progress
    return false;
  }

  geometry_msgs::msg::Twist DroneControl::navigateToWaypointWithAltitudeControl(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_waypoint,
    double sonar_range,
    const sensor_msgs::msg::LaserScan& lidar_data,
    double dt,
    bool allow_descent) {
    
    // Validate delta time input
    const double DEFAULT_DT = 0.1;
    if (dt <= 0.0) dt = DEFAULT_DT;
    
    // Create modified target that ignores Z component for horizontal navigation
    geometry_msgs::msg::PoseStamped horizontal_target = target_waypoint;
    horizontal_target.pose.position.z = current_pose.pose.position.z; // Use current Z for horizontal planning
    
    // Calculate horizontal control using existing position control (without altitude)
    geometry_msgs::msg::Twist horizontal_cmd = calculateAdvancedPositionControl(
        current_pose, horizontal_target, dt, false); // false = ignore altitude control
    
    // Check for obstacles ahead using LiDAR
    double min_obstacle_distance = checkForObstaclesAhead(lidar_data);
    
    // Determine target altitude and forward motion based on obstacle detection
    double target_altitude = terrain_following_altitude_; // Default 5m above ground
    const double MAX_EMERGENCY_ALTITUDE = 50.0; // Maximum altitude limit for safety
    const double OBSTACLE_SLOWDOWN_FACTOR = 0.05; // Reduce speed to 5% when obstacle detected
    const double EXTRA_CLIMB_AFTER_CLEAR = 1.5; // Climb extra 1.5m after clearing obstacle
    const double PANIC_CLIMB_TIMEOUT = 2.0; // Enter panic climb after 2 seconds of emergency climb
    const double PANIC_CLIMB_SPEED = 10.0; // Vertical speed for panic climb (m/s)
    
    double speed_multiplier = 1.0; // By default, full speed
    
    // If descent is allowed (e.g., landing at depot), use a low target altitude to force descent
    if (allow_descent) {
      target_altitude = 0.5;  // Target very low altitude (0.5m) to force descent
      RCLCPP_DEBUG(logger_, "Descent mode active - targeting %.2fm altitude (current: %.2fm)", 
                   target_altitude, sonar_range);
    }
    else if (min_obstacle_distance < obstacle_detection_distance_ && min_obstacle_distance > 0.1) {
      // Obstacle detected - determine response based on proximity
      // Very close obstacles (<2m): STOP forward motion entirely, climb only
      // Medium distance (2-5m): Slow to 5%, keep climbing
      const double CRITICAL_DISTANCE = 2.0; // Stop forward motion if closer than 2m
      
      if (min_obstacle_distance < CRITICAL_DISTANCE) {
        speed_multiplier = 0.0; // STOP all horizontal movement
      } else {
        speed_multiplier = OBSTACLE_SLOWDOWN_FACTOR; // Slow to 5%
      }
      
      if (!emergency_climb_active_) {
        // Just detected obstacle - start emergency climb and timer
        clearance_target_altitude_ = sonar_range + emergency_climb_altitude_;
        clearance_target_altitude_ = std::min(clearance_target_altitude_, MAX_EMERGENCY_ALTITUDE);
        emergency_climb_active_ = true;
        emergency_climb_start_ = std::chrono::steady_clock::now();
        panic_climb_active_ = false;
        clearance_hold_active_ = false; // Reset the post-clear climb flag
        
        if (speed_multiplier == 0.0) {
          RCLCPP_WARN(logger_, "CRITICAL: Obstacle %.2fm ahead - STOPPING forward motion, CLIMBING to %.2fm (current: %.2fm)", 
                      min_obstacle_distance, clearance_target_altitude_, sonar_range);
        } else {
          RCLCPP_WARN(logger_, "Obstacle detected %.2fm ahead - SLOWING to %.0f%% speed, climbing to %.2fm (current: %.2fm)", 
                      min_obstacle_distance, speed_multiplier * 100.0, clearance_target_altitude_, sonar_range);
        }
      }
      
      // Check if we've been climbing for too long (tall obstacle) - enter PANIC CLIMB
      auto climb_duration = std::chrono::steady_clock::now() - emergency_climb_start_;
      double climb_seconds = std::chrono::duration<double>(climb_duration).count();
      
      if (climb_seconds > PANIC_CLIMB_TIMEOUT && !panic_climb_active_) {
        panic_climb_active_ = true;
        // Add 2m safety margin above current altitude for tall obstacles
        clearance_target_altitude_ = sonar_range + 2.0; // Just 2m above current position
        clearance_target_altitude_ = std::min(clearance_target_altitude_, MAX_EMERGENCY_ALTITUDE);
        RCLCPP_ERROR(logger_, "PANIC CLIMB ACTIVATED! Tall obstacle detected (%.1fs climbing). Ascending FAST to %.2fm!", 
                     climb_seconds, clearance_target_altitude_);
      }
      
      target_altitude = clearance_target_altitude_;
      
      // Keep climbing if we haven't reached clearance altitude yet
      if (sonar_range < clearance_target_altitude_ - 0.5) {
        RCLCPP_DEBUG(logger_, "Climbing to clearance: %.2fm / %.2fm (obstacle at %.2fm)", 
                     sonar_range, clearance_target_altitude_, min_obstacle_distance);
      }
      
    } else if (emergency_climb_active_ && min_obstacle_distance > obstacle_detection_distance_ * 1.2) {
      // Obstacle just cleared - continue climbing extra before resuming normal operations
      // Reduced threshold from 1.5 to 1.2 (6m instead of 7.5m) for faster recovery
      if (!clearance_hold_active_) {
        // First time clearing - add extra climb height
        clearance_target_altitude_ = sonar_range + EXTRA_CLIMB_AFTER_CLEAR;
        clearance_target_altitude_ = std::min(clearance_target_altitude_, MAX_EMERGENCY_ALTITUDE);
        clearance_hold_active_ = true;
        RCLCPP_INFO(logger_, "Obstacle cleared! Climbing extra %.2fm to %.2fm for safety - resuming forward speed", 
                    EXTRA_CLIMB_AFTER_CLEAR, clearance_target_altitude_);
      }
      
      // Check if we've completed the extra climb
      if (sonar_range >= clearance_target_altitude_ - 0.5) {
        // Extra climb complete - now return to normal
        emergency_climb_active_ = false;
        clearance_hold_active_ = false;
        panic_climb_active_ = false;
        target_altitude = terrain_following_altitude_;
        speed_multiplier = 1.0;
        RCLCPP_INFO(logger_, "Extra climb complete at %.2fm - continuing at full speed", sonar_range);
      } else {
        // Still doing extra climb - resume normal forward speed while climbing
        target_altitude = clearance_target_altitude_;
        speed_multiplier = 1.0; // Full forward speed during extra climb
        RCLCPP_DEBUG(logger_, "Extra climb in progress: %.2fm / %.2fm (full forward speed)", sonar_range, clearance_target_altitude_);
      }
      
    } else if (emergency_climb_active_) {
      // Still in emergency climb zone - maintain clearance altitude and slow speed
      target_altitude = clearance_target_altitude_;
      speed_multiplier = OBSTACLE_SLOWDOWN_FACTOR;
    }
    
    // Calculate altitude control based on sonar reading
    double altitude_error = target_altitude - sonar_range;
    double altitude_command = pid_z_->calculateWithWindupProtection(target_altitude, sonar_range, dt, 1.0);
    
    // Apply panic climb speed if activated (overrides PID for tall obstacles)
    if (panic_climb_active_) {
      altitude_command = PANIC_CLIMB_SPEED; // Force ridiculous climb rate
      RCLCPP_DEBUG(logger_, "PANIC CLIMB: Forcing vertical speed to %.1f m/s (altitude: %.2fm / %.2fm)",
                   altitude_command, sonar_range, target_altitude);
    } else {
      // Conservative altitude velocity limits for safety during normal operation
      altitude_command = std::clamp(altitude_command, -5.0, 5.0); // +-5m/s max vertical velocity
    }
    
    // Combine horizontal and vertical commands
    geometry_msgs::msg::Twist cmd_vel = horizontal_cmd;
    
    // Apply speed reduction if obstacle detected
    cmd_vel.linear.x *= speed_multiplier;
    cmd_vel.linear.y *= speed_multiplier;
    cmd_vel.angular.z *= speed_multiplier; // Also slow rotation
    
    cmd_vel.linear.z = altitude_command;
    
    // Periodic debug output for altitude control monitoring
    static int altitude_log_counter = 0;
    if (altitude_log_counter % 50 == 0) { // Every 5 seconds at 10Hz
      RCLCPP_DEBUG(logger_, "Altitude control: sonar=%.2fm target=%.2fm error=%.2fm cmd_z=%.2f%s", 
                   sonar_range, target_altitude, altitude_error, altitude_command,
                   emergency_climb_active_ ? " [EMERGENCY CLIMB]" : "");
    }
    altitude_log_counter++;
    
    return cmd_vel;
  }

  void DroneControl::setEmergencyClimbParams(double detection_distance, double climb_altitude) {
    obstacle_detection_distance_ = std::max(2.0, detection_distance); // Minimum 2m detection
    emergency_climb_altitude_ = std::max(2.0, climb_altitude); // Minimum 2m climb
    RCLCPP_INFO(logger_, "Emergency climb parameters: detect=%.2fm climb=%.2fm", 
                obstacle_detection_distance_, emergency_climb_altitude_);
  }

  double DroneControl::checkForObstaclesAhead(const sensor_msgs::msg::LaserScan& lidar_data) {
    if (lidar_data.ranges.empty()) {
      return std::numeric_limits<double>::infinity(); // No data = no obstacles
    }
    
    // Calculate forward-facing LiDAR indices (0deg is forward)
    int total_rays = static_cast<int>(lidar_data.ranges.size());
    double angle_increment = lidar_data.angle_increment;
    double angle_min = lidar_data.angle_min;
    
    // Check forward sector (30-deg from forward direction)
    double forward_sector_half_angle = M_PI / 6.0; // 30 degrees
    
    double min_distance = std::numeric_limits<double>::infinity();
    int valid_readings = 0;
    
    for (int i = 0; i < total_rays; ++i) {
      double angle = angle_min + i * angle_increment;
      
      // Normalize angle to [-π, π]
      angle = normalizeAngle(angle);
      
      // Check if ray is in forward-facing sector
      if (std::abs(angle) <= forward_sector_half_angle) {
        double range = lidar_data.ranges[i];
        
        // Filter out invalid readings
        if (range >= lidar_data.range_min && range <= lidar_data.range_max && 
            !std::isinf(range) && !std::isnan(range)) {
          min_distance = std::min(min_distance, static_cast<double>(range));
          valid_readings++;
        }
      }
    }
    
    // Return infinity if no valid readings in forward sector
    if (valid_readings == 0) {
      return std::numeric_limits<double>::infinity();
    }
    
    return min_distance;
  }

}  // namespace drone_swarm