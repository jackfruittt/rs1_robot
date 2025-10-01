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
    : drone_mass_(1.5), max_thrust_(20.0), max_velocity_(5.0), max_angular_velocity_(1.0),
      first_command_(true), current_yaw_(0.0), imu_data_available_(false),
      logger_(rclcpp::get_logger("drone_control"))
  {
    // Initialise PID controllers with optimised gains from flyToGoal testing
    pid_x_ = std::make_unique<PIDController>(0.8, 0.05, 0.15);  // Horizontal X control
    pid_y_ = std::make_unique<PIDController>(0.8, 0.05, 0.15);  // Horizontal Y control
    pid_z_ = std::make_unique<PIDController>(0.3, 0.02, 0.1);   // Vertical Z control (more conservative)
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

  void DroneControl::updateVelocityPIDGains(double kp, double ki, double kd) {
    // Future implementation - velocity controllers not yet implemented
    (void)kp; (void)ki; (void)kd;
  }

  void DroneControl::updateAttitudePIDGains(double kp, double ki, double kd) {
    // Future implementation - attitude controllers not yet implemented
    (void)kp; (void)ki; (void)kd;
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
      double max_forward_vel = 2.0;
      if (horizontal_distance > 5.0) {
        max_forward_vel = 2.5;  // Higher speed when far from target
      } else if (horizontal_distance > 1.0) {
        max_forward_vel = 2.0;  // Medium speed when approaching target
      } else {
        max_forward_vel = 1.0;  // Slow speed for precision when close
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
      double max_horizontal_vel = 2.0;
      if (horizontal_distance > 5.0) {
        max_horizontal_vel = 2.0;
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

  // geometry_msgs::msg::Twist DroneControl::calculateVelocityControl(
  //   const geometry_msgs::msg::Twist& current_velocity,
  //   const geometry_msgs::msg::Twist& target_velocity,
  //   double dt)
  // {
  //   geometry_msgs::msg::Twist cmd_vel;
    
  //   // Simple feedforward velocity control (velocity PID controllers not yet implemented)
  //   cmd_vel.linear.x = target_velocity.linear.x;
  //   cmd_vel.linear.y = target_velocity.linear.y;
  //   cmd_vel.linear.z = target_velocity.linear.z;
    
  //   applyControlLimits(cmd_vel);
  //   return cmd_vel;
  // }

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

}  // namespace drone_swarm