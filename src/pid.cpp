#include "pid.h"
#include <algorithm>
#include <cmath>

namespace drone_swarm
{

  PIDController::PIDController() 
    : kp_(0.0), ki_(0.0), kd_(0.0),
      min_output_(-1e6), max_output_(1e6),
      min_integral_(-1.0), max_integral_(1.0),
      integral_threshold_(2.0),
      previous_error_(0.0), integral_(0.0),
      proportional_term_(0.0), integral_term_(0.0), derivative_term_(0.0),
      first_run_(true)
  {
    // Default constructor initialises PID with zero gains and conservative limits
  }

  PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
      min_output_(-1e6), max_output_(1e6),
      min_integral_(-1.0), max_integral_(1.0),
      integral_threshold_(2.0),
      previous_error_(0.0), integral_(0.0),
      proportional_term_(0.0), integral_term_(0.0), derivative_term_(0.0),
      first_run_(true)
  {
    // Parameterised constructor with specified PID gains
  }

  void PIDController::setGains(double kp, double ki, double kd) {
    // Update PID gains for tuning control performance
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void PIDController::setLimits(double min_output, double max_output) {
    // Configure output saturation limits to prevent actuator damage
    min_output_ = min_output;
    max_output_ = max_output;
  }

  void PIDController::setIntegralLimits(double min_integral, double max_integral) {
    // Set integral windup protection limits for stable control
    min_integral_ = min_integral;
    max_integral_ = max_integral;
  }

  void PIDController::setIntegralThreshold(double threshold) {
    // Set error threshold for conditional integral accumulation
    integral_threshold_ = threshold;
  }

  double PIDController::calculate(double setpoint, double process_variable, double dt) {
    // Calculate control error
    double error = setpoint - process_variable;
    
    // Proportional term: immediate response to current error
    proportional_term_ = kp_ * error;
    
    // Integral term: accumulate error over time for steady-state accuracy
    integral_ += error * dt;
    clampIntegral();  // Apply windup protection
    integral_term_ = ki_ * integral_;
    
    // Derivative term: predict future error based on rate of change
    if (first_run_) {
      derivative_term_ = 0.0;  // No derivative on first iteration
      first_run_ = false;
    } else {
      derivative_term_ = kd_ * (error - previous_error_) / dt;
    }
    
    // Store error for next derivative calculation
    previous_error_ = error;
    
    // Calculate total control output
    double output = proportional_term_ + integral_term_ + derivative_term_;
    
    // Apply output saturation limits for safety
    return std::clamp(output, min_output_, max_output_);
  }

  double PIDController::calculateWithWindupProtection(double setpoint, double process_variable, double dt, double windup_threshold) {
    // Calculate control error
    double error = setpoint - process_variable;
    
    // Proportional term: immediate response to current error
    proportional_term_ = kp_ * error;
    
    // Integral term with enhanced windup protection (derived from flyToGoal logic)
    // Only accumulate integral when error is small enough to prevent windup
    if (std::abs(error) < windup_threshold) {
      integral_ += error * dt;
      clampIntegral();  // Apply integral limits
    }
    integral_term_ = ki_ * integral_;
    
    // Derivative term: predict future error based on rate of change
    if (first_run_) {
      derivative_term_ = 0.0;  // No derivative on first iteration
      first_run_ = false;
    } else {
      derivative_term_ = kd_ * (error - previous_error_) / dt;
    }
    
    // Store error for next derivative calculation
    previous_error_ = error;
    
    // Calculate total control output
    double output = proportional_term_ + integral_term_ + derivative_term_;
    
    // Apply output saturation limits for safety
    return std::clamp(output, min_output_, max_output_);
  }

  void PIDController::reset() {
    // Reset all internal states for clean controller restart
    previous_error_ = 0.0;
    integral_ = 0.0;
    proportional_term_ = 0.0;
    integral_term_ = 0.0;
    derivative_term_ = 0.0;
    first_run_ = true;
  }

  void PIDController::clampIntegral() {
    // Apply integral windup protection by clamping accumulated error
    integral_ = std::clamp(integral_, min_integral_, max_integral_);
  }

}  // namespace drone_swarm