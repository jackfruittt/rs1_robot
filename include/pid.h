/**
 * @file pid.h
 * @brief Enhanced PID Controller with windup protection for drone control systems
 * @author RS1 Robot Team
 * @date 2025
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

namespace drone_swarm
{

/**
 * @class PIDController
 * @brief Enhanced PID Controller with advanced features for precise drone control
 * 
 * This PID controller includes windup protection, conditional integration,
 * and configurable limits for robust drone flight control applications.
 * Designed for high-frequency control loops.
 */
class PIDController
{
public:
  /**
   * @brief Default constructor
   * 
   * Initialises PID controller with default gains (all zeros).
   * Must call setGains() before use.
   */
  PIDController();
  
  /**
   * @brief Parameterised constructor
   * @param kp Proportional gain
   * @param ki Integral gain  
   * @param kd Derivative gain
   */
  PIDController(double kp, double ki, double kd);
  
  /**
   * @brief Set PID gains
   * @param kp Proportional gain - immediate response to current error
   * @param ki Integral gain - eliminates steady-state error
   * @param kd Derivative gain - dampens oscillations and improves stability
   */
  void setGains(double kp, double ki, double kd);
  
  /**
   * @brief Set output limits to prevent actuator saturation
   * @param min_output Minimum output value
   * @param max_output Maximum output value
   */
  void setLimits(double min_output, double max_output);
  
  /**
   * @brief Set integral term limits to prevent windup
   * @param min_integral Minimum integral accumulator value
   * @param max_integral Maximum integral accumulator value
   */
  void setIntegralLimits(double min_integral, double max_integral);
  
  /**
   * @brief Set threshold for conditional integration
   * @param threshold Only integrate when absolute error is below this value
   * 
   * This prevents integral windup during large transients while maintaining
   * steady-state error elimination for small errors.
   */
  void setIntegralThreshold(double threshold);
  
  /**
   * @brief Standard PID calculation
   * @param setpoint Desired value
   * @param process_variable Current measured value
   * @param dt Time step since last calculation (seconds)
   * @return Control output value
   */
  double calculate(double setpoint, double process_variable, double dt);
  
  /**
   * @brief Reset PID controller state
   * 
   * Clears integral accumulator, previous error, and resets first run flag.
   * Use when changing setpoints or after control interruptions.
   */
  void reset();
  
  /**
   * @brief Advanced PID calculation with enhanced windup protection
   * @param setpoint Desired value
   * @param process_variable Current measured value  
   * @param dt Time step since last calculation (seconds)
   * @param windup_threshold Threshold for windup protection (default: 2.0)
   * @return Control output value
   * 
   * This method includes advanced windup protection mechanisms
   * optimised for drone flight control applications.
   */
  double calculateWithWindupProtection(double setpoint, double process_variable, double dt, double windup_threshold = 2.0);
  
  // Getters for tuning and debugging
  /**
   * @brief Get current proportional term value
   * @return Current proportional contribution to output
   */
  double getProportional() const { return proportional_term_; }
  
  /**
   * @brief Get current integral term value
   * @return Current integral contribution to output
   */
  double getIntegral() const { return integral_term_; }
  
  /**
   * @brief Get current derivative term value
   * @return Current derivative contribution to output
   */
  double getDerivative() const { return derivative_term_; }
  
  /**
   * @brief Get last error value
   * @return Previous error for debugging
   */
  double getLastError() const { return previous_error_; }
  
  /**
   * @brief Get integral accumulator value
   * @return Current integral accumulator for debugging
   */
  double getIntegralAccumulator() const { return integral_; }

private:
  // PID gains
  double kp_, ki_, kd_;
  
  // Output and integral limits
  double min_output_, max_output_;
  double min_integral_, max_integral_;
  double integral_threshold_;  ///< Only integrate when error is below this threshold
  
  // State variables
  double previous_error_;
  double integral_;
  double proportional_term_, integral_term_, derivative_term_;
  bool first_run_;
  
  /**
   * @brief Clamp integral accumulator within limits
   */
  void clampIntegral();
};

}  // namespace drone_swarm

#endif  // PID_CONTROLLER_H_