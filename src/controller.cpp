#include "controller.h"

using namespace std::chrono_literals;

Controller::Controller(
    // Reveive publishers from drone_node, ensuring controller focuses only on controlling and not doing any of the ROS work
    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub,
    const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr &takeoff_pub,
    const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr &land_pub,
    const rclcpp::Logger &logger)
    : cmd_vel_pub_(cmd_vel_pub), takeoff_pub_(takeoff_pub), land_pub_(land_pub), logger_(logger),
      has_last_waypoint_(false)
{
}

// Process control commands for drone
bool Controller::processCommand(VehicleData &vehicle, const VehicleData::Command &command)
{
    switch (command.type)
    {
    case VehicleData::Command::Type::TAKEOFF:
        RCLCPP_INFO(logger_, "Processing TAKEOFF command for %s", vehicle.id.c_str());
        return takeoff(vehicle);

    case VehicleData::Command::Type::LANDING:
        RCLCPP_INFO(logger_, "Processing LAND command for %s", vehicle.id.c_str());
        return land(vehicle);

    case VehicleData::Command::Type::FLYING:
        RCLCPP_INFO(logger_, "Processing FLY_TO_GOAL command for %s to [%.2f, %.2f, %.2f]",
                    vehicle.id.c_str(), command.goal.position.x, command.goal.position.y, command.goal.position.z);
        return flyToGoal(vehicle, command.goal);

    case VehicleData::Command::Type::STOP:
        RCLCPP_INFO(logger_, "Processing STOP command for %s", vehicle.id.c_str());
        vehicle.mission_active = false;
        return true;

    default:
        RCLCPP_ERROR(logger_, "Unknown command type for %s", vehicle.id.c_str());
        return false;
    }
}

void Controller::startMission(VehicleData &vehicle)
{
    // Clear existing command queue
    {
        std::lock_guard<std::mutex> lock(vehicle.queue_mutex);
        std::queue<VehicleData::Command> empty;
        std::swap(vehicle.command_queue, empty);
    }

    // Set mission parameters
    vehicle.mission_active = true;
    vehicle.current_goal_index = 0;
    vehicle.mission_progress = 0.0;
    vehicle.takeoff_complete = false;

    // Add takeoff command
    {
        std::lock_guard<std::mutex> lock(vehicle.queue_mutex);
        vehicle.command_queue.push(VehicleData::Command(VehicleData::Command::Type::TAKEOFF));
    }

    // Add goals
    {
        std::lock_guard<std::mutex> lock(vehicle.goals_mutex);
        for (const auto &goal : vehicle.goals)
        {
            std::lock_guard<std::mutex> lock_queue(vehicle.queue_mutex);
            vehicle.command_queue.push(
                VehicleData::Command(VehicleData::Command::Type::FLYING, goal));
        }
    }

    // Let drone hover after completing last goal so it can do new goals if published
    // Landing will only happen when mission is explicitly stopped or new goals require it

    // Notify controller thread
    vehicle.queue_cv.notify_one();

    RCLCPP_INFO(logger_, "Mission started - drone will hover at final goal until new commands received");
}

void Controller::stopMission(VehicleData &vehicle)
{
    // Clear existing command queue
    {
        std::lock_guard<std::mutex> lock(vehicle.queue_mutex);
        std::queue<VehicleData::Command> empty;
        std::swap(vehicle.command_queue, empty);

        // Stop command
        vehicle.command_queue.push(VehicleData::Command(VehicleData::Command::Type::STOP));

        // Landing command
        vehicle.command_queue.push(VehicleData::Command(VehicleData::Command::Type::LANDING));
    }

    // Notify controller thread
    vehicle.queue_cv.notify_one();
}

bool Controller::takeoff(VehicleData &vehicle)
{
    RCLCPP_INFO(logger_, "Taking off %s...", vehicle.id.c_str());

    // Publish takeoff command - send multiple times to ensure it's received
    std_msgs::msg::Empty takeoff_msg;
    for (int i = 0; i < 5; i++) // 5 repetitions
    {
        takeoff_pub_->publish(takeoff_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // delay each message send to not overload
    }

    // Initial climb
    current_altitude_adjustment_ = 1.0; // 1m/s, increase if needs to be more aggressive

    // Log takeoff completion
    RCLCPP_INFO(logger_, "Takeoff initiated for %s", vehicle.id.c_str());
    vehicle.takeoff_complete = true;

    // Send direct climb command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.z = 1.0; // Force strong initial ascent
    cmd_vel_pub_->publish(cmd_vel);

    // Wait for drone to reach minimum safe height
    rclcpp::Rate rate(10); // 10 Hz
    int timeout = 100;     // 10 seconds timeout

    RCLCPP_INFO(logger_, "Waiting for drone to reach safe takeoff height...");

    for (int i = 0; i < timeout; i++)
    {
        double altitude = 0.0;
        {
            std::lock_guard<std::mutex> lock(vehicle.odom_mutex);
            altitude = vehicle.current_odom.pose.pose.position.z;
        }

        // Debug output at 1Hz
        if (i % 10 == 0)
        {
            RCLCPP_INFO(logger_, "Takeoff progress: %.2fm (target: 2.5m)", altitude);
        }

        // Periodically resend climb command
        if (i % 20 == 0)
        {
            cmd_vel.linear.z = 1.0;
            cmd_vel_pub_->publish(cmd_vel);
        }

        if (altitude > 2.5)
        {
            RCLCPP_INFO(logger_, "Drone reached safe takeoff height: %.2fm", altitude);
            break;
        }

        if (i == timeout - 1)
        {
            RCLCPP_WARN(logger_, "Takeoff timeout - drone reached %.2fm (wanted 2.5m)", altitude);
        }

        rate.sleep();
    }

    // Stabilise at takeoff height
    cmd_vel.linear.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);

    // Reset altitude adjustment to neutral
    current_altitude_adjustment_ = 0.0;

    return true;
}

bool Controller::land(VehicleData &vehicle)
{
    RCLCPP_INFO(logger_, "Landing %s...", vehicle.id.c_str());

    // Publish landing command
    std_msgs::msg::Empty land_msg;
    land_pub_->publish(land_msg);

    vehicle.takeoff_complete = false;
    return true;
}

bool Controller::flyToGoal(VehicleData& vehicle, const geometry_msgs::msg::Pose& goal) {
    // Check if this specific goal has altitude override (z > 0.1)
    bool use_manual_altitude = (goal.position.z > 0.1);
    
    if (use_manual_altitude) {
        RCLCPP_INFO(logger_, "Flying %s to goal: [%.2f, %.2f, %.2f] (MANUAL ALTITUDE)",
                    vehicle.id.c_str(), goal.position.x, goal.position.y, goal.position.z);
    } else {
        RCLCPP_INFO(logger_, "Flying %s to goal: [%.2f, %.2f, Z=terrain] (TERRAIN FOLLOWING)",
                    vehicle.id.c_str(), goal.position.x, goal.position.y);
    }

    rclcpp::Rate rate(10);  // 10 Hz
    int max_attempts = 1800; // Maximum 3 minutes
    
    // Static variables for velocity smoothing and PID control - ADDED FOR IGNITION COMPATIBILITY
    static geometry_msgs::msg::Twist last_cmd;
    static bool first_command = true;
    static double integral_x = 0.0, integral_y = 0.0, integral_z = 0.0;
    static double last_error_x = 0.0, last_error_y = 0.0, last_error_z = 0.0;
    
    // Reset for new goal - ADDED FOR IGNITION COMPATIBILITY
    if (first_command) {
        integral_x = integral_y = integral_z = 0.0;
        last_error_x = last_error_y = last_error_z = 0.0;
        first_command = false;
    }
    
    for (int i = 0; i < max_attempts; i++)
    {
        if (!vehicle.mission_active) {
            return false;
        }

        // Get current position
        geometry_msgs::msg::Point current_pos;
        {
            std::lock_guard<std::mutex> lock(vehicle.odom_mutex);
            current_pos = vehicle.current_odom.pose.pose.position;
        }

        // Calculate errors (position-based control) - MODIFIED FOR IGNITION
        double error_x = goal.position.x - current_pos.x;
        double error_y = goal.position.y - current_pos.y;
        double horizontal_distance = std::sqrt(error_x*error_x + error_y*error_y);

        // Check if goal reached with tighter tolerance - MODIFIED FOR IGNITION
        bool goal_reached = false;
        if (use_manual_altitude) {
            double altitude_error = std::abs(goal.position.z - current_pos.z);
            goal_reached = (horizontal_distance < 0.8 && altitude_error < 0.5);  // Relaxed tolerance
        } else {
            goal_reached = (horizontal_distance < 0.8);  // Relaxed tolerance
        }

        if (goal_reached) {
            RCLCPP_INFO(logger_, "Goal reached for %s (distance: %.2fm)", vehicle.id.c_str(), horizontal_distance);
            
            // Brief stabilisation hover
            geometry_msgs::msg::Twist hover_cmd;
            hover_cmd.linear.x = 0.0;
            hover_cmd.linear.y = 0.0;
            hover_cmd.angular.x = hover_cmd.angular.y = hover_cmd.angular.z = 0.0;
            
            if (use_manual_altitude) {
                double altitude_error = goal.position.z - current_pos.z;
                hover_cmd.linear.z = altitude_error * 0.15;  // Gentle altitude correction
                hover_cmd.linear.z = std::clamp(hover_cmd.linear.z, -0.1, 0.1);
            } else {
                hover_cmd.linear.z = current_altitude_adjustment_ * 0.5;  // Damped terrain following
            }
            
            // Stabilization period - publish multiple times for stability
            for (int hover_count = 0; hover_count < 15; hover_count++) {  // 1.5 second stabilization
                cmd_vel_pub_->publish(hover_cmd);
                rate.sleep();
            }
            
            // Reset static variables for next goal
            first_command = true;
            return true;
        }

        // PID Controller Parameters (tuned for basic VelocityControl)
        const double kp_xy = 0.8;   // Proportional gain for X,Y
        const double ki_xy = 0.05;  // Integral gain for X,Y
        const double kd_xy = 0.15;  // Derivative gain for X,Y
        const double dt = 0.1;      // 10Hz control loop
        
        // Calculate PID terms for X and Y
        double p_term_x = kp_xy * error_x;
        double p_term_y = kp_xy * error_y;
        
        // Integral terms (with windup protection)
        if (std::abs(error_x) < 2.0) {  // Only integrate when close
            integral_x += error_x * dt;
            integral_x = std::clamp(integral_x, -1.0, 1.0);  // Anti-windup
        }
        if (std::abs(error_y) < 2.0) {
            integral_y += error_y * dt;
            integral_y = std::clamp(integral_y, -1.0, 1.0);
        }
        
        double i_term_x = ki_xy * integral_x;
        double i_term_y = ki_xy * integral_y;
        
        // Derivative terms
        double d_term_x = kd_xy * (error_x - last_error_x) / dt;
        double d_term_y = kd_xy * (error_y - last_error_y) / dt;
        
        // Calculate raw velocity commands
        geometry_msgs::msg::Twist raw_cmd;
        raw_cmd.linear.x = p_term_x + i_term_x + d_term_x;
        raw_cmd.linear.y = p_term_y + i_term_y + d_term_y;
        
        // Velocity limiting and scaling for basic VelocityControl
        double max_horizontal_vel = 5.0;  // Reduced from 0.8 for stability
        
        // Distance-based speed ramping
        if (horizontal_distance > 5.0) {
            max_horizontal_vel = 3.0;  // Full speed when far
        } else if (horizontal_distance > 1.0) {
            max_horizontal_vel = 3.0;  // Medium speed when approaching
        } else {
            max_horizontal_vel = 1.5;  // Slow speed when close
        }
        
        // Apply velocity limits
        double cmd_magnitude = std::sqrt(raw_cmd.linear.x*raw_cmd.linear.x + raw_cmd.linear.y*raw_cmd.linear.y);
        if (cmd_magnitude > max_horizontal_vel) {
            raw_cmd.linear.x = (raw_cmd.linear.x / cmd_magnitude) * max_horizontal_vel;
            raw_cmd.linear.y = (raw_cmd.linear.y / cmd_magnitude) * max_horizontal_vel;
        }
        
        // Altitude control with IGNITION LOGIC integrated
        if (use_manual_altitude) {
            double altitude_error = goal.position.z - current_pos.z;
            
            // PID for altitude
            const double kp_z = 0.3;
            const double ki_z = 0.02;
            const double kd_z = 0.1;
            
            if (std::abs(altitude_error) < 1.0) {
                integral_z += altitude_error * dt;
                integral_z = std::clamp(integral_z, -0.5, 0.5);
            }
            
            double d_term_z = kd_z * (altitude_error - last_error_z) / dt;
            
            raw_cmd.linear.z = kp_z * altitude_error + ki_z * integral_z + d_term_z;
            raw_cmd.linear.z = std::clamp(raw_cmd.linear.z, -0.3, 0.3);  // Conservative Z limits
            
            last_error_z = altitude_error;
        } else {
            // Terrain following with damping
            raw_cmd.linear.z = current_altitude_adjustment_ * 0.7;  // Damped response
            
            // Safety altitude limit
            if (current_pos.z > 8.0 && raw_cmd.linear.z > 0) {
                raw_cmd.linear.z = std::min(raw_cmd.linear.z, 0.0);
            }
        }
        
        // Low-pass filter for smooth commands ( for basic VelocityControl plugin)
        geometry_msgs::msg::Twist cmd_vel;
        const double alpha = 0.4;  // Smoothing factor (0 = no smoothing, 1 = no filtering)
        
        if (i == 0) {  // First iteration
            cmd_vel = raw_cmd;
        } else {
            cmd_vel.linear.x = alpha * raw_cmd.linear.x + (1.0 - alpha) * last_cmd.linear.x;
            cmd_vel.linear.y = alpha * raw_cmd.linear.y + (1.0 - alpha) * last_cmd.linear.y;
            cmd_vel.linear.z = alpha * raw_cmd.linear.z + (1.0 - alpha) * last_cmd.linear.z;
        }
        
        // Ensure angular velocities are zero (prevent unwanted rotation)
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        
        // Final scaling for basic VelocityControl compatibility
        cmd_vel.linear.x *= 0.8;  // Scale down for basic plugin
        cmd_vel.linear.y *= 0.8;
        cmd_vel.linear.z *= 0.9;
        
        // Publish command
        cmd_vel_pub_->publish(cmd_vel);
        
        // Store for next iteration
        last_cmd = cmd_vel;
        last_error_x = error_x;
        last_error_y = error_y;

        // Debug output with PID information
        if (i % 20 == 0) {  // Every 2 seconds
            if (use_manual_altitude) {
                RCLCPP_INFO(logger_, "PID: dist=%.2fm, alt_err=%.2fm, cmd=[%.2f,%.2f,%.2f], P=[%.2f,%.2f], I=[%.2f,%.2f]", 
                           horizontal_distance, goal.position.z - current_pos.z,
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z,
                           p_term_x, p_term_y, i_term_x, i_term_y);
            } else {
                RCLCPP_INFO(logger_, "PID: dist=%.2fm, alt=%.2fm, cmd=[%.2f,%.2f,%.2f], adj=%.2fm/s", 
                           horizontal_distance, current_pos.z,
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z,
                           current_altitude_adjustment_);
            }
        }

        rate.sleep();
    }

    RCLCPP_ERROR(logger_, "Failed to reach goal within time limit");
    // Reset static variables on failure
    first_command = true;
    return false;
}

void Controller::abortMission(VehicleData &vehicle)
{
    RCLCPP_WARN(logger_, "Aborting mission for %s - returning to start position",
                vehicle.id.c_str());

    // Clear existing command queue
    {
        std::lock_guard<std::mutex> lock(vehicle.queue_mutex);
        std::queue<VehicleData::Command> empty;
        std::swap(vehicle.command_queue, empty);

        // Get current position for safety check
        geometry_msgs::msg::Point current_pos;
        {
            std::lock_guard<std::mutex> odom_lock(vehicle.odom_mutex);
            current_pos = vehicle.current_odom.pose.pose.position;
        }

        // Only return to start if we're far from it (Change later so it returns to 0, 0 regardless and lands) - INCOMPLETE
        double dist_to_start = calculateDistance(current_pos, vehicle.start_position.position);
        if (dist_to_start > 1.0)
        {
            // Add command to return to starting position
            vehicle.command_queue.push(
                VehicleData::Command(VehicleData::Command::Type::FLYING,
                                     vehicle.start_position));
        }

        // Landing command
        vehicle.command_queue.push(
            VehicleData::Command(VehicleData::Command::Type::LANDING));

        // Mark mission as failed
        vehicle.mission_active = false;
    }

    // Notify controller thread
    vehicle.queue_cv.notify_one();
}


// This function only works when the drone hasn't taken off???
bool Controller::checkForObstacles(const sensor_msgs::msg::LaserScan::SharedPtr &laser_scan)
{
    if (!laser_scan)
    {
        return false;
    }

    // Check ALL 360 degrees for obstacles
    double critical_distance = 1.0; // 1m, emergency ascent
    double warning_distance = 3.0;  // 3m warning distance

    bool obstacle_detected = false;
    double closest_range = std::numeric_limits<double>::max();

    // Check all laser readings
    for (size_t i = 0; i < laser_scan->ranges.size(); i++)
    {
        float range = laser_scan->ranges[i];

        // Check if reading is valid
        if (!std::isnan(range) && !std::isinf(range) &&
            range > laser_scan->range_min &&
            range < laser_scan->range_max)
        {

            if (range < closest_range)
            {
                closest_range = range;
            }

            if (range < critical_distance)
            {
                obstacle_detected = true;

                // Calculate angle of obstacle
                float angle = laser_scan->angle_min + (i * laser_scan->angle_increment);
                float angle_deg = angle * 180.0 / M_PI;

                RCLCPP_ERROR(logger_, "OBSTACLE at %.2fm, angle %.1f degrees - EMERGENCY ASCENT!",
                             range, angle_deg);
            }
            else if (range < warning_distance)
            {
                // Simple throttling using static variable
                static auto last_warning = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();

                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warning).count() > 1000)
                {
                    float angle = laser_scan->angle_min + (i * laser_scan->angle_increment);
                    float angle_deg = angle * 180.0 / M_PI;

                    RCLCPP_WARN(logger_, "Obstacle at %.2fm, angle %.1f degrees",
                                range, angle_deg);
                    last_warning = now;
                }
            }
        }
    }

    if (obstacle_detected)
    {
        RCLCPP_ERROR(logger_, "Closest obstacle at %.2fm - must ascend immediately!", closest_range);
    }

    return obstacle_detected;
}

double Controller::calculateAltitudeAdjustment(double current_altitude, double sonar_range)
{
    // Print raw data for debugging
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0)
    {
        RCLCPP_DEBUG(logger_, "RAW sonar=%.2fm, altitude=%.2fm", sonar_range, current_altitude);
    }

    // Strict validation - if sonar is reporting weird values, don't trust it
    if (std::isnan(sonar_range) || std::isinf(sonar_range) ||
        sonar_range <= 0.1 || sonar_range > 10.0)
    {
        RCLCPP_WARN(logger_, "Invalid sonar reading: %.2f - stopping adjustment", sonar_range);
        return 0.0; // Return 0.0 to prevent drift
    }

    // Target height remains 2.0m
    double target_height = 2.0; 

    // Calculate error - positive to go up
    double error = target_height - sonar_range;

    // Add integral term to eliminate steady-state error
    static double integral_error = 0.0;
    double dt = 0.1; // For 10Hz

    // Only integrate when error is small to prevent windup
    if (std::abs(error) < 0.5)
    {
        integral_error += error * dt;
        // Clamp integral term to prevent excessive accumulation
        integral_error = std::clamp(integral_error, -1.0, 1.0);
    }
    else
    {
        // Reset integral when error is large
        integral_error = 0.0;
    }

    // Asymmetric PI control - different gains for ascent and descent
    double kp, ki;
    if (error > 0)
    {              // Too low - need to ascend
        kp = 0.85; // Slightly reduced from 0.90 to prevent overshoot
        ki = 0.15; // Add integral term for uphill
    }
    else
    {              // Too high - need to descend
        kp = 0.45; // Slightly reduced from 0.50
        ki = 0.10; // Less integral for downhill (gravity)
    }

    // Calculate adjustment with both P and I terms
    double p_term = kp * error;
    double i_term = ki * integral_error;
    double adjustment = p_term + i_term;

    // Allow faster ascent when too low (safety priority)
    double max_ascent = (error > 1.0) ? 1.0 : 0.6; // Faster ascent when far below target
    double max_descent = 0.5;                      // Keep moderate descent rate

    adjustment = std::clamp(adjustment, -max_descent, max_ascent);

    // HARD LIMIT on absolute altitude regardless of terrain
    if (current_altitude > 8.0)
    {
        RCLCPP_ERROR(logger_, "MAXIMUM ALTITUDE REACHED (%.2fm) - EMERGENCY DESCENT", current_altitude);
        integral_error = 0.0; // Reset integral term
        return -0.5;          // Force immediate descent
    }

    // Continuous ascent detection (safety feature)
    static int continuous_ascent_counter = 0;
    static double last_altitude = 0.0;

    if (current_altitude > last_altitude + 0.05 && adjustment > 0.0)
    {
        continuous_ascent_counter++;
        if (continuous_ascent_counter > 20)
        {
            RCLCPP_ERROR(logger_, "CONTINUOUS ASCENT DETECTED (%.2fm) - EMERGENCY CORRECTION",
                         current_altitude);
            continuous_ascent_counter = 0;
            integral_error = 0.0; // Reset integral term
            return -0.5;          // Force descent
        }
    }
    else
    {
        continuous_ascent_counter = 0;
    }
    last_altitude = current_altitude;

    // Add slight damping - prevent oscillation
    static double last_adjustment = 0.0;
    double damped_adjustment = 0.7 * adjustment + 0.3 * last_adjustment;
    last_adjustment = damped_adjustment;

    // More frequent debug logging with integral term info
    if (debug_counter % 5 == 0)
    {
        double ground_elevation = current_altitude - sonar_range;
        RCLCPP_INFO(logger_,
                    "Altitude control: alt=%.2fm, sonar=%.2fm, target=%.2fm, error=%.2fm, P=%.2f, I=%.2f, cmd=%.2fm/s, ground=%.2fm",
                    current_altitude, sonar_range, target_height, error,
                    p_term, i_term, damped_adjustment, ground_elevation);
    }

    return damped_adjustment;
}

double Controller::calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Functional when drone doesn't take off, tested in Gazebo, breaks when drone is moving 
bool Controller::checkObstaclesInDirection(const sensor_msgs::msg::LaserScan::SharedPtr &laser_scan,
                                           const geometry_msgs::msg::Point &current_pos,
                                           const geometry_msgs::msg::Point &target_pos)
{
    if (!laser_scan)
    {
        return false;
    }

    // Calculate direction of travel (NOT drone heading)
    double dx = target_pos.x - current_pos.x;
    double dy = target_pos.y - current_pos.y;
    double travel_distance = std::sqrt(dx * dx + dy * dy);

    if (travel_distance < 0.1)
    {
        return false; // Already at target
    }

    // Normalise direction vector
    dx /= travel_distance;
    dy /= travel_distance;

    // Calculate travel direction angle based on navigation
    double travel_angle = std::atan2(dy, dx);

    // Wide cone to catch obstacles in travel path
    double cone_half_angle = M_PI / 4.0; // ±45 degrees

    bool obstacle_detected = false;
    double closest_obstacle = std::numeric_limits<double>::max();

    // Check for obstacles
    for (size_t i = 0; i < laser_scan->ranges.size(); i++)
    {
        float range = laser_scan->ranges[i];

        // Skip invalid readings
        if (std::isnan(range) || std::isinf(range) ||
            range <= laser_scan->range_min ||
            range >= laser_scan->range_max)
        {
            continue;
        }

        // Calculate angle
        float laser_angle = laser_scan->angle_min + (i * laser_scan->angle_increment);

        // Calculate angle difference from travel direction
        double angle_diff = std::abs(laser_angle - travel_angle);
        if (angle_diff > M_PI)
        {
            angle_diff = 2 * M_PI - angle_diff;
        }

        // Check if obstacle is in our travel path
        if (angle_diff <= cone_half_angle)
        {
            // Obstacle in travel direction - check distance
            if (range < 3.0)
            { // 3m detection distance
                obstacle_detected = true;
                closest_obstacle = std::min(closest_obstacle, static_cast<double>(range));

                RCLCPP_DEBUG(logger_, "Obstacle in travel path: %.2fm at %.1f° (travel dir: %.1f°)",
                             range, laser_angle * 180.0 / M_PI, travel_angle * 180.0 / M_PI);
            }
        }
    }

    if (obstacle_detected)
    {
        RCLCPP_WARN(logger_, "OBSTACLE IN TRAVEL PATH: %.2fm - ascending!", closest_obstacle);
    }

    return obstacle_detected;
}

bool Controller::isObstacleClearanceAchieved(const sensor_msgs::msg::LaserScan::SharedPtr &laser_scan,
                                             const geometry_msgs::msg::Point &current_pos,
                                             const geometry_msgs::msg::Point &target_pos,
                                             double clearance_distance)
{
    if (!laser_scan)
        return false;

    // Calculate travel direction
    double dx = target_pos.x - current_pos.x;
    double dy = target_pos.y - current_pos.y;
    double travel_distance = std::sqrt(dx * dx + dy * dy);

    if (travel_distance < 0.1)
        return true;

    double travel_angle = std::atan2(dy / travel_distance, dx / travel_distance);
    double cone_half_angle = M_PI / 4.0; // ±45 degrees

    // Check if path is clear
    for (size_t i = 0; i < laser_scan->ranges.size(); i++)
    {
        float range = laser_scan->ranges[i];

        if (std::isnan(range) || std::isinf(range) ||
            range <= laser_scan->range_min ||
            range >= laser_scan->range_max)
        {
            continue;
        }

        float laser_angle = laser_scan->angle_min + (i * laser_scan->angle_increment);
        double angle_diff = std::abs(laser_angle - travel_angle);
        if (angle_diff > M_PI)
            angle_diff = 2 * M_PI - angle_diff;

        if (angle_diff <= cone_half_angle && range < clearance_distance)
        {
            return false; // Still obstacles in path
        }
    }

    return true; // Path is clear
}
