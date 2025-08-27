#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "common.h"

class Controller {
public:
    /**
     * @brief Constructor for Controller
     * 
     * @param cmd_vel_pub Publisher for velocity commands
     * @param takeoff_pub Publisher for takeoff commands
     * @param land_pub Publisher for landing commands
     * @param logger Logger to use
     */
    Controller(
        const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& cmd_vel_pub,
        const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr& takeoff_pub,
        const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr& land_pub,
        const rclcpp::Logger& logger);
    
    /**
     * @brief Process a vehicle's command
     * 
     * Executes a command from the vehicle's queue
     * @param vehicle Vehicle data
     * @param command Command to execute
     * @return true if command executed successfully, false otherwise
     */
    bool processCommand(VehicleData& vehicle, const VehicleData::Command& command);
    
    /**
     * @brief Start a mission for a vehicle
     * 
     * Creates and queues commands for a complete mission
     * @param vehicle Vehicle data
     */
    void startMission(VehicleData& vehicle);
    
    /**
     * @brief Stop the mission for a vehicle
     * 
     * Clears command queue and adds a STOP command
     * @param vehicle Vehicle data
     */
    void stopMission(VehicleData& vehicle);
    
    /**
     * @brief Command the drone to take off
     * 
     * Sends the drone takeoff command and monitors altitude
     * @param vehicle Vehicle data
     * @return true if takeoff successful, false otherwise
     */
    bool takeoff(VehicleData& vehicle);
    
    /**
     * @brief Command the drone to land
     * 
     * Sends the drone landing command
     * @param vehicle Vehicle data
     * @return true if landing command sent successfully
     */
    bool land(VehicleData& vehicle);
    
    /**
     * @brief Fly to a goal position
     * 
     * Controls the drone to reach a specified goal position
     * @param vehicle Vehicle data
     * @param goal The goal position to reach
     * @return true if goal reached successfully, false otherwise
     */
    bool flyToGoal(VehicleData& vehicle, const geometry_msgs::msg::Pose& goal);
    
    /**
     * @brief Calculate the distance between two points
     * 
     * Computes the Euclidean distance between two positions
     * @param p1 First position
     * @param p2 Second position
     * @return Distance in meters
     */
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);

    /**
     * @brief Abort mission if waypoints are not traversable
     * 
     * Computes the Euclidean distance between two positions
     * @param vehicle Vehicle data
     */
    void abortMission(VehicleData& vehicle);

    /**
     * @brief Check for obstacles using laser data (360-degree emergency detection)
     * 
     * Scans all laser readings for very close obstacles that require immediate
     * emergency ascent regardless of travel direction.
     * 
     * @param laser_scan Latest laser scan data
     * @return true if critical obstacle detected requiring emergency ascent, false otherwise
     */
    bool checkForObstacles(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan);
    
    /**
     * @brief Calculate altitude adjustment based on sonar reading
     * 
     * @param current_altitude Current drone altitude
     * @param sonar_range Distance to ground from sonar
     * @return Vertical velocity adjustment
     */
    double calculateAltitudeAdjustment(double current_altitude, double sonar_range);

    /**
     * @brief Set the current altitude adjustment
     * Used to maintain altitude control during flight
     */
    void setAltitudeAdjustment(double adjustment) { 
        current_altitude_adjustment_ = adjustment; 
    }

    /**
     * @brief Check for obstacles in the direction of travel
     * 
     * @param laser_scan Latest laser scan data
     * @param current_pos Current drone position
     * @param target_pos Target position we're navigating to
     * @return true if obstacle detected in travel direction, false otherwise
     */
    bool checkObstaclesInDirection(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan,
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Point& target_pos);

    /**
     * @brief Check if obstacle clearance has been achieved
     * 
     * @param laser_scan Latest laser scan data
     * @param current_pos Current drone position
     * @param target_pos Target position we're navigating to
     * @param clearance_distance Minimum clearance distance required
     * @return true if path is clear, false if obstacles still present
     */    
    bool isObstacleClearanceAchieved(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan,
                                const geometry_msgs::msg::Point& current_pos,
                                const geometry_msgs::msg::Point& target_pos,
                                double clearance_distance = 4.0);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;
    rclcpp::Logger logger_;

    geometry_msgs::msg::Point last_waypoint_position_;
    std::vector<geometry_msgs::msg::Pose> generated_waypoints_;
    
    bool has_last_waypoint_;

    // Target is exactly 2m above ground
    double desired_ground_clearance_ = 2.0;  // 2m
    double min_obstacle_distance_ = 3.0;    // 3m
    double current_altitude_adjustment_ = 0.0;
};

#endif // CONTROLLER_H