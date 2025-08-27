#ifndef SENSOR_NODE_H
#define SENSOR_NODE_H

#include "common.h"
#include "laserprocessing.h"
#include "terrain_map.h"  // Use the new TerrainMap class

/**
 * @brief SensorNode class for ROS sensor data handling
 * 
 * This class focuses on ROS communication and delegates all mapping
 * operations to the TerrainMap class for logic handling
 */
class SensorNode : public rclcpp::Node
{
public:
    SensorNode();
    ~SensorNode();

    // Public interface methods - delegate to TerrainMap
    bool isObstacleDetected() const;
    double getElevationAtPoint(double x, double y) const;
    double getGradientMagnitudeAtPoint(double x, double y) const;
    void getGridMap(grid_map::GridMap& map) const;
    
private:
    // ROS callbacks
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);
    void sonarCallback(const std::shared_ptr<sensor_msgs::msg::Range> msg);
    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
    
    // Timer callbacks
    void publishMapCallback();
    void publishGradientVisualizationCallback();
    
    // Sensor data processing
    void processLaserData();
    bool isValidSonarReading(const std::shared_ptr<sensor_msgs::msg::Range> msg) const;
    nav_msgs::msg::Odometry::SharedPtr getLatestOdom() const;
    
    // ROS communication setup
    void setupSubscribers();
    void setupPublishers();
    void setupTimers();

    // Subscribers, Publishers, and Timers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gradient_arrows_pub_;
    
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr gradient_timer_;
    
    // Thread-safe sensor data storage
    mutable std::mutex laser_mutex_;
    mutable std::mutex sonar_mutex_;
    mutable std::mutex odom_mutex_;
    
    // Latest sensor readings
    sensor_msgs::msg::LaserScan::SharedPtr latest_laser_;
    sensor_msgs::msg::Range::SharedPtr latest_sonar_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    
    // Obstacle detection
    std::atomic<bool> obstacle_detected_;
    
    // Drone namespace for topics
    std::string drone_namespace_;
    
    // Sensor processing utilities
    std::unique_ptr<LaserProcessing> laser_processor_;
    
    // Use TerrainMap for all mapping operations
    std::unique_ptr<TerrainMap> terrain_map_;
};

#endif // SENSOR_NODE_H