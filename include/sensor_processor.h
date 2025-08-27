#ifndef SENSOR_PROCESSOR_H
#define SENSOR_PROCESSOR_H

#include "common.h"
#include "terrain_map.h"
#include <mutex>
#include <memory>

/**
 * @brief SensorProcessor class for processing sensor data for a single drone
 * 
 * This class handles sensor data processing and terrain mapping for one drone
 * in a thread-safe manner without ROS node overhead.
 */
class SensorProcessor
{
public:
    explicit SensorProcessor(const std::string& drone_namespace);
    ~SensorProcessor();
    
    // Sensor data processing
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                         const nav_msgs::msg::Odometry::SharedPtr odom);
    
    void processSonarRange(const sensor_msgs::msg::Range::SharedPtr msg,
                          const nav_msgs::msg::Odometry::SharedPtr odom);
    
    // Data retrieval
    grid_map_msgs::msg::GridMap getGridMapMessage() const;
    bool isObstacleDetected() const;
    double getElevationAtPoint(double x, double y) const;
    double getGradientMagnitudeAtPoint(double x, double y) const;
    
    // Getters for namespace
    std::string getDroneNamespace() const { return drone_namespace_; }

private:
    // Drone identification
    std::string drone_namespace_;
    
    // Terrain mapping
    std::unique_ptr<TerrainMap> terrain_map_;
    
    // Thread safety
    mutable std::mutex processing_mutex_;
    
    // Latest processed data (for debugging/monitoring)
    sensor_msgs::msg::LaserScan::SharedPtr latest_laser_;
    sensor_msgs::msg::Range::SharedPtr latest_sonar_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
};

#endif // SENSOR_PROCESSOR_H
