#include "sensor_processor.h"

SensorProcessor::SensorProcessor(const std::string& drone_namespace)
    : drone_namespace_(drone_namespace)
{
    RCLCPP_INFO(rclcpp::get_logger("sensor_processor"), 
                "Initializing SensorProcessor for %s", drone_namespace_.c_str());
    
    // Initialize terrain map
    terrain_map_ = std::make_unique<TerrainMap>(
        rclcpp::get_logger("sensor_processor"), 
        0.1,  // 10cm resolution
        1.0   // 1m initial size
    );
    
    RCLCPP_INFO(rclcpp::get_logger("sensor_processor"), 
                "SensorProcessor initialized for %s", drone_namespace_.c_str());
}

SensorProcessor::~SensorProcessor()
{
    RCLCPP_INFO(rclcpp::get_logger("sensor_processor"), 
                "SensorProcessor for %s shutting down", drone_namespace_.c_str());
}

void SensorProcessor::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                       const nav_msgs::msg::Odometry::SharedPtr odom)
{
    if (!msg || !odom) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(processing_mutex_);
    
    try {
        // TODO: Implement laser scan processing for terrain mapping
        // For now, just update the position to expand the map
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        
        terrain_map_->checkAndExpandForPosition(x, y);
        
        // Update latest processed data
        latest_laser_ = msg;
        latest_odom_ = odom;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sensor_processor"), 
                     "Error processing laser scan for %s: %s", 
                     drone_namespace_.c_str(), e.what());
    }
}

void SensorProcessor::processSonarRange(const sensor_msgs::msg::Range::SharedPtr msg,
                                        const nav_msgs::msg::Odometry::SharedPtr odom)
{
    if (!msg || !odom) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(processing_mutex_);
    
    try {
        // TODO: Implement sonar data processing for terrain mapping
        // For now, just update elevation at current position
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        double z = odom->pose.pose.position.z;
        
        // Estimate ground elevation from drone altitude and sonar reading
        double ground_elevation = z - msg->range;
        terrain_map_->updateElevation(x, y, ground_elevation);
        
        // Update latest processed data
        latest_sonar_ = msg;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sensor_processor"), 
                     "Error processing sonar range for %s: %s", 
                     drone_namespace_.c_str(), e.what());
    }
}

grid_map_msgs::msg::GridMap SensorProcessor::getGridMapMessage() const
{
    std::lock_guard<std::mutex> lock(processing_mutex_);
    
    grid_map_msgs::msg::GridMap grid_map_msg;
    
    if (terrain_map_) {
        grid_map::GridMap map;
        terrain_map_->getGridMap(map);
        grid_map::GridMapRosConverter::toMessage(map, grid_map_msg);
    }
    
    return grid_map_msg;
}

bool SensorProcessor::isObstacleDetected() const
{
    std::lock_guard<std::mutex> lock(processing_mutex_);
    return terrain_map_ ? terrain_map_->hasObstacles() : false;
}

double SensorProcessor::getElevationAtPoint(double x, double y) const
{
    std::lock_guard<std::mutex> lock(processing_mutex_);
    return terrain_map_ ? terrain_map_->getElevationAtPoint(x, y) : 0.0;
}

double SensorProcessor::getGradientMagnitudeAtPoint(double x, double y) const
{
    std::lock_guard<std::mutex> lock(processing_mutex_);
    return terrain_map_ ? terrain_map_->getGradientMagnitudeAtPoint(x, y) : 0.0;
}
