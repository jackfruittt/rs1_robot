#include "sensor_node.h"

using namespace std::chrono_literals;

SensorNode::SensorNode()
    : Node("sensor_processor"),
      obstacle_detected_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing SensorNode with TerrainMap...");

    // Declare drone namespace parameter
    this->declare_parameter("drone_namespace", std::string("rs1_drone"));
    drone_namespace_ = this->get_parameter("drone_namespace").as_string();

    // Create the terrain map with configuration
    terrain_map_ = std::make_unique<TerrainMap>(
        this->get_logger(),
        0.1,    // 10cm resolution
        1.0    // Start with 10m x 10m map
    );
    
    // Set up ROS communication, alternative and cleaner approach to drone node. Intentionally done to show both ways. This is my preferred. 
    setupSubscribers();
    setupPublishers();
    setupTimers();
    
    RCLCPP_INFO(this->get_logger(), "SensorNode initialized successfully with TerrainMap");
}

SensorNode::~SensorNode() {
    RCLCPP_INFO(this->get_logger(), "SensorNode shutting down");
}

void SensorNode::setupSubscribers() {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/" + drone_namespace_ + "/laserscan", 10,
        std::bind(&SensorNode::laserCallback, this, std::placeholders::_1));
    
    sonar_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/" + drone_namespace_ + "/sonar", 10,
        std::bind(&SensorNode::sonarCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + drone_namespace_ + "/odom", 10,
        std::bind(&SensorNode::odomCallback, this, std::placeholders::_1));
}

void SensorNode::setupPublishers() {
    grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/grid_map", 10);

    obstacle_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/obstacle_detected", 10);
    
    gradient_arrows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/terrain_gradients", 10);
}

void SensorNode::setupTimers() {
    // Publish grid map every 500ms
    publish_timer_ = this->create_wall_timer(
        500ms, std::bind(&SensorNode::publishMapCallback, this));
    
    // Publish gradient visualization every 1 second
    gradient_timer_ = this->create_wall_timer(
        1000ms, std::bind(&SensorNode::publishGradientVisualizationCallback, this));
}

// ROS Callbacks - Focus on communication, delegate processing to TerrainMap
void SensorNode::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
    {
        std::lock_guard<std::mutex> lock(laser_mutex_);
        latest_laser_ = msg;
    }
    
    // Process laser data for obstacle detection
    processLaserData();
}

void SensorNode::sonarCallback(const std::shared_ptr<sensor_msgs::msg::Range> msg) {
    {
        std::lock_guard<std::mutex> lock(sonar_mutex_);
        latest_sonar_ = msg;
    }
    
    // Validate sonar reading
    if (!isValidSonarReading(msg)) {
        return;
    }
    
    // Get current drone position
    auto odom = getLatestOdom();
    if (!odom) {
        return;
    }
    
    // Calculate ground elevation
    double drone_x = odom->pose.pose.position.x;
    double drone_y = odom->pose.pose.position.y;
    double drone_altitude = odom->pose.pose.position.z;
    double ground_elevation = drone_altitude - msg->range;
    
    // DELEGATE: Let TerrainMap handle all mapping operations
    bool updated = terrain_map_->updateElevation(drone_x, drone_y, ground_elevation);
    
    if (updated) {
        // Calculate gradients around the updated area
        terrain_map_->calculateGradientsAroundPoint(drone_x, drone_y, 2.0);
    }
}

void SensorNode::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
}

// Timer Callbacks
void SensorNode::publishMapCallback() {
    try {
        // Get grid map from TerrainMap and publish it
        auto grid_map_msg = terrain_map_->getGridMapMessage();
        grid_map_pub_->publish(grid_map_msg);
        
        // Log statistics periodically
        static int publish_count = 0;
        if (++publish_count % 20 == 0) {  // Every 10 seconds
            terrain_map_->logStatistics();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in publishMapCallback: %s", e.what());
    }
}

void SensorNode::publishGradientVisualizationCallback() {
    try {
        // Get gradient arrows from TerrainMap and publish them
        auto gradient_arrows = terrain_map_->getGradientArrows();
        gradient_arrows_pub_->publish(gradient_arrows);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing gradient visualization: %s", e.what());
    }
}

// Sensor Data Processing
void SensorNode::processLaserData() {
    std::lock_guard<std::mutex> lock(laser_mutex_);
    
    if (!latest_laser_) {
        return;
    }
    
    // Create or update laser processor
    if (!laser_processor_) {
        laser_processor_ = std::make_unique<LaserProcessing>(*latest_laser_);
    } else {
        laser_processor_->newScan(*latest_laser_);
    }
    
    // Count object readings for obstacle detection
    unsigned int object_readings = laser_processor_->countObjectReadings();
    
    // Simple obstacle detection threshold
    obstacle_detected_ = (object_readings > 10);
    
    if (obstacle_detected_) {
        RCLCPP_DEBUG(this->get_logger(), "Obstacle detected with %u laser readings", object_readings);
    }

    // Publish obstacle status
    std_msgs::msg::Bool obstacle_msg;
    obstacle_msg.data = obstacle_detected_;
    obstacle_detected_pub_->publish(obstacle_msg);
}

bool SensorNode::isValidSonarReading(const std::shared_ptr<sensor_msgs::msg::Range> msg) const {
    return !(std::isnan(msg->range) || std::isinf(msg->range) || 
             msg->range <= 0.0 || msg->range > msg->max_range);
}

nav_msgs::msg::Odometry::SharedPtr SensorNode::getLatestOdom() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return latest_odom_;
}

// Public Interface Methods - Delegate to TerrainMap
bool SensorNode::isObstacleDetected() const {
    return obstacle_detected_.load();
}

double SensorNode::getElevationAtPoint(double x, double y) const {
    return terrain_map_->getElevationAtPoint(x, y);
}

double SensorNode::getGradientMagnitudeAtPoint(double x, double y) const {
    return terrain_map_->getGradientMagnitudeAtPoint(x, y);
}

void SensorNode::getGridMap(grid_map::GridMap& map) const {
    terrain_map_->getGridMap(map);
}