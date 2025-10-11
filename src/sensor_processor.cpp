#include "sensor_processor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_swarm
{

SensorProcessor::SensorProcessor(rclcpp::Node* node, const std::string& drone_namespace)
  : node_(node), drone_namespace_(drone_namespace), current_altitude_(-1.0)
{
  // Load parameters
  node_->declare_parameter("sensor.lidar_max_range", 30.0);
  node_->declare_parameter("sensor.lidar_min_range", 0.1);
  node_->declare_parameter("sensor.min_obstacle_size", 0.1);
  node_->declare_parameter("sensor.max_obstacle_size", 5.0);
  
  lidar_max_range_ = node_->get_parameter("sensor.lidar_max_range").as_double();
  lidar_min_range_ = node_->get_parameter("sensor.lidar_min_range").as_double();
  min_obstacle_size_ = node_->get_parameter("sensor.min_obstacle_size").as_double();
  max_obstacle_size_ = node_->get_parameter("sensor.max_obstacle_size").as_double();
  
  // Set up frame names
  map_frame_ = "map";
  base_frame_ = drone_namespace_ + "/base_link";
  lidar_frame_ = drone_namespace_ + "/lidar_link";
  sonar_frame_ = drone_namespace_ + "/down_sonar_link";
  
  // Initialise TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  RCLCPP_INFO(node_->get_logger(), "SensorProcessor initialised for %s", drone_namespace_.c_str());
}

void SensorProcessor::initialise()
{
  // Create subscriptions
  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/" + drone_namespace_ + "/lidar", 10,
    std::bind(&SensorProcessor::lidarCallback, this, std::placeholders::_1));
    
  sonar_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
    "/" + drone_namespace_ + "/sonar", 10,
    std::bind(&SensorProcessor::sonarCallback, this, std::placeholders::_1));
  
  // Create cleanup timer
  cleanup_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SensorProcessor::cleanupOldDetections, this));
    
  RCLCPP_INFO(node_->get_logger(), "SensorProcessor subscriptions initialized for %s", 
              drone_namespace_.c_str());
}

void SensorProcessor::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(detection_mutex_);
  
  // Process lidar data to extract objects
  auto new_objects = processLidarData(*msg);
}

void SensorProcessor::sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  if (msg->range >= msg->min_range && msg->range <= msg->max_range) {
    current_altitude_ = msg->range;
  } else {
    current_altitude_ = -1.0; // Invalid reading
  }
}

std::vector<DetectedObject> SensorProcessor::processLidarData(const sensor_msgs::msg::LaserScan& scan)
{
  std::vector<DetectedObject> objects;
  
  // Simple obstacle detection using clustering
  std::vector<geometry_msgs::msg::Point> raw_points;
  
  // Convert polar coordinates to cartesian
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float range = scan.ranges[i];
    
    // Skip invalid readings
    if (range < scan.range_min || range > scan.range_max || 
        std::isnan(range) || std::isinf(range)) {
      continue;
    }
    
    // Skip readings beyond our processing range
    if (range < lidar_min_range_ || range > lidar_max_range_) {
      continue;
    }
    
    float angle = scan.angle_min + i * scan.angle_increment;
    
    geometry_msgs::msg::Point point;
    point.x = range * cos(angle);
    point.y = range * sin(angle);
    point.z = 0.0; // 2D lidar
    
    raw_points.push_back(point);
  }
  
  // Cluster nearby points into objects
  std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
  std::vector<bool> processed(raw_points.size(), false);
  
  for (size_t i = 0; i < raw_points.size(); ++i) {
    if (processed[i]) continue;
    
    std::vector<geometry_msgs::msg::Point> cluster;
    cluster.push_back(raw_points[i]);
    processed[i] = true;
    
    // Find nearby points
    for (size_t j = i + 1; j < raw_points.size(); ++j) {
      if (processed[j]) continue;
      
      double distance = sqrt(
        pow(raw_points[i].x - raw_points[j].x, 2) +
        pow(raw_points[i].y - raw_points[j].y, 2)
      );
    }
    
    if (cluster.size() >= 2) { // Minimum cluster size
      clusters.push_back(cluster);
    }
  }
  
  // Convert clusters to detected objects
  for (const auto& cluster : clusters) {
    DetectedObject obj;
    
    // Calculate cluster center
    obj.position.x = 0.0;
    obj.position.y = 0.0;
    obj.position.z = 0.0;
    
    for (const auto& point : cluster) {
      obj.position.x += point.x;
      obj.position.y += point.y;
    }
    
    obj.position.x /= cluster.size();
    obj.position.y /= cluster.size();
    
    // Transform to map frame
    obj.position = transformToMapFrame(obj.position, lidar_frame_);
    
    // Calculate properties
    obj.distance = sqrt(obj.position.x * obj.position.x + obj.position.y * obj.position.y);
    obj.confidence = calculateConfidence(obj.distance);
    obj.type = "obstacle"; // Default classification
    obj.timestamp = node_->get_clock()->now();
    obj.sensor_source = "lidar";
    
    // Filter by size (rough estimate based on cluster span)
    double cluster_size = 0.0;
    for (size_t i = 1; i < cluster.size(); ++i) {
      double dist = sqrt(
        pow(cluster[i].x - cluster[0].x, 2) +
        pow(cluster[i].y - cluster[0].y, 2)
      );
      cluster_size = std::max(cluster_size, dist);
    }
    
    if (cluster_size >= min_obstacle_size_ && cluster_size <= max_obstacle_size_) {
      objects.push_back(obj);
    }
  }
  
  return objects;
}

geometry_msgs::msg::Point SensorProcessor::transformToMapFrame(
  const geometry_msgs::msg::Point& point, const std::string& source_frame)
{
  geometry_msgs::msg::PointStamped point_in, point_out;
  point_in.header.frame_id = source_frame;
  point_in.header.stamp = node_->get_clock()->now();
  point_in.point = point;
  
  try {
    tf_buffer_->transform(point_in, point_out, map_frame_);
    return point_out.point;
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", ex.what());
    return point; // Return original if transform fails
  }
}

double SensorProcessor::calculateConfidence(double range, double intensity)
{
  // Confidence decreases with distance and increases with intensity
  double distance_factor = std::max(0.1, 1.0 - (range / lidar_max_range_));
  double intensity_factor = intensity; // If intensity is available from lidar
  
  return std::min(1.0, distance_factor * intensity_factor);
}

} // namespace drone_swarm