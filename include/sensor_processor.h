/**
 * @file sensor_processor.h
 * @brief Sensor data processing for LiDAR and sonar
 * @author Jackson Russell
 * @date October 2025
 */

#ifndef DRONE_SENSOR_PROCESSOR_H
#define DRONE_SENSOR_PROCESSOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <memory>
#include <string>
#include <mutex>

namespace drone_swarm
{

/**
 * @struct DetectedObject
 * @brief Represents an object detected by sensors
 */
struct DetectedObject {
  geometry_msgs::msg::Point position;  ///< Position in map frame
  double distance;                     ///< Distance from drone
  double confidence;                   ///< Detection confidence (0.0-1.0)
  std::string type;                    ///< Object type ("obstacle", "landmark", "unknown")
  rclcpp::Time timestamp;              ///< When detected
  std::string sensor_source;           ///< Which sensor detected it ("lidar", "sonar")
};

class SensorProcessor
{
public:
  /**
   * @brief Constructor
   * @param node Pointer to parent ROS2 node for parameter access and logging
   * @param drone_namespace Namespace for this specific drone
   */
  explicit SensorProcessor(rclcpp::Node* node, const std::string& drone_namespace);
  
  /**
   * @brief Initialise sensor subscriptions and publishers
   */
  void initialise();
  
  /**
   * @brief Get all currently detected objects
   * @return Vector of detected objects in map frame
   */
  std::vector<DetectedObject> getDetectedObjects() const;
  
  /**
   * @brief Get current altitude from sonar
   * @return Altitude above ground in meters, -1.0 if invalid
   */
  double getCurrentAltitude() const;

private:
  // Sensor callbacks
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  
  // Processing methods
  std::vector<DetectedObject> processLidarData(const sensor_msgs::msg::LaserScan& scan);
  void processObstacles(const sensor_msgs::msg::LaserScan& scan, 
                       std::vector<DetectedObject>& objects);
  void recognizeLandmarks(std::vector<DetectedObject>& objects);
  
  // Utility methods
  geometry_msgs::msg::Point transformToMapFrame(const geometry_msgs::msg::Point& point, 
                                               const std::string& source_frame);
  double calculateConfidence(double range, double intensity = 1.0);
  void publishVisualisation();
  void cleanupOldDetections();
  
  // ROS interfaces
  rclcpp::Node* node_;
  std::string drone_namespace_;
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;
  
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  
  // TF2 for coordinate transformations
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Detection data
  double current_altitude_;
  
  // Thread safety
  mutable std::mutex detection_mutex_;
  mutable std::mutex landmark_mutex_;
  
  // Configuration parameters
  double lidar_max_range_;
  double lidar_min_range_;
  double min_obstacle_size_;
  double max_obstacle_size_;
  
  // Frame names
  std::string map_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string sonar_frame_;
};

} // namespace drone_swarm

#endif // DRONE_SENSOR_PROCESSOR_H