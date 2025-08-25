#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mutex>
#include <vector>

#include "rs1_robot/srv/set_waypoints.hpp"

class WaypointNavNode : public rclcpp::Node {
public:
  explicit WaypointNavNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<rs1_robot::srv::SetWaypoints>::SharedPtr set_wp_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();
  void publishMarkers();
  void setWaypointsCb(const std::shared_ptr<rs1_robot::srv::SetWaypoints::Request> req,
                      std::shared_ptr<rs1_robot::srv::SetWaypoints::Response> res);

  // State
  std::mutex mtx_;
  geometry_msgs::msg::Pose current_pose_;
  bool have_pose_ = false;

  std::vector<geometry_msgs::msg::Pose> waypoints_;
  std::size_t wp_idx_ = 0;

  // Params
  double loop_hz_;
  double kp_lin_;
  double kp_yaw_;
  double max_speed_;
  double max_yaw_rate_;
  double arrive_xy_tol_;
  double arrive_yaw_tol_;
  std::string frame_id_;
};
