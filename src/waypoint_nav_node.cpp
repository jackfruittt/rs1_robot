#include "waypoint_nav_node.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
using namespace std::chrono_literals;

static double yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion qq;
  tf2::fromMsg(q, qq);
  double roll, pitch, yaw;
  tf2::Matrix3x3(qq).getRPY(roll, pitch, yaw);
  return yaw;
}

WaypointNavNode::WaypointNavNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("waypoint_nav", options)
{
  // Parameters (override per namespace if you want)
  loop_hz_        = this->declare_parameter("loop_hz", 50.0);
  kp_lin_         = this->declare_parameter("kp_lin", 0.8);
  kp_yaw_         = this->declare_parameter("kp_yaw", 1.5);
  max_speed_      = this->declare_parameter("max_speed", 1.0);
  max_yaw_rate_   = this->declare_parameter("max_yaw_rate", 1.0);
  arrive_xy_tol_  = this->declare_parameter("arrive_xy_tol", 0.15);
  arrive_yaw_tol_ = this->declare_parameter("arrive_yaw_tol", 0.2);
  frame_id_       = this->declare_parameter("frame_id", "world");

  // I/O (relative names; namespaces come from launch)
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 20, std::bind(&WaypointNavNode::odomCb, this, std::placeholders::_1));
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  set_wp_srv_ = create_service<rs1_robot::srv::SetWaypoints>(
      "set_waypoints",
      std::bind(&WaypointNavNode::setWaypointsCb, this, std::placeholders::_1, std::placeholders::_2));

  auto period = std::chrono::duration<double>(1.0 / std::max(1.0, loop_hz_));
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                             std::bind(&WaypointNavNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "waypoint_nav ready (kp_lin=%.2f kp_yaw=%.2f max_v=%.2f)",
              kp_lin_, kp_yaw_, max_speed_);
}

void WaypointNavNode::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  current_pose_ = msg->pose.pose;
  have_pose_ = true;
}

void WaypointNavNode::setWaypointsCb(
  const std::shared_ptr<rs1_robot::srv::SetWaypoints::Request> req,
  std::shared_ptr<rs1_robot::srv::SetWaypoints::Response> res)
{
  if (req->targets.poses.empty()) {
    res->accepted = false;
    res->info = "Empty waypoint list";
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    waypoints_ = req->targets.poses;
    wp_idx_ = 0;
  }
  res->accepted = true;
  res->info = "Waypoints accepted";
  RCLCPP_INFO(get_logger(), "Received %zu waypoints", waypoints_.size());
}

void WaypointNavNode::controlLoop() {
  geometry_msgs::msg::Twist cmd;

  // Copy state
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Pose goal;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_pose_) { cmd_pub_->publish(cmd); return; }
    if (wp_idx_ >= waypoints_.size()) { cmd_pub_->publish(cmd); return; }
    pose = current_pose_;
    goal = waypoints_[wp_idx_];
  }

  // Errors
  const double dx = goal.position.x - pose.position.x;
  const double dy = goal.position.y - pose.position.y;
  const double dz = goal.position.z - pose.position.z;
  const double dist_xy = std::hypot(dx, dy);
  const double target_yaw = std::atan2(dy, dx);
  const double yaw = yaw_from_quat(pose.orientation);
  const double yaw_err = std::atan2(std::sin(target_yaw - yaw), std::cos(target_yaw - yaw));

  // Controller
  cmd.linear.x  = std::clamp(kp_lin_ * dist_xy, -max_speed_,  max_speed_);
  cmd.linear.z  = std::clamp(kp_lin_ * dz,      -max_speed_,  max_speed_);
  cmd.angular.z = std::clamp(kp_yaw_ * yaw_err, -max_yaw_rate_, max_yaw_rate_);

  // Arrival criteria
  if (dist_xy < arrive_xy_tol_ && std::abs(yaw_err) < arrive_yaw_tol_) {
    std::lock_guard<std::mutex> lk(mtx_);
    wp_idx_++;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Advancing to waypoint %zu/%zu", wp_idx_, waypoints_.size());
  }

  cmd_pub_->publish(cmd);
  publishMarkers();
}

void WaypointNavNode::publishMarkers() {
  if (!marker_pub_->get_subscription_count()) return;

  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = now();
  m.ns = "waypoints";
  m.id = 1;
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = m.scale.y = m.scale.z = 0.15;
  m.color.a = 0.9; m.color.r = 0.1f; m.color.g = 0.8f; m.color.b = 0.2f;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &p : waypoints_) m.points.push_back(p.position);
  }
  arr.markers.push_back(m);

  visualization_msgs::msg::Marker cur;
  cur.header = m.header;
  cur.ns = "current_target";
  cur.id = 2;
  cur.type = visualization_msgs::msg::Marker::SPHERE;
  cur.scale.x = cur.scale.y = cur.scale.z = 0.25;
  cur.color.a = 1.0; cur.color.r = 1.0f; cur.color.g = 0.2f; cur.color.b = 0.2f;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (wp_idx_ < waypoints_.size()) cur.pose = waypoints_[wp_idx_];
  }
  arr.markers.push_back(cur);

  marker_pub_->publish(arr);
}
