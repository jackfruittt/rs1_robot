#include "mission/waypoint_planner.h"
#include <cmath>

namespace drone_swarm
{

  WaypointPlanner::WaypointPlanner() : current_waypoint_index_(0) {}

  void WaypointPlanner::setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) {
    // Set new waypoint sequence and reset index to start
    waypoints_ = waypoints;
    current_waypoint_index_ = 0;
    
    // TODO: ADD LOGGING FOR NAV2 RRT PLANNER INTEGRATION
    // RCLCPP_INFO(rclcpp::get_logger("path_planner"), 
    //            "Set %zu waypoints from autonomous path planner", waypoints.size());
    // Example: Log when waypoints are set from autonomous planner vs manual input
  }

  geometry_msgs::msg::PoseStamped WaypointPlanner::getNextWaypoint() {
    // Return next waypoint and advance index if available
    if (hasNextWaypoint()) {
      return waypoints_[current_waypoint_index_++];
    }
    return geometry_msgs::msg::PoseStamped();
  }

  geometry_msgs::msg::PoseStamped WaypointPlanner::getCurrentWaypoint() const {
    // Return current waypoint without advancing index
    if (hasNextWaypoint()) {
      return waypoints_[current_waypoint_index_];
    }
    return geometry_msgs::msg::PoseStamped();
  }

  bool WaypointPlanner::hasNextWaypoint() const {
    return current_waypoint_index_ < waypoints_.size();
  }

  void WaypointPlanner::reset() {
    current_waypoint_index_ = 0;
  }

  double WaypointPlanner::getDistanceToWaypoint(const geometry_msgs::msg::PoseStamped& current_pose) const {
    if (!hasNextWaypoint()) return 0.0;
    
    const auto& target = waypoints_[current_waypoint_index_];
    double dx = target.pose.position.x - current_pose.pose.position.x;
    double dy = target.pose.position.y - current_pose.pose.position.y;
    // Use 2D distance only - altitude is independently controlled by terrain following
    // This prevents obstacle avoidance climbs from blocking waypoint progression
    
    return std::sqrt(dx*dx + dy*dy);
  }
  
  std::vector<geometry_msgs::msg::PoseStamped> WaypointPlanner::getAllWaypoints() const {
    return waypoints_;
  }
  
  size_t WaypointPlanner::getCurrentWaypointIndex() const {
    return current_waypoint_index_;
  }
  
  void WaypointPlanner::setCurrentWaypointIndex(size_t index) {
    // Clamp to valid range
    if (index > waypoints_.size()) {
      current_waypoint_index_ = waypoints_.size();
    } else {
      current_waypoint_index_ = index;
    }
  }

  // TODO: ADD NAV2 PATH CONVERSION IMPLEMENTATION FOR AUTONOMOUS PATH PLANNING
  // void PathPlanner::setWaypointsFromPath(const nav_msgs::msg::Path& path, double subsample_distance) {
  //   if (path.poses.empty()) {
  //     RCLCPP_WARN(rclcpp::get_logger("path_planner"), "Cannot set waypoints from empty path");
  //     return;
  //   }
  //   
  //   std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  //   waypoints.reserve(path.poses.size());
  //   
  //   // Always include the first waypoint
  //   waypoints.push_back(path.poses[0]);
  //   
  //   // Subsample the path based on distance
  //   for (size_t i = 1; i < path.poses.size(); ++i) {
  //     const auto& current = path.poses[i];
  //     const auto& last_added = waypoints.back();
  //     
  //     // Calculate distance from last added waypoint
  //     double dx = current.pose.position.x - last_added.pose.position.x;
  //     double dy = current.pose.position.y - last_added.pose.position.y;
  //     double dz = current.pose.position.z - last_added.pose.position.z;
  //     double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
  //     
  //     // Add waypoint if far enough from last one, or if it's the final waypoint
  //     if (distance >= subsample_distance || i == path.poses.size() - 1) {
  //       waypoints.push_back(current);
  //     }
  //   }
  //   
  //   // Set the subsampled waypoints
  //   setWaypoints(waypoints);
  //   
  //   RCLCPP_INFO(rclcpp::get_logger("path_planner"), 
  //              "Converted nav2 path (%zu poses) to %zu waypoints with %.1fm spacing",
  //              path.poses.size(), waypoints.size(), subsample_distance);
  // }
  // Example: Convert nav2 path to waypoints with configurable spacing

}  // namespace drone_swarm
