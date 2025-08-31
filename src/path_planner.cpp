#include "mission/path_planner.h"
#include <cmath>

namespace drone_swarm
{

  PathPlanner::PathPlanner() : current_waypoint_index_(0) {}

  void PathPlanner::setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) {
    // Set new waypoint sequence and reset index to start
    waypoints_ = waypoints;
    current_waypoint_index_ = 0;
  }

  geometry_msgs::msg::PoseStamped PathPlanner::getNextWaypoint() {
    // Return next waypoint and advance index if available
    if (hasNextWaypoint()) {
      return waypoints_[current_waypoint_index_++];
    }
    return geometry_msgs::msg::PoseStamped();
  }

  geometry_msgs::msg::PoseStamped PathPlanner::getCurrentWaypoint() const {
    // Return current waypoint without advancing index
    if (hasNextWaypoint()) {
      return waypoints_[current_waypoint_index_];
    }
    return geometry_msgs::msg::PoseStamped();
  }

  bool PathPlanner::hasNextWaypoint() const {
    return current_waypoint_index_ < waypoints_.size();
  }

  void PathPlanner::reset() {
    current_waypoint_index_ = 0;
  }

  double PathPlanner::getDistanceToWaypoint(const geometry_msgs::msg::PoseStamped& current_pose) const {
    if (!hasNextWaypoint()) return 0.0;
    
    const auto& target = waypoints_[current_waypoint_index_];
    double dx = target.pose.position.x - current_pose.pose.position.x;
    double dy = target.pose.position.y - current_pose.pose.position.y;
    double dz = target.pose.position.z - current_pose.pose.position.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

}  // namespace drone_swarm
