#include "mission/waypoint_planner.h"
#include <cmath>
#include <random>
#include <algorithm>

namespace drone_swarm
{

  WaypointPlanner::WaypointPlanner() : current_waypoint_index_(0) {}

  void WaypointPlanner::setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) {
    // Set new waypoint sequence and reset index to start
    waypoints_ = waypoints;
    current_waypoint_index_ = 0;
    
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
  
  /******************************** TEMU RRT *******************************************/
  std::vector<geometry_msgs::msg::PoseStamped> WaypointPlanner::generateRandomWaypoints(
      const geometry_msgs::msg::PoseStamped& start_pose,
      int num_waypoints,
      double min_distance,
      double max_distance,
      double altitude) {
    
    std::vector<geometry_msgs::msg::PoseStamped> new_waypoints;
    std::vector<geometry_msgs::msg::Point> path_points;
    
    // Random number generator with time-based seed for variability
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> distance_dist(min_distance, max_distance);
    std::uniform_real_distribution<> angle_dist(0.0, 2.0 * M_PI);
    
    // Map boundaries
    constexpr double MAP_MIN_X = -47.0;
    constexpr double MAP_MAX_X = 47.0;
    constexpr double MAP_MIN_Y = -47.0;
    constexpr double MAP_MAX_Y = 47.0;
    constexpr double BOUNDARY_MARGIN = 1.0;  // Stay 1m from edges

    // Start from current position
    geometry_msgs::msg::Point current_point;
    current_point.x = start_pose.pose.position.x;
    current_point.y = start_pose.pose.position.y;
    current_point.z = altitude;
    path_points.push_back(current_point);
    
    // Track last direction to encourage smooth paths
    double last_angle = 0.0;
    bool has_last_angle = false;
    
    int attempts = 0;
    const int max_attempts = 1000;  // Prevent infinite loops
    
    while (path_points.size() < static_cast<size_t>(num_waypoints + 1) && attempts < max_attempts) {
      attempts++;
      
      // Generate candidate point
      double distance = distance_dist(gen);
      double angle;
      
      if (has_last_angle) {
        // Bias towards continuing in similar direction with some randomness
        // Using 45 degree std dev for straighter, more exploratory paths to prevent circling the same small area
        std::normal_distribution<> angle_variation(last_angle, M_PI / 4.0);  // 45 degree std dev
        angle = angle_variation(gen);
        
        // Ensure no sharp reversals (> 150 degrees turn)
        double angle_diff = std::abs(std::remainder(angle - last_angle, 2.0 * M_PI));
        if (angle_diff > 5.0 * M_PI / 6.0) {  // If turn > 150 degrees
          continue;  // Try again with different angle
        }
      } else {
        // First point can go in any direction
        angle = angle_dist(gen);
      }
      
      // Calculate candidate position
      geometry_msgs::msg::Point candidate;
      candidate.x = current_point.x + distance * std::cos(angle);
      candidate.y = current_point.y + distance * std::sin(angle);
      candidate.z = altitude;
      
      // Check map boundaries with margin
      if (candidate.x < MAP_MIN_X + BOUNDARY_MARGIN || candidate.x > MAP_MAX_X - BOUNDARY_MARGIN ||
          candidate.y < MAP_MIN_Y + BOUNDARY_MARGIN || candidate.y > MAP_MAX_Y - BOUNDARY_MARGIN) {
        continue;  // Out of bounds, try again
      }
      
      // Check for self-intersection with existing path
      if (wouldCauseIntersection(path_points, candidate)) {
        continue;  // Would intersect, try again
      }
      
      // Validate point, add it to path
      path_points.push_back(candidate);
      current_point = candidate;
      last_angle = angle;
      has_last_angle = true;
      
      // Reset attempts counter on successful addition
      attempts = 0;
    }
    
    // Convert path points to PoseStamped waypoints
    for (const auto& point : path_points) {
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.pose.position = point;
      waypoint.pose.orientation.w = 1.0;  // Identity quaternion (no specific orientation)
      new_waypoints.push_back(waypoint);
    }
    
    return new_waypoints;
  }
  
  bool WaypointPlanner::segmentsIntersect(
      const geometry_msgs::msg::Point& p1,
      const geometry_msgs::msg::Point& p2,
      const geometry_msgs::msg::Point& p3,
      const geometry_msgs::msg::Point& p4) const {
    
    // Use cross product method to determine if line segments intersect
    // Segment 1: p1 -> p2
    // Segment 2: p3 -> p4
    
    auto sign = [](double val) { return (val > 0.0) ? 1 : ((val < 0.0) ? -1 : 0); };
    
    auto cross_product = [](double ax, double ay, double bx, double by) {
      return ax * by - ay * bx;
    };
    
    // Vector from p1 to p2
    double d1x = p2.x - p1.x;
    double d1y = p2.y - p1.y;
    
    // Vector from p3 to p4
    double d2x = p4.x - p3.x;
    double d2y = p4.y - p3.y;
    
    // Check if segments are parallel (cross product near zero)
    double cross_d1_d2 = cross_product(d1x, d1y, d2x, d2y);
    if (std::abs(cross_d1_d2) < 1e-10) {
      return false;  // Parallel or collinear, treat as non-intersecting
    }
    
    // Vector from p1 to p3
    double d3x = p3.x - p1.x;
    double d3y = p3.y - p1.y;
    
    // Check if p3 and p4 are on opposite sides of line through p1-p2
    double cross1 = cross_product(d1x, d1y, d3x, d3y);
    double cross2 = cross_product(d1x, d1y, p4.x - p1.x, p4.y - p1.y);
    
    if (sign(cross1) == sign(cross2) && sign(cross1) != 0) {
      return false;  // Same side, no intersection
    }
    
    // Check if p1 and p2 are on opposite sides of line through p3-p4
    double cross3 = cross_product(d2x, d2y, -d3x, -d3y);
    double cross4 = cross_product(d2x, d2y, p2.x - p3.x, p2.y - p3.y);
    
    if (sign(cross3) == sign(cross4) && sign(cross3) != 0) {
      return false;  // Same side, no intersection
    }
    
    return true;  // Segments intersect
  }
  
  bool WaypointPlanner::wouldCauseIntersection(
      const std::vector<geometry_msgs::msg::Point>& existing_path,
      const geometry_msgs::msg::Point& new_point) const {
    
    if (existing_path.size() < 2) {
      return false;  // Need at least 2 points to form a segment
    }
    
    // New segment would be from last point to new point
    const auto& segment_start = existing_path.back();
    
    // Check if new segment intersects with any existing segment
    // Check up to size()-2 because the segment
    // that shares the starting point (the last segment)
    for (size_t i = 0; i + 1 < existing_path.size() - 1; ++i) {
      const auto& seg_p1 = existing_path[i];
      const auto& seg_p2 = existing_path[i + 1];
      
      if (segmentsIntersect(segment_start, new_point, seg_p1, seg_p2)) {
        return true;  // Intersection detected
      }
    }
    
    return false;  // No intersections
  }

}  // namespace drone_swarm
