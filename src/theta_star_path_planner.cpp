#include "theta_star_path_planner.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace drone_navigation {

struct ThetaStarNodeComparator {
  bool operator()(const ThetaStarNode* a, const ThetaStarNode* b) const {
    return a->f_cost > b->f_cost; // Min-heap
  }
};

ThetaStarPathPlanner::ThetaStarPathPlanner()
{
  // Initialise empty grid (all free space for now)
  occupancy_grid_.resize(grid_height_, std::vector<bool>(grid_width_, false));
}

void ThetaStarPathPlanner::updateOccupancyGrid(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                               float drone_x, float drone_y)
{
  // Reset occupancy grid  
  for (auto& row : occupancy_grid_) {
    std::fill(row.begin(), row.end(), false);
  }

  // Process each LiDAR ray
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];
    
    // Skip invalid readings
    if (range < scan->range_min || range > scan->range_max || std::isnan(range)) {
      continue;
    }

    // Calculate ray angle
    float ray_angle = scan->angle_min + i * scan->angle_increment;
    
    // Calculate obstacle position in world coordinates
    float obstacle_x = drone_x + range * cos(ray_angle);
    float obstacle_y = drone_y + range * sin(ray_angle);
    
    // Convert to grid coordinates
    int grid_x = static_cast<int>((obstacle_x - grid_origin_x_) / grid_resolution_);
    int grid_y = static_cast<int>((obstacle_y - grid_origin_y_) / grid_resolution_);
    
    // Mark cell as occupied if within grid bounds
    if (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_) {
      occupancy_grid_[grid_y][grid_x] = true;
    }
  }
}

std::vector<std::pair<float, float>> ThetaStarPathPlanner::planPath(float start_x, float start_y, 
                                                                    float goal_x, float goal_y)
{
  geometry_msgs::msg::Point start_point, goal_point;
  start_point.x = start_x; start_point.y = start_y; start_point.z = 0;
  goal_point.x = goal_x; goal_point.y = goal_y; goal_point.z = 0;
  
  auto world_path = planPath(start_point, goal_point);
  
  std::vector<std::pair<float, float>> waypoints;
  for (const auto& point : world_path) {
    waypoints.emplace_back(point.x, point.y);
  }
  
  return waypoints;
}

std::vector<geometry_msgs::msg::Point> ThetaStarPathPlanner::planPath(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal)
{
  GridCell start_cell = worldToGrid(start);
  GridCell goal_cell = worldToGrid(goal);

  std::vector<GridCell> grid_path = planPathGrid(start_cell, goal_cell);
  return convertGridToWorldPath(grid_path);
}

GridCell ThetaStarPathPlanner::worldToGrid(const geometry_msgs::msg::Point& world_point) const
{
  GridCell cell;
  cell.x = static_cast<int>((world_point.x - grid_origin_x_) / grid_resolution_);
  cell.y = static_cast<int>((world_point.y - grid_origin_y_) / grid_resolution_);
  return cell;
}

void ThetaStarPathPlanner::gridToWorld(int grid_x, int grid_y, float& world_x, float& world_y) const
{
  world_x = grid_origin_x_ + (grid_x + 0.5f) * grid_resolution_;
  world_y = grid_origin_y_ + (grid_y + 0.5f) * grid_resolution_;
}

std::vector<geometry_msgs::msg::Point> ThetaStarPathPlanner::convertGridToWorldPath(
    const std::vector<GridCell>& grid_path) const
{
  std::vector<geometry_msgs::msg::Point> world_path;
  for (const auto& cell : grid_path) {
    geometry_msgs::msg::Point point;
    float x, y;
    gridToWorld(cell.x, cell.y, x, y);
    point.x = x;
    point.y = y;
    point.z = 1.0f; // Default height
    world_path.push_back(point);
  }
  return world_path;
}

bool ThetaStarPathPlanner::hasLineOfSight(const GridCell& from, const GridCell& to) const
{
  // Bresenham's line algorithm to check if path is clear
  int x0 = from.x, y0 = from.y;
  int x1 = to.x, y1 = to.y;
  
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  
  while (true) {
    // Check if current cell is occupied
    if (!isValidCell(x0, y0)) {
      return false;
    }
    
    if (x0 == x1 && y0 == y1) {
      break;
    }
    
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
  
  return true;
}

float ThetaStarPathPlanner::euclideanDistance(const GridCell& from, const GridCell& to) const
{
  int dx = to.x - from.x;
  int dy = to.y - from.y;
  return sqrt(dx * dx + dy * dy) * grid_resolution_;
}

std::vector<GridCell> ThetaStarPathPlanner::planPathGrid(const GridCell& start, const GridCell& goal)
{
  // Theta* implementation using priority queue and hash maps
  std::priority_queue<ThetaStarNode*, std::vector<ThetaStarNode*>, ThetaStarNodeComparator> open_set;
  std::unordered_map<GridCell, ThetaStarNode*, GridCellHash> open_map;
  std::unordered_map<GridCell, ThetaStarNode*, GridCellHash> closed_set;
  
  // Create start node
  auto start_node = new ThetaStarNode(start);
  start_node->g_cost = 0.0f;
  start_node->h_cost = calculateHeuristic(start, goal);
  start_node->calculateFCost();
  
  open_set.push(start_node);
  open_map[start] = start_node;
  
  std::vector<GridCell> path;
  
  while (!open_set.empty()) {
    // Get node with lowest f_cost
    ThetaStarNode* current = open_set.top();
    open_set.pop();
    open_map.erase(current->cell);
    
    // Add to closed set
    closed_set[current->cell] = current;
    
    // Check if we reached the goal
    if (current->cell == goal) {
      // Reconstruct path
      ThetaStarNode* node = current;
      while (node != nullptr) {
        path.push_back(node->cell);
        node = node->parent;
      }
      break;
    }
    
    // Explore neighbors
    for (const GridCell& neighbor_cell : getNeighbors(current->cell)) {
      if (!isValidCell(neighbor_cell.x, neighbor_cell.y) || 
          closed_set.count(neighbor_cell)) {
        continue;
      }
      
      auto neighbor_node = new ThetaStarNode(neighbor_cell);
      
      // Try direct path from grandparent
      if (current->parent != nullptr && 
          hasLineOfSight(current->parent->cell, neighbor_cell)) {
        // Path 1: Direct line from grandparent (any-angle)
        setPath1(current, neighbor_node, goal);
      } else {
        // Path 2: Standard A* grid-based path
        setPath2(current, neighbor_node, goal);
      }
      
      // Check if path is better
      auto open_it = open_map.find(neighbor_cell);
      if (open_it != open_map.end()) {
        if (neighbor_node->g_cost >= open_it->second->g_cost) {
          delete neighbor_node;
          continue;
        }
        delete open_it->second;
        open_map.erase(open_it);
      }
      
      open_map[neighbor_cell] = neighbor_node;
      open_set.push(neighbor_node);
    }
  }
  
  // Clean up memory
  for (auto& [cell, node] : open_map) {
    delete node;
  }
  for (auto& [cell, node] : closed_set) {
    delete node;
  }
  
  // Reverse to get path from start to goal
  std::reverse(path.begin(), path.end());
  return path;
}

void ThetaStarPathPlanner::setPath1(ThetaStarNode* current, ThetaStarNode* neighbor, const GridCell& goal)
{
  // Path 1: Direct line from grandparent
  neighbor->parent = current->parent;
  neighbor->g_cost = current->parent->g_cost + 
                    euclideanDistance(current->parent->cell, neighbor->cell);
  neighbor->h_cost = calculateHeuristic(neighbor->cell, goal);
  neighbor->calculateFCost();
}

void ThetaStarPathPlanner::setPath2(ThetaStarNode* current, ThetaStarNode* neighbor, const GridCell& goal)
{
  // Path 2: Standard grid-based path (like A*)
  neighbor->parent = current;
  neighbor->g_cost = current->g_cost + euclideanDistance(current->cell, neighbor->cell);
  neighbor->h_cost = calculateHeuristic(neighbor->cell, goal);
  neighbor->calculateFCost();
}

bool ThetaStarPathPlanner::isValidCell(int x, int y) const
{
  return (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_ && !occupancy_grid_[y][x]);
}

float ThetaStarPathPlanner::calculateHeuristic(const GridCell& from, const GridCell& to) const
{
  return euclideanDistance(from, to);
}

nav_msgs::msg::Path ThetaStarPathPlanner::waypointsToPath(
    const std::vector<std::pair<float, float>>& waypoints,
    const std::string& frame_id) const
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = frame_id;
  path_msg.header.stamp = rclcpp::Clock().now();
  
  for (const auto& waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path_msg.header;
    pose_stamped.pose.position.x = waypoint.first;
    pose_stamped.pose.position.y = waypoint.second;
    pose_stamped.pose.position.z = 1.0; // Default height
    pose_stamped.pose.orientation.w = 1.0; // No rotation
    path_msg.poses.push_back(pose_stamped);
  }
  
  return path_msg;
}

nav_msgs::msg::OccupancyGrid ThetaStarPathPlanner::getOccupancyGridMsg(const std::string& frame_id)
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = frame_id;
  grid_msg.header.stamp = rclcpp::Clock().now();
  grid_msg.info.resolution = grid_resolution_;
  grid_msg.info.width = grid_width_;
  grid_msg.info.height = grid_height_;
  grid_msg.info.origin.position.x = grid_origin_x_;
  grid_msg.info.origin.position.y = grid_origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  
  grid_msg.data.resize(grid_width_ * grid_height_);
  for (int y = 0; y < grid_height_; ++y) {
    for (int x = 0; x < grid_width_; ++x) {
      grid_msg.data[y * grid_width_ + x] = occupancy_grid_[y][x] ? 100 : 0; // 100 = occupied, 0 = free
    }
  }
  
  return grid_msg;
}

bool ThetaStarPathPlanner::isPositionFree(float x, float y) const
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = static_cast<int>((y - grid_origin_y_) / grid_resolution_);
  return isValidCell(grid_x, grid_y);
}

std::vector<GridCell> ThetaStarPathPlanner::getNeighbors(const GridCell& cell) const
{
  std::vector<GridCell> neighbors;
  
  // 8-connected neighbors (same as A*)
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      
      GridCell neighbor{cell.x + dx, cell.y + dy};
      if (isValidCell(neighbor.x, neighbor.y)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  
  return neighbors;
}

} // namespace drone_navigation