/**
 * @file theta_star_path_planner.h
 * @brief Theta* path planner implementation for optimal any-angle pathfinding
 * 
 * This file contains the Theta* pathfinding algorithm, which extends A* to support
 * any-angle pathfinding. Unlike traditional A* that is constrained to grid movements,
 * Theta* can create direct paths between any two points when line-of-sight exists,
 * resulting in geometrically optimal paths without staircase patterns.
 * 
 * 
 * @author Jackson Russell
 * @date October 2025
 */

#ifndef THETA_STAR_PATH_PLANNER_H
#define THETA_STAR_PATH_PLANNER_H

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace drone_navigation {

/**
 * @brief Represents a cell in the discrete grid coordinate system
 * 
 * Grid cells are used internally by the Theta* algorithm to discretise
 * the continuous world space into a searchable grid structure.
 */
struct GridCell {
  int x; ///< X coordinate in grid space
  int y; ///< Y coordinate in grid space
  
  /**
   * @brief Equality operator for grid cell comparison
   * @param other The other grid cell to compare against
   * @return True if both x and y coordinates match
   */
  bool operator==(const GridCell& other) const {
    return x == other.x && y == other.y;
  }
};

/**
 * @brief Hash function for GridCell to enable use in unordered containers
 * 
 * Simple hash combining function to create unique hashes for grid cells.
 * Required for std::unordered_map and std::unordered_set usage.
 */
struct GridCellHash {
  /**
   * @brief Calculate hash value for a grid cell
   * @param cell The grid cell to hash
   * @return Hash value suitable for unordered containers
   */
  std::size_t operator()(const GridCell& cell) const {
    return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
  }
};

/**
 * @brief Node structure for Theta* algorithm pathfinding
 * 
 * Represents a single node in the Theta* search space, containing
 * cost information and parent relationships for path reconstruction.
 */
struct ThetaStarNode {
  GridCell cell;                    ///< Grid position of this node
  float g_cost = 0.0f;             ///< Actual cost from start node (metres)
  float h_cost = 0.0f;             ///< Heuristic cost estimate to goal (metres)
  float f_cost = 0.0f;             ///< Total estimated cost (g_cost + h_cost)
  ThetaStarNode* parent = nullptr; ///< Parent node for path reconstruction

  /**
   * @brief Construct new Theta* node
   * @param c Grid cell coordinates for this node
   */
  ThetaStarNode(const GridCell& c) : cell(c) {}
  
  /**
   * @brief Calculate the total F-cost for this node
   * 
   * Updates f_cost based on current g_cost and h_cost values.
   * Used by the priority queue to determine search order.
   */
  void calculateFCost() {
    f_cost = g_cost + h_cost;
  }
};

class ThetaStarPathPlanner {
public:
  /**
   * @brief Construct a new Theta* path planner
   * 
   * Initialises the planner with default grid parameters:
   * - 200x200 grid cells
   * - 0.1m resolution per cell  
   * - 20x20m coverage area
   * - Origin at (-10, -10)
   */
  ThetaStarPathPlanner();
  
  /**
   * @brief Default destructor
   */
  ~ThetaStarPathPlanner() = default;

  /////////////////////////// LiDAR Integration Functions /////////////////////////////////////
  
  /**
   * @brief Update occupancy grid from LiDAR scan data
   * 
   * Processes LiDAR scan data to update the internal occupancy grid
   * used for obstacle detection during pathfinding.
   * 
   * @param scan Shared pointer to LiDAR scan message
   * @param drone_x Current drone X position in world coordinates (metres)
   * @param drone_y Current drone Y position in world coordinates (metres)
   */
  void updateOccupancyGrid(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                           float drone_x, float drone_y);

  /////////////////////////// Main Pathfinding Functions /////////////////////////////////////
  
  /**
   * @brief Plan optimal path between two points (coordinate version)
   * 
   * Computes an optimal path from start to goal using the Theta* algorithm.
   * Returns waypoints as coordinate pairs suitable for direct navigation.
   * 
   * @param start_x Start X coordinate in world frame (metres)
   * @param start_y Start Y coordinate in world frame (metres)
   * @param goal_x Goal X coordinate in world frame (metres)
   * @param goal_y Goal Y coordinate in world frame (metres)
   * @return Vector of waypoint coordinate pairs (x, y) in metres
   */
  std::vector<std::pair<float, float>> planPath(float start_x, float start_y, 
                                                float goal_x, float goal_y);
  
  /**
   * @brief Plan optimal path between two points (ROS message version)
   * 
   * Computes an optimal path using ROS geometry messages for input/output.
   * Convenient for integration with ROS navigation stack.
   * 
   * @param start Start position as ROS Point message
   * @param goal Goal position as ROS Point message  
   * @return Vector of waypoints as ROS Point messages
   */
  std::vector<geometry_msgs::msg::Point> planPath(
      const geometry_msgs::msg::Point& start,
      const geometry_msgs::msg::Point& goal);

  /////////////////////////// Coordinate System Functions /////////////////////////////////////
  
  /**
   * @brief Convert world coordinates to grid cell indices
   * 
   * @param world_point 3D point in world coordinates (Z ignored)
   * @return Grid cell coordinates for internal pathfinding
   */
  GridCell worldToGrid(const geometry_msgs::msg::Point& world_point) const;
  
  /**
   * @brief Convert grid cell indices to world coordinates
   * 
   * @param grid_x X coordinate in grid space
   * @param grid_y Y coordinate in grid space 
   * @param world_x Output X coordinate in world space (metres)
   * @param world_y Output Y coordinate in world space (metres)
   */
  void gridToWorld(int grid_x, int grid_y, float& world_x, float& world_y) const;
  
  /**
   * @brief Convert grid-based path to world coordinate waypoints
   * 
   * @param grid_path Vector of grid cells representing the path
   * @return Vector of world coordinate waypoints as ROS Points
   */
  std::vector<geometry_msgs::msg::Point> convertGridToWorldPath(
      const std::vector<GridCell>& grid_path) const;

/////////////////////////// ROS /////////////////////////////////////  
 
  /**
   * @brief Convert waypoint coordinates to ROS Path message
   * 
   * @param waypoints Vector of waypoint coordinate pairs (x, y)
   * @param frame_id Frame identifier for the path (e.g., "map")
   * @return ROS Path message suitable for visualisation and navigation
   */
  nav_msgs::msg::Path waypointsToPath(
      const std::vector<std::pair<float, float>>& waypoints,
      const std::string& frame_id) const;
  
  /**
   * @brief Get current occupancy grid as ROS message
   * 
   * @param frame_id Frame identifier for the occupancy grid
   * @return ROS OccupancyGrid message for visualisation
   */
  nav_msgs::msg::OccupancyGrid getOccupancyGridMsg(const std::string& frame_id);
  
  /**
   * @brief Check if a world position is free of obstacles
   * 
   * @param x X coordinate in world frame (metres)
   * @param y Y coordinate in world frame (metres)
   * @return True if position is free, false if occupied or out of bounds
   */
  bool isPositionFree(float x, float y) const;

  // === Core Theta* Algorithm Functions ===
  
  /**
   * @brief Execute Theta* pathfinding on grid coordinates
   * 
   * Core implementation of the Theta* algorithm that performs the actual
   * pathfinding using any-angle optimisation techniques.
   * 
   * @param start Starting grid cell
   * @param goal Goal grid cell
   * @return Vector of grid cells representing the optimal path
   */
  std::vector<GridCell> planPathGrid(const GridCell& start, const GridCell& goal);
  
  /**
   * @brief Check line-of-sight between two grid cells
   * 
   * Uses Bresenham's line algorithm to determine if a direct path exists
   * between two cells without intersecting obstacles.
   * 
   * @param from Starting grid cell
   * @param to Ending grid cell  
   * @return True if clear line-of-sight exists, false otherwise
   */
  bool hasLineOfSight(const GridCell& from, const GridCell& to) const;
  
  /**
   * @brief Calculate Euclidean distance between grid cells
   * 
   * @param from Starting grid cell
   * @param to Ending grid cell
   * @return Euclidean distance in metres
   */
  float euclideanDistance(const GridCell& from, const GridCell& to) const;

private:
  /////////////////////////// Grid Config Params /////////////////////////////////////
  
  int grid_width_ = 200;           ///< Grid width in cells (covers 20m at 0.1m resolution)
  int grid_height_ = 200;          ///< Grid height in cells (covers 20m at 0.1m resolution)  
  float grid_resolution_ = 0.1f;   ///< Metres per grid cell (0.1m = 10cm resolution)
  float grid_origin_x_ = -10.0f;   ///< World X coordinate of grid bottom-left corner (metres)
  float grid_origin_y_ = -10.0f;   ///< World Y coordinate of grid bottom-left corner (metres)

  /////////////////////////// Internal Data Structures /////////////////////////////////////
  
  /**
   * @brief 2D occupancy grid for obstacle representation
   * 
   * Grid where false = free space, true = occupied by obstacle.
   * Updated dynamically from LiDAR scan data.
   */
  std::vector<std::vector<bool>> occupancy_grid_;

  /////////////////////////// Private Helper Functions /////////////////////////////////////
  
  /**
   * @brief Validate grid cell coordinates are within bounds and unoccupied
   * 
   * @param x Grid X coordinate
   * @param y Grid Y coordinate  
   * @return True if cell is valid and free, false otherwise
   */
  bool isValidCell(int x, int y) const;
  
  /**
   * @brief Calculate heuristic cost estimate between grid cells
   * 
   * Uses Euclidean distance as the heuristic for the Theta* algorithm.
   * 
   * @param from Starting grid cell
   * @param to Goal grid cell
   * @return Heuristic cost estimate in metres
   */
  float calculateHeuristic(const GridCell& from, const GridCell& to) const;
  
  /**
   * @brief Get valid neighbouring cells for pathfinding expansion
   * 
   * Returns 8-connected neighbours (N, S, E, W, NE, NW, SE, SW) that
   * are within grid bounds and not occupied by obstacles.
   * 
   * @param cell Current grid cell
   * @return Vector of valid neighbouring cells
   */
  std::vector<GridCell> getNeighbors(const GridCell& cell) const;
  
  /**
   * @brief Set path costs using direct line from grandparent (Theta* optimisation)
   * 
   * Instead of following grid edges,
   * attempt to create a direct path from the grandparent node.
   * 
   * @param current Current node being processed
   * @param neighbor Neighbouring node to update
   * @param goal Goal cell for heuristic calculation
   */
  void setPath1(ThetaStarNode* current, ThetaStarNode* neighbor, const GridCell& goal);
  
  /**
   * @brief Set path costs using standard grid-based path (fallback grid behaviour)
   * 
   * Used when direct line-of-sight to grandparent is blocked by obstacles.
   * 
   * @param current Current node being processed  
   * @param neighbor Neighbouring node to update
   * @param goal Goal cell for heuristic calculation
   */
  void setPath2(ThetaStarNode* current, ThetaStarNode* neighbor, const GridCell& goal);
}; 

} // namespace drone_navigation

#endif // THETA_STAR_PATH_PLANNER_H