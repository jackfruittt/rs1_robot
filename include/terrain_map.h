#ifndef TERRAIN_MAP_H
#define TERRAIN_MAP_H

#include <set>
#include "common.h"

// Index comparator for std::set
struct IndexComparator {
    bool operator()(const grid_map::Index& a, const grid_map::Index& b) const {
        if (a(0) != b(0)) return a(0) < b(0);
        return a(1) < b(1);
    }
};

/**
 * @brief TerrainMap class for incremental 1x1m cell-by-cell terrain mapping
 * 
 * This class handles drone-centric grid map operations:
 * - Starts with 1x1m grid centred on drone
 * - Expands by single 1m cells in flight direction
 * - Each cell represents 1m x 1m of terrain
 */
class TerrainMap {
public:
    /**
     * @brief Constructor for TerrainMap
     * 
     * @param logger ROS logger instance
     * @param resolution Map resolution in metres per cell (should be 1.0 for 1x1m cells)
     * @param initial_size Initial map size in metres (should be 1.0 for 1x1m start)
     */
    TerrainMap(const rclcpp::Logger& logger, 
               double resolution = 0.1,        // 0.1m per cell for high resolution mapping
               double initial_size = 1.0);     // Start with 1x1m
    
    ~TerrainMap() = default;

    // Core elevation mapping
    bool updateElevation(double x, double y, double elevation);
    double getElevationAtPoint(double x, double y) const;
    
    // Gradient analysis
    double getGradientMagnitudeAtPoint(double x, double y) const;
    void calculateGradientsAroundPoint(double x, double y, double radius = 1.0);
    
    // Incremental map expansion
    void checkAndExpandForPosition(double x, double y);
    bool isPositionInMap(double x, double y) const;
    
    // Data access
    void getGridMap(grid_map::GridMap& map) const;
    grid_map_msgs::msg::GridMap getGridMapMessage() const;
    
    // Visualisation
    visualization_msgs::msg::MarkerArray getGradientArrows() const;
    
    // Statistics and debugging
    void logStatistics() const;
    void logMapBounds() const;
    size_t getActiveAreasCount() const;
    
    // Map info
    grid_map::Position getMapCentre() const;
    grid_map::Length getMapSize() const;

    // Configuration (kept for compatibility)
    void setExpansionParameters(double margin, double threshold, double max_size);
    void setDirectionalExpansion(bool enabled);

private:
    // Core map operations
    void initialiseMap(double start_x = 0.0, double start_y = 0.0);
    void expandMapByOneCell(double target_x, double target_y);
    bool needsExpansion(double x, double y) const;
    void markActiveArea(double x, double y);
    void calculateGradients();
    void cleanupActiveAreas();
    
    // Thread safety
    mutable std::mutex map_mutex_;
    
    // Map data
    grid_map::GridMap grid_map_;
    std::set<grid_map::Index, IndexComparator> active_cells_;
    
    // Logger
    rclcpp::Logger logger_;
    
    // Configuration parameters
    double map_resolution_;     // Should be 1.0 (1m per cell)
    double cell_size_;         // Size of each cell in metres
    
    // Track initial position for relative expansion
    double initial_x_;
    double initial_y_;
    bool map_initialised_;
};

#endif // TERRAIN_MAP_H