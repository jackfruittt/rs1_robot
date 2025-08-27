#include "terrain_map.h"
#include <cmath>
#include <algorithm>

TerrainMap::TerrainMap(const rclcpp::Logger& logger, double resolution, double /* initial_size */)
    : logger_(logger),
      map_resolution_(resolution),
      cell_size_(resolution),
      initial_x_(0.0),
      initial_y_(0.0),
      map_initialised_(false)
{
    RCLCPP_INFO(logger_, "Initialising TerrainMap for 1x1m incremental expansion (resolution=%.1fm)", 
                resolution);
    // Map will be initialised when first drone position is received
}

void TerrainMap::initialiseMap(double start_x, double start_y) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (map_initialised_) {
        return;  // Already initialised
    }
    
    // Store initial position
    initial_x_ = start_x;
    initial_y_ = start_y;
    
    // Create initial grid centred on drone's starting position
    // Start with 1m x 1m physical size regardless of resolution
    double initial_physical_size = 1.0;  // Always 1m x 1m
    
    grid_map_.setFrameId("world");
    grid_map_.setGeometry(grid_map::Length(initial_physical_size, initial_physical_size), map_resolution_);
    grid_map_.setPosition(grid_map::Position(start_x, start_y));
    
    // Add required layers
    grid_map_.add("elevation", 0.0);
    grid_map_.add("variance", 0.0);
    grid_map_.add("upper_bound", 0.0);
    grid_map_.add("num_measurements", 0.0);
    grid_map_.add("gradient_magnitude", 0.0);
    grid_map_.add("gradient_x", 0.0);
    grid_map_.add("gradient_y", 0.0);
    
    map_initialised_ = true;
    
    RCLCPP_INFO(logger_, "TerrainMap initialised at (%.2f, %.2f) with %.1fm x %.1fm grid (%d x %d cells)", 
                start_x, start_y, initial_physical_size, initial_physical_size,
                static_cast<int>(initial_physical_size / map_resolution_),
                static_cast<int>(initial_physical_size / map_resolution_));
    logMapBounds();
}

bool TerrainMap::updateElevation(double x, double y, double elevation) {
    // Initialise map if this is the first update
    if (!map_initialised_) {
        initialiseMap(x, y);
    }
    
    // Check if we need to expand the map
    checkAndExpandForPosition(x, y);
    
    // Validate input
    if (std::isnan(elevation) || std::isinf(elevation) || 
        elevation < -100.0 || elevation > 1000.0) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    try {
        grid_map::Position position(x, y);
        grid_map::Index index;
        
        if (!grid_map_.getIndex(position, index)) {
            RCLCPP_WARN(logger_, "Position (%.2f, %.2f) outside map bounds", x, y);
            return false;
        }
        
        // Update elevation using Kalman filter approach
        if (grid_map_.at("num_measurements", index) < 1.0) {
            // First measurement
            grid_map_.at("elevation", index) = elevation;
            grid_map_.at("variance", index) = 0.01;
            grid_map_.at("upper_bound", index) = elevation + 3 * std::sqrt(0.01);
            grid_map_.at("num_measurements", index) = 1.0;
            
            RCLCPP_DEBUG(logger_, "First elevation measurement at (%.2f, %.2f): %.2fm", 
                        x, y, elevation);
        } else {
            // Update existing measurement
            double current_elevation = grid_map_.at("elevation", index);
            double variance = grid_map_.at("variance", index);
            
            double k = variance / (variance + 0.01);
            double new_elevation = current_elevation + k * (elevation - current_elevation);
            double new_variance = (1 - k) * variance;
            
            grid_map_.at("elevation", index) = new_elevation;
            grid_map_.at("variance", index) = new_variance;
            grid_map_.at("upper_bound", index) = new_elevation + 3 * std::sqrt(new_variance);
            grid_map_.at("num_measurements", index) += 1.0;
        }
        
        // Mark this area as active for gradient calculation
        markActiveArea(x, y);
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "Error updating elevation at (%.2f, %.2f): %s", x, y, e.what());
        return false;
    }
}

void TerrainMap::checkAndExpandForPosition(double x, double y) {
    if (!map_initialised_) {
        return;
    }
    
    if (needsExpansion(x, y)) {
        RCLCPP_INFO(logger_, "Expanding map for drone position (%.2f, %.2f)", x, y);
        expandMapByOneCell(x, y);
    }
}

bool TerrainMap::needsExpansion(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    grid_map::Position pos(x, y);
    return !grid_map_.isInside(pos);
}

void TerrainMap::expandMapByOneCell(double target_x, double target_y) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // Get current map bounds
    grid_map::Position current_centre = grid_map_.getPosition();
    grid_map::Length current_length = grid_map_.getLength();
    
    // Calculate current bounds
    double min_x = current_centre.x() - current_length.x() / 2.0;
    double max_x = current_centre.x() + current_length.x() / 2.0;
    double min_y = current_centre.y() - current_length.y() / 2.0;
    double max_y = current_centre.y() + current_length.y() / 2.0;
    
    RCLCPP_DEBUG(logger_, "Current bounds: X[%.2f to %.2f], Y[%.2f to %.2f]", 
                min_x, max_x, min_y, max_y);
    RCLCPP_DEBUG(logger_, "Target position: (%.2f, %.2f)", target_x, target_y);
    
    // Determine expansion direction and new bounds
    // Expand by exactly 1.0m in the appropriate direction, EDIT: Doesn't do L-Shape stuff will expand in the y-axis 1m along total x-legnth and vice versa.  
    double new_min_x = min_x, new_max_x = max_x;
    double new_min_y = min_y, new_max_y = max_y;
    
    if (target_x < min_x) {
        new_min_x = min_x - 1.0;  // Expand west by 1m
        RCLCPP_INFO(logger_, "Expanding WEST by 1m: new_min_x = %.2f", new_min_x);
    }
    if (target_x > max_x) {
        new_max_x = max_x + 1.0;  // Expand east by 1m
        RCLCPP_INFO(logger_, "Expanding EAST by 1m: new_max_x = %.2f", new_max_x);
    }
    if (target_y < min_y) {
        new_min_y = min_y - 1.0;  // Expand south by 1m
        RCLCPP_INFO(logger_, "Expanding SOUTH by 1m: new_min_y = %.2f", new_min_y);
    }
    if (target_y > max_y) {
        new_max_y = max_y + 1.0;  // Expand north by 1m
        RCLCPP_INFO(logger_, "Expanding NORTH by 1m: new_max_y = %.2f", new_max_y);
    }
    
    // Calculate new map properties
    double new_length_x = new_max_x - new_min_x;
    double new_length_y = new_max_y - new_min_y;
    double new_centre_x = (new_min_x + new_max_x) / 2.0;
    double new_centre_y = (new_min_y + new_max_y) / 2.0;
    
    RCLCPP_INFO(logger_, "New map: %.1fm x %.1fm centred at (%.2f, %.2f)", 
                new_length_x, new_length_y, new_centre_x, new_centre_y);
    
    // Create expanded map
    grid_map::GridMap expanded_map;
    expanded_map.setFrameId("world");
    expanded_map.setGeometry(grid_map::Length(new_length_x, new_length_y), map_resolution_);
    expanded_map.setPosition(grid_map::Position(new_centre_x, new_centre_y));
    
    // Add all layers
    for (const auto& layer : grid_map_.getLayers()) {
        expanded_map.add(layer, 0.0);
    }
    
    // Copy existing data
    int copied_cells = 0;
    for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index old_index(*iterator);
        grid_map::Position position;
        
        if (grid_map_.getPosition(old_index, position)) {
            grid_map::Index new_index;
            if (expanded_map.getIndex(position, new_index)) {
                for (const auto& layer : grid_map_.getLayers()) {
                    if (grid_map_.isValid(old_index, layer)) {
                        expanded_map.at(layer, new_index) = grid_map_.at(layer, old_index);
                    }
                }
                copied_cells++;
            }
        }
    }
    
    // Update active cells mapping
    std::set<grid_map::Index, IndexComparator> new_active_cells;
    for (const auto& old_index : active_cells_) {
        grid_map::Position position;
        if (grid_map_.getPosition(old_index, position)) {
            grid_map::Index new_index;
            if (expanded_map.getIndex(position, new_index)) {
                new_active_cells.insert(new_index);
            }
        }
    }
    
    // Replace old map
    grid_map_ = std::move(expanded_map);
    active_cells_ = std::move(new_active_cells);
    
    RCLCPP_INFO(logger_, "Map expanded to %.1fm x %.1fm, copied %d cells", 
                new_length_x, new_length_y, copied_cells);
    
    logMapBounds();
}

double TerrainMap::getElevationAtPoint(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!map_initialised_ || !grid_map_.exists("elevation")) {
        return 0.0;
    }
    
    try {
        grid_map::Position position(x, y);
        
        if (!grid_map_.isInside(position)) {
            return 0.0;
        }
        
        grid_map::Index index;
        if (grid_map_.getIndex(position, index) && grid_map_.isValid(index, "elevation")) {
            return grid_map_.at("elevation", index);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "Error getting elevation at (%.2f, %.2f): %s", x, y, e.what());
    }
    
    return 0.0;
}

double TerrainMap::getGradientMagnitudeAtPoint(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!map_initialised_ || !grid_map_.exists("gradient_magnitude")) {
        return 0.0;
    }
    
    try {
        grid_map::Position position(x, y);
        
        if (!grid_map_.isInside(position)) {
            return 0.0;
        }
        
        grid_map::Index index;
        if (grid_map_.getIndex(position, index) && grid_map_.isValid(index, "gradient_magnitude")) {
            return grid_map_.at("gradient_magnitude", index);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "Error getting gradient at (%.2f, %.2f): %s", x, y, e.what());
    }
    
    return 0.0;
}

void TerrainMap::calculateGradientsAroundPoint(double x, double y, double /* radius */) {
    markActiveArea(x, y);
    calculateGradients();
}

void TerrainMap::calculateGradients() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!map_initialised_ || active_cells_.empty()) {
        return;
    }
    
    int gradients_calculated = 0;
    
    for (const auto& index : active_cells_) {
        if (!grid_map_.isValid(index, "elevation")) {
            continue;
        }
        
        // Calculate gradients using finite differences
        double gradient_x = 0.0, gradient_y = 0.0;
        
        // Get neighbouring indices
        grid_map::Index index_left = index;
        grid_map::Index index_right = index;
        grid_map::Index index_up = index;
        grid_map::Index index_down = index;
        
        // Move indices with bounds checking
        if (index(1) > 0) index_left(1) -= 1;
        if (index(1) < grid_map_.getSize()(1) - 1) index_right(1) += 1;
        if (index(0) > 0) index_up(0) -= 1;
        if (index(0) < grid_map_.getSize()(0) - 1) index_down(0) += 1;
        
        // Calculate X gradient
        if (grid_map_.isValid(index_left, "elevation") && 
            grid_map_.isValid(index_right, "elevation")) {
            double height_left = grid_map_.at("elevation", index_left);
            double height_right = grid_map_.at("elevation", index_right);
            gradient_x = (height_right - height_left) / (2.0 * map_resolution_);
        }
        
        // Calculate Y gradient
        if (grid_map_.isValid(index_up, "elevation") && 
            grid_map_.isValid(index_down, "elevation")) {
            double height_up = grid_map_.at("elevation", index_up);
            double height_down = grid_map_.at("elevation", index_down);
            gradient_y = (height_down - height_up) / (2.0 * map_resolution_);
        }
        
        // Calculate magnitude
        double gradient_magnitude = std::sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
        
        // Validate and store
        if (!std::isnan(gradient_magnitude) && !std::isinf(gradient_magnitude)) {
            grid_map_.at("gradient_x", index) = gradient_x;
            grid_map_.at("gradient_y", index) = gradient_y;
            grid_map_.at("gradient_magnitude", index) = gradient_magnitude;
            gradients_calculated++;
        }
    }
    
    RCLCPP_DEBUG(logger_, "Calculated gradients for %d cells", gradients_calculated);
}

void TerrainMap::markActiveArea(double x, double y) {
    if (!map_initialised_) return;
    
    grid_map::Position pos(x, y);
    grid_map::Index index;
    
    if (grid_map_.isInside(pos) && grid_map_.getIndex(pos, index)) {
        active_cells_.insert(index);
    }
}

bool TerrainMap::isPositionInMap(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!map_initialised_) return false;
    
    grid_map::Position position(x, y);
    return grid_map_.isInside(position);
}

void TerrainMap::getGridMap(grid_map::GridMap& map) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_initialised_) {
        map = grid_map_;
    }
}

grid_map_msgs::msg::GridMap TerrainMap::getGridMapMessage() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!map_initialised_) {
        RCLCPP_DEBUG(logger_, "TerrainMap not initialised - returning empty grid map message");
        return grid_map_msgs::msg::GridMap();
    }
    
    try {
        // Create a copy of the grid map for publishing
        grid_map::GridMap publish_map = grid_map_;
        
        // Ensure all cells have valid data for visualisation
        for (grid_map::GridMapIterator iterator(publish_map); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index index(*iterator);
            
            // Fill empty elevation cells with 0.0 for better visualisation
            if (!publish_map.isValid(index, "elevation")) {
                publish_map.at("elevation", index) = 0.0;
            }
            
            // Fill empty gradient cells
            if (!publish_map.isValid(index, "gradient_magnitude")) {
                publish_map.at("gradient_magnitude", index) = 0.0;
            }
        }
        
        // Set timestamp for RViz
        publish_map.setTimestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        
        auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(publish_map);
        
        RCLCPP_DEBUG(logger_, "Publishing grid map: %.1fm x %.1fm with %zu layers",
                    publish_map.getLength().x(), publish_map.getLength().y(),
                    publish_map.getLayers().size());
        
        return *grid_map_msg;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error creating grid map message: %s", e.what());
        return grid_map_msgs::msg::GridMap();
    }
}

void TerrainMap::logStatistics() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!map_initialised_) {
        RCLCPP_INFO(logger_, "TerrainMap not yet initialised");
        return;
    }
    
    grid_map::Position centre = grid_map_.getPosition();
    grid_map::Length length = grid_map_.getLength();
    
    RCLCPP_INFO(logger_, "TerrainMap Statistics:");
    RCLCPP_INFO(logger_, "  Size: %.1fm x %.1fm (%.0f x %.0f cells)", 
                length.x(), length.y(), 
                length.x() / map_resolution_, length.y() / map_resolution_);
    RCLCPP_INFO(logger_, "  Centre: (%.1f, %.1f)", centre.x(), centre.y());
    RCLCPP_INFO(logger_, "  Active areas: %zu", active_cells_.size());
}

void TerrainMap::logMapBounds() const {
    if (!map_initialised_) return;
    
    grid_map::Position centre = grid_map_.getPosition();
    grid_map::Length length = grid_map_.getLength();
    
    double min_x = centre.x() - length.x() / 2.0;
    double max_x = centre.x() + length.x() / 2.0;
    double min_y = centre.y() - length.y() / 2.0;
    double max_y = centre.y() + length.y() / 2.0;
    
    RCLCPP_DEBUG(logger_, "Map bounds: X[%.1f to %.1f], Y[%.1f to %.1f]", 
                min_x, max_x, min_y, max_y);
}

size_t TerrainMap::getActiveAreasCount() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return active_cells_.size();
}

grid_map::Position TerrainMap::getMapCentre() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_initialised_) {
        return grid_map_.getPosition();
    }
    return grid_map::Position(0.0, 0.0);
}

grid_map::Length TerrainMap::getMapSize() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_initialised_) {
        return grid_map_.getLength();
    }
    return grid_map::Length(0.0, 0.0);
}

visualization_msgs::msg::MarkerArray TerrainMap::getGradientArrows() const {
    visualization_msgs::msg::MarkerArray marker_array;
    // Implementation for gradient visualisation
    return marker_array;
}

// Compatibility methods (kept for existing code)
void TerrainMap::setExpansionParameters(double /* margin */, double /* threshold */, double /* max_size */) {
    // No longer used in incremental expansion, kept for compatibility
    RCLCPP_DEBUG(logger_, "setExpansionParameters called (no effect in incremental mode)");
}

void TerrainMap::setDirectionalExpansion(bool /* enabled */) {
    // Always enabled in incremental mode, kept for compatibility
    RCLCPP_DEBUG(logger_, "setDirectionalExpansion called (always enabled in incremental mode)");
}

void TerrainMap::cleanupActiveAreas() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (active_cells_.size() > 1000) {
        // Keep only recent areas - implement cleanup logic here, EDIT: No need to do
        RCLCPP_INFO(logger_, "Cleaning up %zu active areas", active_cells_.size());
    }
}