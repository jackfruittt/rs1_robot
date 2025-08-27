#ifndef COMMON_H
#define COMMON_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <cmath>
#include <random>
#include <thread>

// ROS2 core
#include "rclcpp/rclcpp.hpp"

// Message types
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"

// Service types
#include "std_srvs/srv/set_bool.hpp"

// TF2 libraries
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Grid Map
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

// Local
#include "laserprocessing.h"

/**
 * @brief Vehicle data structure to store state of each drone
 */
struct VehicleData {
    std::string id;
    nav_msgs::msg::Odometry current_odom;
    std::vector<geometry_msgs::msg::Pose> goals;
    std::vector<geometry_msgs::msg::Pose> traversable_waypoints;
    geometry_msgs::msg::Pose start_position;
    std::unique_ptr<LaserProcessing> laser_processor;
    std::atomic<bool> mission_active{false};
    std::atomic<bool> takeoff_complete{false};
    std::atomic<int> current_goal_index{0};
    std::atomic<double> mission_progress{0.0};
    std::atomic<bool> avoiding_obstacle{false};
    std::atomic<double> obstacle_avoidance_start_altitude{0.0};
    
    std::mutex odom_mutex;
    std::mutex goals_mutex;
    
    // Command queue for drone
    struct Command {
        enum class Type {
            TAKEOFF,
            LANDING,
            FLYING,
            STOP
        };
        
        Type type;
        geometry_msgs::msg::Pose goal;  // Used for FLYING
        
        // Default constructor
        Command() : type(Type::STOP), goal() {}
        
        // Constructor with only type
        explicit Command(Type t) : type(t), goal() {}
        
        // Constructor with type and goal
        Command(Type t, const geometry_msgs::msg::Pose& g) : type(t), goal(g) {}
    };
    
    std::queue<Command> command_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    
    VehicleData(const std::string& id_) : id(id_) {}
};

/**
 * @brief Cell in elevation map
 */
struct MapCell {
    double elevation;
    bool has_data;
    
    MapCell(double elev = 0.0, bool has_d = false)
        : elevation(elev), has_data(has_d) {}
};

#endif // COMMON_H