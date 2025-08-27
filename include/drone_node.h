#ifndef DRONE_NODE_H
#define DRONE_NODE_H

#include "common.h"
#include "controller.h"
// #include "waypoint_manager.h"  // TODO: Create this file
// #include "graph.h"             // TODO: Create this file  
// #include "tsp_solver.h"        // TODO: Create this file


class SensorNode; // Forward declaration for dependencies 

/**
 * @brief DroneNode class for managing multiple drones
 * 
 * This class implements a threaded controller that can manage multiple drones
 * for terrain surveying.
 */
class DroneNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for DroneNode
     * 
     * Initialises publishers, subscribers, services, and starts the controller thread
     */
    DroneNode();

    /**
     * @brief Destructor for DroneNode
     * 
     * Ensures proper cleanup, especially for threads
     */
    ~DroneNode();
    
    /**
     * @brief Method to set sensor node reference
     * 
     * @param sensor_node Shared pointer to the sensor node
     */
    void setSensorNode(std::shared_ptr<SensorNode> sensor_node);

private:

    // Callback functions

    /**
     * @brief Callback for odometry messages
     */
    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /**
     * @brief Callback for goal messages
     */
    void goalCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    /**
     * @brief Callback for grid map messages from sensor node
     */
    void gridMapCallback(const std::shared_ptr<grid_map_msgs::msg::GridMap> msg);

    /**
     * @brief Service callback for mission control
     */
    void missionControlCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     * @brief Timer callback for mission progress updates
     */
    void timerCallback();

    /**
     * @brief Main controller thread function
     */
    void controllerThread();
    
    /**
     * @brief Validates if all goals are traversable
     * 
     * @param vehicle The vehicle to check goals for
     * @return true if all goals are traversable, false otherwise
     */
    bool validateAllGoals(VehicleData& vehicle);
    
    /**
     * @brief Return drone to origin (0,0) and land
     * 
     * Used when goals are untraversable or outside terrain bounds
     * @param vehicle Vehicle data
     */
    void returnToOriginAndLand(VehicleData& vehicle);

    /**
     * @brief Get the elevation at a specific point from the grid map
     */
    double getElevationAtPoint(const geometry_msgs::msg::Point& point);

    /**
     * @brief build the traverisibility graph
     * 
     */
    // std::unique_ptr<Graph> buildTraversabilityGraph(const VehicleData& vehicle);  // TODO: Implement Graph class


    /**
     * @brief Store current mission goals in vehicle's goals vector
     * 
     * Maintains persistent storage of goals for mission planning
     * @param vehicle Vehicle to store goals in
     * @param new_goals Vector of new goals to store
     */
    void storeMissionGoals(VehicleData& vehicle, const std::vector<geometry_msgs::msg::Pose>& new_goals);

    /**
     * @brief Solve TSP and reorder goals optimally (advanced mode only)
     * 
     * @param vehicle Vehicle containing goals to optimize
     * @return true if TSP solution found and goals reordered, false otherwise
     */
    bool solveTSPAndReorderGoals(VehicleData& vehicle);  // Implemented but disabled
    
    /**
     * @brief Check if path between two points is traversable
     * 
     * @param from Starting point
     * @param to Ending point
     * @return true if path is traversable based on terrain gradients
     */
    bool isPathTraversable(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to);
    
    /**
     * @brief Publish all valid TSP paths as visualization markers
     * 
     * @param solution TSP solution containing all valid paths
     * @param goals Original goal positions
     * @param start_position Starting position
     */
    void publishTSPPathVisualisation(
        // const TSPSolver::TSPSolution& solution,  // TODO: Fix when TSP is implemented
        const int& solution,  // Temporary placeholder
        const std::vector<geometry_msgs::msg::Pose>& goals,
        const geometry_msgs::msg::Point& start_position);

    // Subscribers, Publishers, Services, Timers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_sub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;

    sensor_msgs::msg::LaserScan::SharedPtr latest_laser_;
    sensor_msgs::msg::Range::SharedPtr latest_sonar_;
    std::mutex laser_mutex_;
    std::mutex sonar_mutex_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mission_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Controller state
    std::unique_ptr<Controller> controller_;
    // std::unique_ptr<WaypointManager> waypoint_manager_;  // TODO: Re-enable when implemented
    std::thread* controller_thread_;
    std::atomic<bool> should_terminate_;
    std::vector<std::unique_ptr<VehicleData>> vehicles_;

    // Grid map from sensor node
    grid_map_msgs::msg::GridMap latest_grid_map_;
    std::mutex grid_map_mutex_;
    bool has_grid_map_ = false;
    
    // Parameters
    double max_gradient_;  // Maximum traversable gradient in percent
    std::string drone_namespace_;  // Drone namespace for topics

    // Advanced Mode
    bool advanced_mode_;

    // Sensor node reference
    std::shared_ptr<SensorNode> sensor_node_;

    // TODO: Re-enable when implemented
    // // TSP solver for advanced mode
    // std::unique_ptr<TSPSolver> tsp_solver_;
};

#endif // DRONE_NODE_H