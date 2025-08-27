#include "rclcpp/rclcpp.hpp"
#include "drone_node.h"
#include "sensor_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create both nodes
    auto sensor_node = std::make_shared<SensorNode>();
    auto drone_node = std::make_shared<DroneNode>();
    
    // Connect sensor node to drone node
    drone_node->setSensorNode(sensor_node);
    
    // Create executor to run both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensor_node);
    executor.add_node(drone_node);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting integrated drone and sensor nodes...");
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in executor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}