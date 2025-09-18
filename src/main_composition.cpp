#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mission_node.h"
#include "drone_node.h"

int main(int argc, char * argv[])
{
  // Initialise ROS 2 system
  rclcpp::init(argc, argv);

  // Create single-threaded executor for efficient composed nodes
  rclcpp::executors::SingleThreadedExecutor executor;

  // Configure node options for composition with optimised intra-process communication
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  
  // Create both mission planner and drone controller nodes
  auto mission_planner = std::make_shared<drone_swarm::MissionPlannerNode>(options);
  auto drone_controller = std::make_shared<drone_swarm::DroneControllerNode>(options);

  // Add both nodes to single executor for coordinated execution
  executor.add_node(mission_planner);
  executor.add_node(drone_controller);

  // Get drone namespace for informative logging
  std::string drone_namespace = "unknown";
  try {
    drone_namespace = drone_controller->get_parameter("drone_namespace").as_string();
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("composition_main"), 
                "Could not get drone_namespace parameter: %s", e.what());
  }

  RCLCPP_INFO(rclcpp::get_logger("composition_main"), 
              "Starting composed drone nodes for %s: Mission Planner + Drone Controller", 
              drone_namespace.c_str());
  RCLCPP_INFO(rclcpp::get_logger("composition_main"),
              "Using intra-process communication for optimal performance");

  // Spin the executor (single threaded for both nodes)
  try {
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("composition_main"), 
                 "Exception in executor: %s", e.what());
  }

  RCLCPP_INFO(rclcpp::get_logger("composition_main"), 
              "Shutting down composed nodes for %s", drone_namespace.c_str());

  rclcpp::shutdown();
  return 0;
}