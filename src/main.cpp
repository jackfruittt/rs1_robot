#include <rclcpp/rclcpp.hpp>
#include "waypoint_nav_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointNavNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
