#include "sensor_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Create and run the sensor processing node
    auto node = std::make_shared<SensorNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}