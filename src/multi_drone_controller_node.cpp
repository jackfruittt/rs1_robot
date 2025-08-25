#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <memory>
#include <string>

class MultiDroneController : public rclcpp::Node
{
public:
    MultiDroneController() : Node("multi_drone_controller")
    {
        // Get number of drones from parameter
        auto drone_num = this->declare_parameter("drone_num", 1);
        
        // Create publishers for each drone
        for (int i = 0; i < drone_num; i++) {
            std::string topic = "rs1_drone_" + std::to_string(i) + "/cmd_vel";
            vel_pubs_.push_back(
                this->create_publisher<geometry_msgs::msg::Twist>(topic, 10)
            );
        }
    }

private:
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> vel_pubs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiDroneController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}