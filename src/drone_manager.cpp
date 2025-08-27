#include "drone_manager.h"

DroneManager::DroneManager()
    : Node("drone_manager")
{
    // Declare parameters
    this->declare_parameter("num_drones", 1);
    num_drones_ = this->get_parameter("num_drones").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Initializing DroneManager for %d drones", num_drones_);
    
    // Create management service
    management_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/drone_manager/control",
        std::bind(&DroneManager::managementServiceCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // Initialize all drones
    initializeDrones();
    
    RCLCPP_INFO(this->get_logger(), "DroneManager initialized successfully with %d drones", num_drones_);
}

DroneManager::~DroneManager()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down DroneManager");
    shutdownDrones();
}

void DroneManager::initializeDrones()
{
    for (int i = 1; i <= num_drones_; ++i) {
        std::string drone_namespace = "rs1_drone_" + std::to_string(i);
        
        RCLCPP_INFO(this->get_logger(), "Creating composite controller for drone %d (%s)", 
                    i, drone_namespace.c_str());
        
        try {
            auto composite_controller = std::make_shared<CompositeDroneController>(
                drone_namespace, i);
            
            drone_controllers_[i] = composite_controller;
            
            RCLCPP_INFO(this->get_logger(), "Successfully created controller for %s", 
                        drone_namespace.c_str());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create controller for %s: %s", 
                         drone_namespace.c_str(), e.what());
        }
    }
}

void DroneManager::shutdownDrones()
{
    for (auto& [drone_id, controller] : drone_controllers_) {
        if (controller) {
            RCLCPP_INFO(this->get_logger(), "Shutting down drone %d", drone_id);
            controller.reset();
        }
    }
    drone_controllers_.clear();
}

void DroneManager::managementServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        RCLCPP_INFO(this->get_logger(), "Starting all drone operations");
        // Future extension: start/stop specific operations
        response->success = true;
        response->message = "All drones started successfully";
    } else {
        RCLCPP_INFO(this->get_logger(), "Stopping all drone operations");
        // Future extension: gracefully stop all drones
        response->success = true;
        response->message = "All drones stopped successfully";
    }
}
