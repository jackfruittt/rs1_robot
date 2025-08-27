#ifndef DRONE_MANAGER_H
#define DRONE_MANAGER_H

#include "common.h"
#include "composite_drone_controller.h"
#include <map>
#include <memory>

/**
 * @brief DroneManager class for managing multiple drone instances
 * 
 * This class manages the lifecycle of multiple CompositedroneController instances,
 * each representing a drone with its sensor processor and controller.
 */
class DroneManager : public rclcpp::Node
{
public:
    DroneManager();
    ~DroneManager();

private:
    void initializeDrones();
    void shutdownDrones();
    
    // Parameters
    int num_drones_;
    
    // Map of drone ID to composite controller
    std::map<int, std::shared_ptr<CompositeDroneController>> drone_controllers_;
    
    // Service for dynamic drone management (future extension)
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr management_service_;
    
    void managementServiceCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

#endif // DRONE_MANAGER_H
