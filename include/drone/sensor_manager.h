/**
 * @file sensor_manager.h
 * @brief Sensor management class for drone operations.
 * @author Jackson Russell
 * OTHER AUTHORS ADD HERE AND BELOW
 * @date August-2025
 */

#ifndef DRONE_SWARM_SENSOR_MANAGER_
#define DRONE_SWARM_SENSOR_MANAGER_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <mutex>
#include <chrono>

namespace drone_swarm {
    
    class SensorManager {
    public:
    SensorManager();
    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // UNCOMMENT IF IMPLEMENTING IMU AND GPS
    // void updateIMU(const sensor_msgs::msg::Imu::SharedPtr msg);
    // void updateGPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    
    void updateLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void updateSonar(const sensor_msgs::msg::Range::SharedPtr msg);

    // Shared pointer functions to sensor data
    geometry_msgs::msg::PoseStamped getCurrentPose() const;
    geometry_msgs::msg::Twist getCurrentVelocity() const;
    sensor_msgs::msg::LaserScan::SharedPtr getLatestLaser() const;
    sensor_msgs::msg::Range::SharedPtr getLatestSonar() const;

    // Validate sensor data
    bool hasValidPose() const;
    bool hasValidOdom() const;
    
    // UNCOMMENT FOR GPS
    // bool hasValidGPS() const;

    private:
    
    // Shared pointers
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    nav_msgs::msg::Odometry latest_odom_;
    // sensor_msgs::msg::Imu latest_imu_;
    // sensor_msgs::msg::NavSatFix latest_gps_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_laser_;
    sensor_msgs::msg::Range::SharedPtr latest_sonar_;
    
    // Thread mutexes
    mutable std::mutex laser_mutex_;
    mutable std::mutex sonar_mutex_;
    
    bool pose_valid_;
    bool odom_valid_;
    bool gps_valid_;
    std::chrono::steady_clock::time_point last_pose_update_;
    std::chrono::steady_clock::time_point last_odom_update_;
    };
}

#endif  // DRONE_SWARM_SENSOR_MANAGER_