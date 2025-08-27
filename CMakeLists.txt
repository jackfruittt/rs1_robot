cmake_minimum_required(VERSION 3.8)
project(rs1_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

# Include directories
include_directories(include)

# Create a library for common code
add_library(rs1_robot_common
    src/common.cpp
    src/laserprocessing.cpp
    src/terrain_map.cpp
    src/sensor_node.cpp
    src/controller.cpp
)

# Target dependencies for the common library
ament_target_dependencies(rs1_robot_common
    rclcpp
    std_srvs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    grid_map_msgs
    grid_map_ros
)

# Add executables for drone operation
add_executable(drone_controller
    src/drone_main.cpp
    src/drone_node.cpp
)

add_executable(sensor_processor_node
    src/sensor_main.cpp
)

# Target dependencies for drone controller
ament_target_dependencies(drone_controller
    rclcpp
    std_srvs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    grid_map_msgs
    grid_map_ros
)

# Link drone controller to common library
target_link_libraries(drone_controller rs1_robot_common)

# Target dependencies for sensor processor
ament_target_dependencies(sensor_processor_node
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    grid_map_msgs
    grid_map_ros
)

# Link sensor processor to common library
target_link_libraries(sensor_processor_node rs1_robot_common)

# Install executables and library
install(TARGETS
    rs1_robot_common
    drone_controller
    sensor_processor_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install package directories
install(
  DIRECTORY config launch models urdf worlds include/ scripts/
  DESTINATION share/${PROJECT_NAME}
)

# Install Python launch files
install(
  PROGRAMS
    launch/rs1_drone_spawner.py
    launch/rs1_single_drone.py
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(
    rclcpp
    std_srvs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

ament_package()
