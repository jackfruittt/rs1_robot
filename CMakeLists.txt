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
find_package(ros_gz_interfaces REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# Include directories
include_directories(include)

# Add executable for multi-drone controller

# Install executables
install(TARGETS
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
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
