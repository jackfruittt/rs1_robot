cmake_minimum_required(VERSION 3.8)
project(rs1_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetWaypoints.srv"
  "srv/ReactToWildfire.srv"
  "srv/ReactToHiker.srv"
  "srv/ReactToDebris.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Find testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(include)

# Create separate libraries for mission planning components
add_library(pid_controller SHARED
  src/pid.cpp
)

# Add dependencies for pid_controller
ament_target_dependencies(pid_controller
  rclcpp
)

add_library(drone_control SHARED
  src/drone_control.cpp
)

# Add dependencies for drone_control
ament_target_dependencies(drone_control
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
)

# Link drone_control with pid_controller
target_link_libraries(drone_control pid_controller)

# Create mission system libraries
add_library(state_machine SHARED
  src/state_machine.cpp
)

add_library(waypoint_planner SHARED
  src/waypoint_planner.cpp
)

add_library(mission_executor SHARED
  src/mission_executor.cpp
)

# Add dependencies for mission system libraries
ament_target_dependencies(state_machine
  rclcpp
)

ament_target_dependencies(waypoint_planner
  rclcpp
  geometry_msgs
)

ament_target_dependencies(mission_executor
  rclcpp
  geometry_msgs
  nav_msgs
)

# Link mission_executor with drone_control
target_link_libraries(mission_executor drone_control)

# Add Theta* path planner library
add_library(theta_star_path_planner SHARED
  src/theta_star_path_planner.cpp
)

ament_target_dependencies(theta_star_path_planner
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Create mission planner component library
add_library(mission_planner_component SHARED
  src/mission_node.cpp
)

# Link mission planner with mission system libraries
target_link_libraries(mission_planner_component
  state_machine
  waypoint_planner
  mission_executor
  theta_star_path_planner
)

# Create enhanced drone controller component with mission planning
add_library(drone_controller_enhanced_component SHARED
  src/drone_node.cpp
)

# Link enhanced drone controller with control libraries
target_link_libraries(drone_controller_enhanced_component drone_control pid_controller)

# Dependencies for new mission planning components
ament_target_dependencies(mission_planner_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
)

# Add dependency on generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mission_planner_component "${cpp_typesupport_target}")

ament_target_dependencies(drone_controller_enhanced_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
)

# Register mission planning components
rclcpp_components_register_nodes(mission_planner_component "drone_swarm::MissionPlannerNode")
rclcpp_components_register_nodes(drone_controller_enhanced_component "drone_swarm::DroneControllerNode")

# Create composed executables
add_executable(drone_mission_composed src/main_composition.cpp)
target_link_libraries(drone_mission_composed
  mission_planner_component
  drone_controller_enhanced_component
)

# Install executables and libraries
install(TARGETS
    pid_controller
    drone_control
    state_machine
    waypoint_planner
    mission_executor
    theta_star_path_planner
    mission_planner_component
    drone_controller_enhanced_component
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    drone_mission_composed
    DESTINATION lib/${PROJECT_NAME}
)

# Install package directories
install(
  DIRECTORY config launch models urdf worlds include/ scripts/
  DESTINATION share/${PROJECT_NAME}
)

# If you specifically need this as an executable launch script:
install(
  PROGRAMS
    launch/rs1_drone_spawner.py
    launch/rs1_single_drone.py
    launch/rs1_swarm_composed.py  
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_dependencies(
    rclcpp
    rclcpp_components
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

#Export mission planning components
ament_export_targets(export_mission_components)
install(TARGETS mission_planner_component
        drone_controller_enhanced_component
        pid_controller
        drone_control
        state_machine
        waypoint_planner
        mission_executor
        theta_star_path_planner
        EXPORT export_mission_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
  PROGRAMS
  "scripts/generate_dynamic_bridge.py"
  "scripts/swarm_status_checker.py"
  DESTINATION lib/${PROJECT_NAME}
)

# Add tests
if(BUILD_TESTING)
  ament_add_gtest(test_theta_star_path_planner test/test_theta_star_path_planner.cpp)
  target_link_libraries(test_theta_star_path_planner theta_star_path_planner)
  ament_target_dependencies(test_theta_star_path_planner
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
  )
endif()

ament_package()
