cmake_minimum_required(VERSION 3.8)
project(rs1_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Public headers in 'include/'
include_directories(include)

# --- Interface generation (srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetWaypoints.srv"          # exact case & path
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# --- Node target
add_executable(waypoint_nav
  src/waypoint_nav_node.cpp
  src/main.cpp
)
ament_target_dependencies(waypoint_nav
  rclcpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs
)

target_include_directories(waypoint_nav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# Ensure the node builds after the interfaces are generated
add_dependencies(waypoint_nav
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 🔑 This both links and wires up the correct C++ typesupport lib
rosidl_target_interfaces(waypoint_nav ${PROJECT_NAME} "rosidl_typesupport_cpp")



# --- Install
install(TARGETS waypoint_nav
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models urdf worlds include/ scripts/
  DESTINATION share/${PROJECT_NAME}
)

# If you specifically need this as an executable launch script:
install(
  PROGRAMS launch/rs1_drone_spawner.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# --- Lint / package
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
