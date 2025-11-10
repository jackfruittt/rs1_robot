## Overview

This package provides a basic simulation environment and autonomous flight control system for the RS1 drone, a modified SJTU drone model optimised for modern Ignition Gazebo Fortress. The drone nodes are composed and optimised to scale better. 

**Mission Control Using Peer Detection** - Drones are decentralised, IDLE drones becomes responders if a surveiying drone detects a scenario by using peer discovery. If there are no IDLE drones the detector becomes the responder. 

**Dynamic Drone Spawing** - The package supports spawning and controlling multiple drones simultaneously, can spawn as many drones as system can handle.

### Available Launch Options
- **Launching using shell script**: `./comp_multi_drone_simple.sh <num_drones> <gazebo headed/headless>` - Working spawner. Shell script automates process

### Node Architecture & Composition

The RS1 drone system utilises **ROS 2 Composable Nodes**, which provide significant performance and efficiency benefits:

#### What are Composable Nodes?
Composable nodes are a ROS 2 feature that allows multiple nodes to run within the same process, enabling:
- **Intra-Process Communication**: Zero-copy message passing between nodes in the same process
- **Reduced Latency**: Direct memory sharing eliminates serialisation overhead
- **Lower Resource Usage**: Single process instead of multiple separate processes
- **Better Performance**: Faster communication for real-time control applications

#### Our Composable Architecture
The system consists of two main composable nodes:

1. **MissionPlannerNode** (`mission_node.h/.cpp`):
   - Manages mission state machine (IDLE → TAKEOFF → WAYPOINT_NAVIGATION → LANDING)
   - Provides mission services for waypoint setting and control
   - Publishes target poses and mission states
   - Handles high-level flight planning and coordination

2. **DroneControllerNode** (`drone_node.h/.cpp`):
   - Executes low-level flight control using advanced PID controllers
   - Processes odometry and target pose data
   - Publishes velocity commands to drone actuators
   - Manages flight modes and safety monitoring

#### Composition Benefits for Drone Control
- **Real-Time Performance**: Mission planner and controller communicate with minimal latency
- **Shared Memory**: Odometry and target pose data shared efficiently between nodes
- **Synchronised Updates**: Both nodes execute in same process for coordinated control
- **Resource Efficiency**: Single executable handles both mission planning and control 

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Ignition Gazebo Fortress version 6.17.0
- Ignition-sensor6 Plugins

### Dependencies
- **Use our installaltion dependency shell script:** - This pacakge comes with a shell script that installs all dependencies. It assumes ROS Humble is installed but not necessarily the full (could be minimal).

## Installation

1. Make a Software directory (If not already made) and clone it into Software
   ```bash
   mkdir Software
   cd ~/Software
   git clone https://github.com/jackfruittt/rs1_robot.git
   ```
2. **Make rs1_ws/src (If not already made) and link the pakcage to src**
   ```bash
   mkdir -p rs1_ws/src
   cd rs1_ws/src
   ln -s ~/Software/rs1_robot
   ```

3. **Run the installation shell script inside the rs1_robot root directory**
   ```bash
   cd ~/Software/rs1_robot
   chmod u+x ./install_rs1_robot_dependencies.sh
   ./install_rs1_robot_dependencies.sh
   ```
   
4. **Build the package**:
   ```bash
   cd ~/rs1_ws
   colcon build --packages-select rs1_robot
   source install/setup.bash
   ```

## Usage
The autonomous mission system provides complete waypoint navigation capabilities:

#### Quick Start - Composable Nodes (Recommended)
```bash
# Source workspace
source rs1_ws/install/setup.bash

# Launch single drone with in world with gazebo headed
./comp_drone_spawner.sh 1 gazebo:=true

# Have the drone takeoff and begin navigating
ros2 service call /rs1_drone_1/takeoff_drone std_srvs/srv/Trigger

# Land the drone
ros2 service call /rs1_drone_/land_drone std_srvs/srv/Trigger
```

#### Mission Services Available
- `/rs1_drone/takeoff_drone` - Begin autonomous mission (takeoff and start waypoint navigation)
- `/rs1_drone/land_drone` - Initiate controlled landing sequence

### Basic Simulation Launch for One drone

Launch the complete simulation environment:
```bash
ros2 launch rs1_robot rs1_robot_ignition.py
```

This will start:
- Ignition Gazebo with the drone model
- ROS2-Gazebo bridge for topic communication
- Robot state publisher for TF frames
- RVIZ for visualisation

## Available Topics
Note: the below topics only say `rs1_drone`, this is reflective of the namespace `rs1_drone_$` where `$` is the drone number. 
### Mission Control Topics
For autonomous mission operation:
- `/rs1_drone/mission_state` (std_msgs/String) - Current mission state
- `/rs1_drone/target_pose` (geometry_msgs/PoseStamped) - Target waypoint position
- `/rs1_drone/cmd_vel` (geometry_msgs/Twist) - Velocity commands from controller
- `/rs1_drone/current_pose` (geometry_msgs/PoseStamped) - Current drone position
- `/rs1_drone/mission_assignment` (std_msgs/String) - Assign waypoints for the drone to travel (this is what the GUI uses in the backend, works manually)

### Control Topics
- `/rs1_drone/cmd_vel` (geometry_msgs/Twist) - Single drone velocity commands
- `/rs1_drone_1/cmd_vel` (geometry_msgs/Twist) - Multi-drone: Drone 1 velocity commands
- `/rs1_drone_2/cmd_vel` (geometry_msgs/Twist) - Multi-drone: Drone 2 velocity commands
- And so on for each drone...

### Sensor Topics (Per Drone)
Each drone has its own complete sensor suite:
- `/rs1_drone_X/imu` (sensor_msgs/Imu) - IMU data
- `/rs1_drone_X/odom` (nav_msgs/Odometry) - Odometry information
- `/rs1_drone_X/navsat` (sensor_msgs/NavSatFix) - GPS data
- `/rs1_drone_X/sonar` (sensor_msgs/Range) - Downward range sensor
- `/rs1_drone_X/lidar` (sensor_msgs/LaserScan) - 2D LiDAR data

### Camera Topics (Per Drone)
- `/rs1_drone_X/front/image` (sensor_msgs/Image) - Front camera image
- `/rs1_drone_X/front/camera_info` (sensor_msgs/CameraInfo) - Front camera info
- `/rs1_drone_X/bottom/image` (sensor_msgs/Image) - Bottom camera image
- `/rs1_drone_X/bottom/camera_info` (sensor_msgs/CameraInfo) - Bottom camera info

*Replace X with drone number (1, 2, 3, etc.)*

### Multi-Drone Topic Verification
```bash
# List all drone topics
ros2 topic list | grep rs1_drone

# Check specific drone sensor data
ros2 topic echo /rs1_drone_1/imu
ros2 topic echo /rs1_drone_2/sonar
ros2 topic echo /rs1_drone_3/lidar
```

## Flight Control Examples

### Autonomous Mission Control (Recommended)
```bash

# Takeoff drones (enters navigation after taking off)
ros2 service call /rs1_drone_1/takeoff_drone std_srvs/srv/Trigger

# Replace rs1_drone_1 with drone number you want to take off
# e.g. if you want drone 3 to take off then its
# ros2 service call /rs1_drone_3/takeoff_drone std_srvs/srv/Trigger

# Land when mission complete
ros2 service call /rs1_drone_1/land_drone std_srvs/srv/Trigger
```

### Drone Control
```bash
ros2 topic pub /rs1_drone_1/mission_assignment std_msgs/msg/String "data: 'ASSIGN,ROUTE,-5.64,12.19,12.76'" -1

ros2 topic pub /rs1_drone_2/mission_assignment std_msgs/msg/String "data: 'ASSIGN,ROUTE,-25,14,20'" -1
```

## Troubleshooting

### Common Issues

**Mission system not responding**:
- Ensure main_composition is running: `ps aux | grep main_composition`
- Check service availability: `ros2 service list | grep rs1_drone`
- Verify mission state: `ros2 topic echo /rs1_drone/mission_state`
- Check for ROS 2 composition errors in terminal output

**Drone not following waypoints**:
- Verify target pose publication: `ros2 topic echo /rs1_drone/target_pose`
- Check cmd_vel output: `ros2 topic hz /rs1_drone/cmd_vel`
- Monitor PID controller logs for control issues
- Ensure drone is receiving odometry: `ros2 topic hz /rs1_drone/odom`

**Composable nodes not communicating**:
- Check intra-process communication is enabled in main_composition
- Verify both nodes are in same process: `ps aux | grep main_composition`
- Monitor topic latency: `ros2 topic hz` on communication topics
- Check for composition registration errors

**Multi-drone visual conflicts**:
- Cosmetic Gazebo warnings about existing visuals - doesn't affect functionality
- All drones spawn and operate correctly despite warnings
- Mesa graphics warnings are system-level and don't impact simulation

**Simulation won't start**:
- Ensure all dependencies are installed
- Check that Ignition Gazebo Fortress is properly installed
- Verify ROS2 Humble sourcing
- Try cleaning: `pkill -f "ign gazebo" && pkill -f "ros2"`

**No sensor data from specific drone**:
- Check if bridge config was generated: `ls -la /tmp/rs1_dynamic_bridge.yaml`
- Verify topics exist: `ros2 topic list | grep rs1_drone_X`
- Check bridge logs for errors

## Acknowledgements

- Based on the original SJTU Drone simulation package
- Adapted for Ignition Gazebo Fortress compatibility
- Uses native Ignition Gazebo plugins for realistic simulation

## Contact

- **Maintainer**: jackfruittt
- **Repository**: [https://github.com/jackfruittt/rs1_robot](https://github.com/jackfruittt/rs1_robot)
- **Issues**: [https://github.com/jackfruittt/rs1_robot/issues](https://github.com/jackfruittt/rs1_robot/issues) 
