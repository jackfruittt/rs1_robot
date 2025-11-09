## Overview

This package provides a complete simulation environment and autonomous flight control system for the RS1 drone, a modified SJTU drone model optimised for modern Ignition Gazebo Fortress. The simulation includes realistic sensor models, flight dynamics, and a comprehensive sensor suite suitable for autonomous navigation, mapping, and surveillance applications.

**NEW: Advanced Mission Control System** - The package now features a complete autonomous mission planning and control system with composable ROS 2 nodes for scalable drone operations.

**NEW: Multi-Drone Support** - The package supports spawning and controlling multiple drones simultaneously with working sensor bridging and independent operation.

**NEW: Multi-Drone Support** - The package now supports spawning and controlling multiple drones simultaneously with working sensor bridging and independent operation (to-do).

## Features

### Autonomous Mission Control System ✅ WORKING
- **Composable Node Architecture**: Mission planner and drone controller as composable ROS 2 nodes
- **Advanced PID Control**: Enhanced position control with integral windup protection
- **Mission State Management**: Comprehensive state machine for takeoff, waypoint navigation, hovering, and landing
- **Waypoint Navigation**: Intelligent path planning with distance-based speed adaptation
- **Service-Based Missions**: ROS 2 services for mission commands and waypoint setting
- **Real-Time Control**: 10Hz control loop with command smoothing and safety limits
- **Professional Documentation**: Full Doxygen documentation with Australian English standards

### Multi-Drone System ✅ WORKING
- **Dynamic Spawning**: Launch 1-10 drones using simple command scripts
- **Self-Contained Launch**: Each spawner handles world, bridge config, and drone spawning
- **Independent Operation**: Each drone operates with isolated sensor topics and control
- **Automatic Bridge Generation**: Dynamic YAML bridge configuration for all drones
- **Linear Positioning**: Drones spawn 2 meters apart in a line for collision avoidance
- **Full Sensor Suite Per Drone**: All sensors (IMU, cameras, sonar, LiDAR, GPS) work independently with their own unique topics which are generated dynamically

### Available Launch Options
- **Spawner**: `./launch_multi_drone_simple.sh <num_drones>` - Working spawner with full mission control
- **Composition**: `ros2 run rs1_robot main_composition` - Composable nodes for single drone
- **Mission Services**: Autonomous waypoint navigation with service-based control

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

### Flight Dynamics
- Native Ignition Gazebo `VelocityControl` plugin
- Realistic quadrotor physics with tuned control parameters
- Odometry publishing for pose estimation
- Collision detection and contact sensing

### Sensor Suite
- **IMU**: High-fidelity inertial measurement unit with realistic noise models
- **GPS/NavSat**: Global positioning system for outdoor navigation
- **Cameras**: 
  - Front-facing camera (640x360, 60Hz) for forward vision
  - Bottom-facing camera (640x360, 15Hz) for downward imaging
- **Range Sensor**: Sonar for altitude measurement (0.02-25m range)
- **2D LiDAR**:  For obstacle detection and mapping (0.1-30m range)

### Simulation Environments
- **Built-in World**: Simple environment with ground plane and obstacles

### ROS2 Integration
- Full ROS2 Humble compatibility with composable node architecture
- Comprehensive topic bridging between Ignition and ROS2
- Mission services for autonomous operation (`start_mission`, `add_waypoint`, `land_drone`)
- Advanced PID control system with windup protection
- Professional documentation standards with Australian English
- RVIZ configuration for visualisation and mission monitoring

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Ignition Gazebo Fortress version 6.17.0
- Ignition-sensor6 Plugins

### Dependencies
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz2
sudo apt install libignition-sensors6
```

## Installation

1. **Clone the repository**:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/jackfruittt/rs1_robot.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rs1_robot
   source install/setup.bash
   ```

## Usage

### Autonomous Mission Control ✅ WORKING

The autonomous mission system provides complete waypoint navigation capabilities:

#### Quick Start - Composable Nodes (Recommended)
```bash
# Source workspace
source rs1_ws/install/setup.bash

# Launch single drone with mission system
ros2 run rs1_robot main_composition

# In another terminal, start a mission
ros2 service call /rs1_drone/start_mission std_srvs/srv/Trigger

# Add waypoints for navigation
ros2 service call /rs1_drone/add_waypoint geometry_msgs/srv/SetPose "{pose: {position: {x: 5.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}"

# Land the drone
ros2 service call /rs1_drone/land_drone std_srvs/srv/Trigger
```

#### Mission Services Available
- `/rs1_drone/start_mission` - Begin autonomous mission (takeoff and start waypoint navigation)
- `/rs1_drone/add_waypoint` - Add waypoint to mission queue
- `/rs1_drone/land_drone` - Initiate controlled landing sequence
- `/rs1_drone/mission_state` - Monitor current mission state

#### Mission States
The mission system follows this state sequence:
1. **IDLE** - Drone on ground, ready for mission
2. **TAKEOFF** - Autonomous takeoff to 3m altitude 
3. **WAYPOINT_NAVIGATION** - Navigate through waypoint sequence
4. **HOVERING** - Stable hover between waypoints
5. **LANDING** - Controlled descent and landing

**What Works**:
- ✅ Complete autonomous takeoff sequence
- ✅ Waypoint navigation with distance-based speed control
- ✅ Advanced PID control with windup protection
- ✅ Mission state management and service interface
- ✅ Controlled landing with position hold
- ✅ Real-time mission monitoring via ROS topics

### Multi-Drone System ✅ WORKING

The multi-drone spawning system is now fully functional and can used as such:

#### Quick Start - Fixed Spawner (Recommended)
```bash
# Source ws 
source rs1_ws/install/setup.bash

# Launch 3 drones (note: Shell script is located in the root folder of rs1_robot, I recommend moving this to the home directory to avoid have to path the the shell script)
./rs1_ws/src/rs1_robot/launch_multi_drone_simple.sh

#If shell script is moved to home directory
./launch_multi_drone_simple.sh
```

#### Direct ROS2 Launch
```bash
# Use the launch file directly
ros2 launch rs1_robot rs1_drone_spawner.py num_drones:=3
```

**What Works**:
- ✅ All drones spawn correctly in Gazebo
- ✅ All sensors operational (IMU, cameras, sonar, LiDAR, GPS)
- ✅ Independent topic namespaces: `/rs1_drone_1/`, `/rs1_drone_2/`, etc.
- ✅ Dynamic bridge configuration generation
- ✅ Self-contained launch (world + bridge + drones)
- ✅ Linear positioning (2m spacing) for collision avoidance

**Multi-Drone Topics**: Each drone operates independently:
- `/rs1_drone_1/cmd_vel`, `/rs1_drone_1/odom`, `/rs1_drone_1/imu`
- `/rs1_drone_1/front/image`, `/rs1_drone_1/sonar`, `/rs1_drone_1/lidar`
- `/rs1_drone_2/cmd_vel`, `/rs1_drone_2/odom`, `/rs1_drone_2/imu`
- And so on for each drone...

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

### Mission Control Topics
For autonomous mission operation:
- `/rs1_drone/mission_state` (std_msgs/String) - Current mission state
- `/rs1_drone/target_pose` (geometry_msgs/PoseStamped) - Target waypoint position
- `/rs1_drone/cmd_vel` (geometry_msgs/Twist) - Velocity commands from controller
- `/rs1_drone/current_pose` (geometry_msgs/PoseStamped) - Current drone position

### Single Drone Topics
For single drone operation (or when referencing a specific drone in multi-drone setup):

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

## Configuration

### Drone Parameters
Edit `config/drone.yaml` to modify:
- Namespace settings
- Control gains (velocity, attitude, angular rate)
- Maximum velocities and accelerations

### Bridge Configuration
Modify `config/gazebo_bridge.yaml` to:
- Add or remove topic bridges
- Change message types
- Adjust topic namespaces

### World Selection
The launch file loads `worlds/simple_trees_builtin.sdf` by default.

## Flight Control Examples

### Autonomous Mission Control (Recommended)
```bash
# Start mission system
ros2 run rs1_robot main_composition

# Begin mission (autonomous takeoff)
ros2 service call /rs1_drone/start_mission std_srvs/srv/Trigger

# Add multiple waypoints
ros2 service call /rs1_drone/add_waypoint geometry_msgs/srv/SetPose "{pose: {position: {x: 10.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}"
ros2 service call /rs1_drone/add_waypoint geometry_msgs/srv/SetPose "{pose: {position: {x: 10.0, y: 10.0, z: 3.0}, orientation: {w: 1.0}}}"
ros2 service call /rs1_drone/add_waypoint geometry_msgs/srv/SetPose "{pose: {position: {x: 0.0, y: 10.0, z: 3.0}, orientation: {w: 1.0}}}"

# Monitor mission progress
ros2 topic echo /rs1_drone/mission_state

# Land when mission complete
ros2 service call /rs1_drone/land_drone std_srvs/srv/Trigger
```

### Manual Single Drone Control
```bash
# Takeoff command (move up 2 metres)
ros2 topic pub /rs1_drone/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 2.0}"

# Move forward at 1 m/s
ros2 topic pub /rs1_drone/cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}"

# Rotate about z-axis at 0.5 rad/s
ros2 topic pub /rs1_drone/cmd_vel geometry_msgs/msg/Twist "angular: {x: 0.0, y: 0.0, z: 0.5}"
```

### Multi-Drone Control (ToDO, In theory would work as such)
```bash
# Control drone 1
ros2 topic pub /rs1_drone_1/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 2.0}"

# Control drone 2 - move forward
ros2 topic pub /rs1_drone_2/cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}"

# Control drone 3 - rotate
ros2 topic pub /rs1_drone_3/cmd_vel geometry_msgs/msg/Twist "angular: {x: 0.0, y: 0.0, z: 0.5}"
```

## Sensor Data Monitoring

### View Sensor Data
```bash
# Monitor IMU data
ros2 topic echo /rs1_drone_X/imu

# Monitor altitude from sonar
ros2 topic echo /rs1_drone_X/sonar

# Monitor GPS coordinates
ros2 topic echo /rs1_drone_X/navsat

# Monitor LiDAR scan data
ros2 topic echo /rs1_drone_X/lidar
```

### Camera Streams
Use RVIZ or image view to visualise camera feeds (alrady loaded into rviz by default but can use if you launch with rviz disabled at first):
```bash
ros2 run image_view image_view image:=/rs1_drone_X/front/image
ros2 run image_view image_view image:=/rs1_drone_X/bottom/image
```

## Development

### Package Structure
```
rs1_robot/
├── config/                    # Configuration files
│   ├── drone.yaml            # Drone parameters  
│   ├── gazebo_bridge.yaml    # Static bridge configuration
│   └── rs1_robot_config.rviz # RVIZ configuration
├── include/                   # Header files (C++ interfaces)
│   ├── drone_control.h       # Advanced flight control system
│   ├── drone_node.h          # Drone controller composable node
│   ├── mission_node.h        # Mission planner composable node
│   ├── pid.h                 # Enhanced PID controller with windup protection
│   ├── mission/              # Mission management components
│   │   ├── mission_state.h   # Mission state enumeration
│   │   ├── state_machine.h   # Mission state machine
│   │   ├── path_planner.h    # Waypoint path planning
│   │   └── mission_executor.h # Mission execution framework
│   └── drone/
│        ├── safety_monitior.h # Monitors drone states and implements safety measures
│        └── sensor_manager.h # Responsible for processing and managing all sensor data
│
├── src/                      # Source files (C++ implementations)
│   ├── main_composition.cpp  # Composable nodes main executable
│   ├── drone_control.cpp     # Flight control implementation
│   ├── drone_node.cpp        # Drone controller node implementation
│   ├── mission_node.cpp      # Mission planner node implementation
│   ├── pid.cpp               # PID controller implementation
│   ├── state_machine.cpp     # State machine implementation
│   ├── path_planner.cpp      # Path planning implementation
│   └── mission_executor.cpp  # Mission execution implementation
├── launch/                   # Launch files
│   ├── rs1_robot_ignition.py     # Single drone simulation
│   ├── rs1_drone_spawner.py      # Multi-drone spawner
│   └── rs1_robot_rviz.py         # RVIZ launch
├── scripts/                  # Utility scripts
│   └── generate_dynamic_bridge.py # Dynamic bridge config generator
├── models/                   # 3D model files
│   ├── quadrotor_4.dae      # Drone visual model
│   └── quadrotor_4.stl      # Drone collision model
├── urdf/                     # Robot description files
│   ├── rs1_drone.urdf.xacro         # Single drone URDF
│   └── rs1_drone_adaptive.urdf.xacro # Multi-drone adaptive URDF
└── worlds/                   # Simulation worlds
    ├── simple_trees_builtin.sdf # Main world (used by multi-drone)
    ├── simple_trees.sdf
    └── simple_test.world
```

### Code Architecture

#### Composable Node System
The RS1 system uses a modern composable architecture for optimal performance:

**Main Composition (`main_composition.cpp`)**:
- Creates shared executor for both nodes
- Enables intra-process communication
- Manages node lifecycle and coordination

**Mission Planner Node (`mission_node.h/.cpp`)**:
- State machine management (IDLE → TAKEOFF → NAVIGATION → LANDING)
- Service interfaces for mission control
- Waypoint planning and coordination
- High-level mission logic

**Drone Controller Node (`drone_node.h/.cpp`)**:
- Low-level flight control execution
- PID-based position and velocity control
- Safety monitoring and flight mode management
- Direct actuator command generation

#### Control System Components

**Advanced PID Controller (`pid.h/.cpp`)**:
- Enhanced PID implementation with integral windup protection
- Configurable gains and output limits
- Optimised for drone flight control applications

**Flight Control System (`drone_control.h/.cpp`)**:
- Multiple PID controllers for X, Y, Z position control
- Distance-based speed adaptation from flyToGoal method
- Command smoothing and safety limiting
- Terrain following capabilities (future implementation)

**Mission Management (`mission/` directory)**:
- `mission_state.h`: Mission state enumeration and definitions
- `state_machine.h/.cpp`: State transition management
- `path_planner.h/.cpp`: Waypoint sequence planning
- `mission_executor.h/.cpp`: Mission execution framework

### Customisation

#### Mission System Configuration
1. **Control Parameters**: Modify PID gains in `drone_control.cpp` constructor
2. **Mission Timing**: Adjust state timeouts in `mission_node.cpp`
3. **Waypoint Tolerance**: Configure approach distances for waypoint completion
4. **Flight Speeds**: Modify velocity limits in `calculateAdvancedPositionControl()`

#### Composable Node Deployment
```bash
# Run as composed nodes (recommended for performance)
ros2 run rs1_robot main_composition

# Run as separate nodes (for debugging)
ros2 run rs1_robot mission_planner_node
ros2 run rs1_robot drone_controller_node  # In separate terminals
```

#### Multi-Drone Configuration
1. **Number of Drones**: Modify shell scripts or use launch parameters
2. **Positioning**: Edit `spawn_drones()` function in spawner files for different formations
3. **Bridge Topics**: The system auto-generates bridge configs, or modify `generate_dynamic_bridge.py`
4. **World Selection**: Currently uses `simple_trees_builtin.sdf` (can be changed in spawner files)

#### Adding New Mission Types
1. Add new states to `MissionState` enum in `mission_state.h`
2. Implement state transitions in `state_machine.cpp`
3. Add mission logic in `mission_node.cpp` mission update callback
4. Create new service interfaces for mission-specific commands

#### Extending Control Capabilities
1. **New PID Controllers**: Add controllers in `drone_control.cpp` constructor
2. **Advanced Control Modes**: Implement in `calculateAdvancedPositionControl()`
3. **Sensor Integration**: Add sensor callbacks in `drone_node.cpp`
4. **Safety Features**: Extend safety monitoring in drone controller

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
- Try cleaning: `pkill -f "ign gazebo" && pkill -f "parameter_bridge"`

**No sensor data from specific drone**:
- Check if bridge config was generated: `ls -la /tmp/rs1_dynamic_bridge.yaml`
- Verify topics exist: `ros2 topic list | grep rs1_drone_X`
- Check bridge logs for errors

**Bridge generation fails**:
- Ensure `scripts/generate_dynamic_bridge.py` is executable
- Check Python dependencies: `python3 -c "import yaml"`
- Verify script path in spawner files

### Debug Commands
```bash
# Check mission system status
ros2 service list | grep rs1_drone
ros2 topic list | grep rs1_drone

# Monitor mission execution
ros2 topic echo /rs1_drone/mission_state
ros2 topic echo /rs1_drone/target_pose
ros2 topic hz /rs1_drone/cmd_vel

# Test mission services
ros2 service call /rs1_drone/start_mission std_srvs/srv/Trigger
ros2 service call /rs1_drone/add_waypoint geometry_msgs/srv/SetPose "{pose: {position: {x: 5.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}"

# Check composable node status
ps aux | grep main_composition
ros2 node list | grep -E "(mission_planner|drone_controller)"

# Monitor control performance
ros2 topic echo /rs1_drone/cmd_vel --no-arr
# List all drone topics
ros2 topic list | grep rs1_drone

# Check specific drone topic data rates
ros2 topic hz /rs1_drone_1/imu
ros2 topic hz /rs1_drone_2/sonar

# Monitor bridge config generation
cat /tmp/rs1_dynamic_bridge.yaml

# Check drone count in bridge config
grep -c "rs1_drone_" /tmp/rs1_dynamic_bridge.yaml

# Verify all drones have topics
for i in {1..3}; do echo "Drone $i:"; ros2 topic list | grep rs1_drone_$i | wc -l; done
```

### Performance Tips
- Use composable nodes for best performance (`main_composition`)
- Monitor system resources during autonomous missions
- Adjust PID gains for smoother control if needed
- Use mission services rather than manual control for efficiency
- Reduce number of drones for better performance in multi-drone setups
- Use `simple_trees_builtin.sdf` world for fastest simulation
- Close unnecessary applications during complex missions

## Acknowledgements

- Based on the original SJTU Drone simulation package
- Adapted for Ignition Gazebo Fortress compatibility
- Uses native Ignition Gazebo plugins for realistic simulation

## Contact

- **Maintainer**: jackfruittt
- **Repository**: [https://github.com/jackfruittt/rs1_robot](https://github.com/jackfruittt/rs1_robot)
- **Issues**: [https://github.com/jackfruittt/rs1_robot/issues](https://github.com/jackfruittt/rs1_robot/issues) 
