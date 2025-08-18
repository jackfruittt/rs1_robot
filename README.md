## Overview

This package provides a complete simulation environment for the RS1 drone, a modified SJTU drone model optimised for modern Ignition Gazebo Fortress. The simulation includes realistic sensor models, flight dynamics, and a comprehensive sensor suite suitable for autonomous navigation, mapping, and surveillance applications.

**NEW: Multi-Drone Support** - The package now supports spawning and controlling multiple drones simultaneously with working sensor bridging and independent operation (to-do).

## Features

### Multi-Drone System ✅ WORKING
- **Dynamic Spawning**: Launch 1-10 drones using simple command scripts
- **Self-Contained Launch**: Each spawner handles world, bridge config, and drone spawning
- **Independent Operation**: Each drone operates with isolated sensor topics and control
- **Automatic Bridge Generation**: Dynamic YAML bridge configuration for all drones
- **Linear Positioning**: Drones spawn 2 meters apart in a line for collision avoidance
- **Full Sensor Suite Per Drone**: All sensors (IMU, cameras, sonar, LiDAR, GPS) work independently with their own unique topics which are generated dynamically

### Available Launch Options
- **Spawner**: `./launch_multi_drone_simple.sh <num_drones>` - Working Spawner with controller not implemented 

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
- Full ROS2 Humble compatibility
- Comprehensive topic bridging between Ignition and ROS2
- Standard ROS message types for seamless integration
- RVIZ configuration for visualisation

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

### Single Drone Control
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
├── config/                 # Configuration files
│   ├── drone.yaml         # Drone parameters  
│   ├── gazebo_bridge.yaml # Static bridge configuration
│   └── rs1_robot_config.rviz # RVIZ configuration
├── launch/                # Launch files
│   ├── rs1_robot_ignition.py     # Single drone simulation
│   ├── rs1_drone_spawner.py      # Multi-drone spawner
│   └── rs1_robot_rviz.py         # RVIZ launch
├── scripts/               # Utility scripts
│   ├── generate_dynamic_bridge.py # Dynamic bridge config generator
├── models/                # 3D model files
│   ├── quadrotor_4.dae    # Drone visual model
│   └── quadrotor_4.stl    # Drone collision model
├── urdf/                  # Robot description files
│   ├── rs1_drone.urdf.xacro         # Single drone URDF
│   └── rs1_drone_adaptive.urdf.xacro # Multi-drone adaptive URDF
└── worlds/                # Simulation worlds
    ├── simple_trees_builtin.sdf # Main world (used by multi-drone)
    ├── simple_trees.sdf
    └── simple_test.world
```

### Customisation

#### Multi-Drone Configuration
1. **Number of Drones**: Modify shell scripts or use launch parameters
2. **Positioning**: Edit `spawn_drones()` function in spawner files for different formations
3. **Bridge Topics**: The system auto-generates bridge configs, or modify `generate_dynamic_bridge.py`
4. **World Selection**: Currently uses `simple_trees_builtin.sdf` (can be changed in spawner files)

#### Adding New Sensors
1. Modify `urdf/rs1_drone_adaptive.urdf.xacro` to add sensor definitions
2. Update `scripts/generate_dynamic_bridge.py` to include new sensor topics
3. Rebuild the package

#### Tuning Flight Parameters
Adjust control gains in `config/drone.yaml`:
- `velocityGain`: Responsiveness to velocity commands
- `attitudeGain`: Attitude control stability  
- `angularRateGain`: Angular velocity control

## Troubleshooting

### Common Issues

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
- Reduce number of drones for better performance
- Use `simple_trees_builtin.sdf` world for fastest simulation
- Monitor system resources: `htop` or `nvidia-smi` (if using GPU)
- Close unnecessary applications during multi-drone simulation

## Acknowledgements

- Based on the original SJTU Drone simulation package
- Adapted for Ignition Gazebo Fortress compatibility
- Uses native Ignition Gazebo plugins for realistic simulation

## Contact

- **Maintainer**: jackfruittt
- **Repository**: [https://github.com/jackfruittt/rs1_robot](https://github.com/jackfruittt/rs1_robot)
- **Issues**: [https://github.com/jackfruittt/rs1_robot/issues](https://github.com/jackfruittt/rs1_robot/issues) 
