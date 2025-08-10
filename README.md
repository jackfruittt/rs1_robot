## Overview

This package provides a complete simulation environment for the RS1 drone, a modified SJTU drone model optimised for modern Ignition Gazebo Fortress. The simulation includes realistic sensor models, flight dynamics, and a comprehensive sensor suite suitable for autonomous navigation, mapping, and surveillance applications.

## Features

### Flight Dynamics
- Native Ignition Gazebo `MulticopterVelocityControl` plugin
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
- Ignition Gazebo Fortress

### Dependencies
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz2
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

### Basic Simulation Launch

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

### Control Topics
- `/rs1_drone/cmd_vel` (geometry_msgs/Twist) - Velocity commands

### Sensor Topics
- `/rs1_drone/imu` (sensor_msgs/Imu) - IMU data
- `/rs1_drone/odom` (nav_msgs/Odometry) - Odometry information
- `/rs1_drone/navsat` (sensor_msgs/NavSatFix) - GPS data
- `/rs1_drone/sonar` (sensor_msgs/Range) - Downward range sensor
- `/rs1_drone/lidar` (sensor_msgs/LaserScan) - 2D LiDAR data

### Camera Topics
- `/rs1_drone/front/image` (sensor_msgs/Image) - Front camera image
- `/rs1_drone/front/camera_info` (sensor_msgs/CameraInfo) - Front camera info
- `/rs1_drone/bottom/image` (sensor_msgs/Image) - Bottom camera image
- `/rs1_drone/bottom/camera_info` (sensor_msgs/CameraInfo) - Bottom camera info

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

## Flight Control Examples (Untested but should be functional)

### Basic Takeoff
```bash
# Takeoff command (move up 2 metres)
ros2 topic pub /rs1_drone/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 2.0}"
```

### Forward Flight
```bash
# Move forward at 1 m/s
ros2 topic pub /rs1_drone/cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}"
```

### Rotation
```bash
# Rotate about z-axis at 0.5 rad/s
ros2 topic pub /rs1_drone/cmd_vel geometry_msgs/msg/Twist "angular: {x: 0.0, y: 0.0, z: 0.5}"
```

## Sensor Data Monitoring

### View Sensor Data
```bash
# Monitor IMU data
ros2 topic echo /rs1_drone/imu

# Monitor altitude from sonar
ros2 topic echo /rs1_drone/sonar

# Monitor GPS coordinates
ros2 topic echo /rs1_drone/navsat

# Monitor LiDAR scan data
ros2 topic echo /rs1_drone/lidar
```

### Camera Streams
Use RVIZ or image view to visualise camera feeds (alrady loaded into rviz by default but can use if you launch with rviz disabled at first):
```bash
ros2 run image_view image_view image:=/rs1_drone/front/image
ros2 run image_view image_view image:=/rs1_drone/bottom/image
```

## Development

### Package Structure
```
rs1_robot/
├── config/                 # Configuration files
│   ├── drone.yaml         # Drone parameters
│   ├── gazebo_bridge.yaml # Topic bridge configuration
│   └── rs1_robot_config.rviz # RVIZ configuration
├── launch/                # Launch files
│   ├── rs1_robot_ignition.py  # Main simulation launch
│   ├── rs1_robot_world.py     # World-only launch
│   └── rs1_robot_rviz.py      # RVIZ launch
├── models/                # 3D model files
│   ├── quadrotor_4.dae    # Drone visual model
│   └── quadrotor_4.stl    # Drone collision model
├── urdf/                  # Robot description files
│   └── sjtu_drone.urdf.xacro  # Drone URDF definition
└── worlds/                # Simulation worlds
    ├── simple_trees_builtin.sdf
    ├── simple_trees.sdf
    └── simple_test.world
```

### Customisation

#### Adding New Sensors
1. Modify `urdf/sjtu_drone.urdf.xacro` to add sensor definitions
2. Update `config/gazebo_bridge.yaml` to bridge new topics
3. Rebuild the package

#### Tuning Flight Parameters
Adjust control gains in `config/drone.yaml`:
- `velocityGain`: Responsiveness to velocity commands
- `attitudeGain`: Attitude control stability
- `angularRateGain`: Angular velocity control

## Troubleshooting

### Common Issues

**Simulation won't start**:
- Ensure all dependencies are installed
- Check that Ignition Gazebo Fortress is properly installed
- Verify ROS2 Humble sourcing

**No sensor data**:
- Check bridge configuration in `gazebo_bridge.yaml`
- Ensure topic names match between URDF and bridge config
- Verify sensors are enabled in the URDF

**Poor flight performance**:
- Adjust control gains in `drone.yaml`
- Check for collision issues in the world
- Ensure adequate computational resources

### Debug Commands
```bash
# List available topics
ros2 topic list

# Check topic data rates
ros2 topic hz /rs1_drone/imu

# Monitor all drone topics
ros2 topic list | grep rs1_drone

# Check TF tree
ros2 run tf2_tools view_frames
```

## Acknowledgements

- Based on the original SJTU Drone simulation package
- Adapted for Ignition Gazebo Fortress compatibility
- Uses native Ignition Gazebo plugins for realistic simulation

## Contact

- **Maintainer**: jackfruittt
- **Repository**: [https://github.com/jackfruittt/rs1_robot](https://github.com/jackfruittt/rs1_robot)
- **Issues**: [https://github.com/jackfruittt/rs1_robot/issues](https://github.com/jackfruittt/rs1_robot/issues) 
