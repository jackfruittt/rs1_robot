#!/usr/bin/env python3
"""
Fixed bridge generation script with proper argument handling
"""

import yaml
import sys
import argparse
from pathlib import Path

def generate_bridge_config(num_drones):
    """Generate bridge configuration for specified number of drones"""
    config = []
    
    # Clock topic (from working example)
    config.append({
        'ros_topic_name': '/clock',
        'gz_topic_name': '/clock',
        'ros_type_name': 'rosgraph_msgs/msg/Clock',
        'gz_type_name': 'gz.msgs.Clock',
        'direction': 'GZ_TO_ROS'
    })
    
    # Generate for each drone
    for i in range(1, num_drones + 1):
        drone_name = f"rs1_drone_{i}"
        
        # Drone control command velocity
        config.append({
            'ros_topic_name': f'/{drone_name}/cmd_vel',
            'gz_topic_name': f'/model/{drone_name}/cmd_vel',
            'ros_type_name': 'geometry_msgs/msg/Twist',
            'gz_type_name': 'ignition.msgs.Twist',
            'direction': 'ROS_TO_GZ'
        })
        
        # Odometry output
        config.append({
            'ros_topic_name': f'/{drone_name}/odom',
            'gz_topic_name': f'/model/{drone_name}/odometry_with_covariance',
            'ros_type_name': 'nav_msgs/msg/Odometry',
            'gz_type_name': 'gz.msgs.OdometryWithCovariance',
            'direction': 'GZ_TO_ROS'
        })
        
        # IMU sensor
        config.append({
            'ros_topic_name': f'/{drone_name}/imu',
            'gz_topic_name': f'/model/{drone_name}/imu',
            'ros_type_name': 'sensor_msgs/msg/Imu',
            'gz_type_name': 'ignition.msgs.IMU',
            'direction': 'GZ_TO_ROS'
        })
        
        # Front camera: RGB
        config.append({
            'ros_topic_name': f'/{drone_name}/front/image',
            'gz_topic_name': f'/model/{drone_name}/front_camera',
            'ros_type_name': 'sensor_msgs/msg/Image',
            'gz_type_name': 'ignition.msgs.Image',
            'direction': 'GZ_TO_ROS'
        })
        
        # Front camera info
        config.append({
            'ros_topic_name': f'/{drone_name}/front/camera_info',
            'gz_topic_name': f'/model/{drone_name}/front_camera/camera_info',
            'ros_type_name': 'sensor_msgs/msg/CameraInfo',
            'gz_type_name': 'ignition.msgs.CameraInfo',
            'direction': 'GZ_TO_ROS'
        })
        
        # Bottom camera: RGB
        config.append({
            'ros_topic_name': f'/{drone_name}/bottom/image',
            'gz_topic_name': f'/model/{drone_name}/bottom_camera',
            'ros_type_name': 'sensor_msgs/msg/Image',
            'gz_type_name': 'ignition.msgs.Image',
            'direction': 'GZ_TO_ROS'
        })
        
        # Bottom camera info
        config.append({
            'ros_topic_name': f'/{drone_name}/bottom/camera_info',
            'gz_topic_name': f'/model/{drone_name}/bottom_camera/camera_info',
            'ros_type_name': 'sensor_msgs/msg/CameraInfo',
            'gz_type_name': 'ignition.msgs.CameraInfo',
            'direction': 'GZ_TO_ROS'
        })
        
        # GPS/NavSat sensor
        config.append({
            'ros_topic_name': f'/{drone_name}/navsat',
            'gz_topic_name': f'/model/{drone_name}/gps',
            'ros_type_name': 'sensor_msgs/msg/NavSatFix',
            'gz_type_name': 'ignition.msgs.NavSat',
            'direction': 'GZ_TO_ROS'
        })
        
        # Downward sonar/range sensor
        config.append({
            'ros_topic_name': f'/{drone_name}/sonar',
            'gz_topic_name': f'/model/{drone_name}/sonar',
            'ros_type_name': 'sensor_msgs/msg/Range',
            'gz_type_name': 'ignition.msgs.LaserScan',
            'direction': 'GZ_TO_ROS'
        })
        
        # 2D Lidar sensor
        config.append({
            'ros_topic_name': f'/{drone_name}/lidar',
            'gz_topic_name': f'/model/{drone_name}/lidar',
            'ros_type_name': 'sensor_msgs/msg/LaserScan',
            'gz_type_name': 'ignition.msgs.LaserScan',
            'direction': 'GZ_TO_ROS'
        })
    
    return config

def main():
    # Create temp directory path in ROS workspace 
    ros_ws_temp = Path.home() / 'rs1_ws' / 'temp'
    default_output = ros_ws_temp / 'rs1_dynamic_bridge.yaml'
    
    # Handle both old style (positional) and new style (with argparse) calls
    if len(sys.argv) == 2:
        # Old style: python3 script.py <num_drones>
        num_drones = int(sys.argv[1])
        output_file = default_output
    elif len(sys.argv) >= 3:
        # New style with -o flag or positional output
        parser = argparse.ArgumentParser(description='Generate bridge config for multiple drones')
        parser.add_argument('num_drones', type=int, help='Number of drones')
        parser.add_argument('-o', '--output', default=default_output, 
                           help='Output file path')
        
        args = parser.parse_args()
        num_drones = args.num_drones
        output_file = args.output
    else:
        print("Usage: python3 generate_dynamic_bridge.py <num_drones> [-o output_file]")
        sys.exit(1)
    
    if num_drones < 1 or num_drones > 10:
        print("Error: Number of drones must be between 1 and 10")
        sys.exit(1)
    
    config = generate_bridge_config(num_drones)
    
    # Ensure output directory exists
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    # Write to file
    with open(output_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"Generated bridge config for {num_drones} drones at {output_file}")

if __name__ == '__main__':
    main()
