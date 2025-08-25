#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import math

def launch_setup(context, *args, **kwargs):
    num_drones = int(context.launch_configurations['num_drones'])
    nodes = []
    
    # Calculate positions in a circle
    radius = 2.0  # meters
    for i in range(num_drones):
        angle = 2 * math.pi * i / num_drones
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        # Spawn each drone with unique namespace
        robot_spawner = Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-name', f'rs1_drone_{i}',
                '-x', str(x),
                '-y', str(y),
                '-z', '1.0'
            ],
            parameters=[{
                'use_sim_time': True,
                'drone_namespace': f'rs1_drone_{i}'
            }]
        )
        nodes.append(robot_spawner)
    
    return nodes

def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones to spawn'
    )
    
    return LaunchDescription([
        num_drones_arg,
        OpaqueFunction(function=launch_setup)
    ])