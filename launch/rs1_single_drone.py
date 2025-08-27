#!/usr/bin/env python3
"""
Single drone launch file with namespace support
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for single drone with namespace"""
    
    ld = LaunchDescription()
    
    # Launch arguments
    drone_namespace_arg = DeclareLaunchArgument(
        'drone_namespace',
        default_value='rs1_drone_1',
        description='Namespace for the drone topics'
    )
    ld.add_action(drone_namespace_arg)
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    ld.add_action(use_sim_time_arg)
    
    # Single drone controller with namespace support
    drone_controller_node = Node(
        package='rs1_robot',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'drone_namespace': LaunchConfiguration('drone_namespace')
        }]
    )
    ld.add_action(drone_controller_node)
    
    return ld
