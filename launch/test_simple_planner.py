#!/usr/bin/env python3
"""
Launch simple path planner with test data
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for simple path planner test"""
    
    # Simple path planner node
    planner_node = Node(
        package='rs1_robot',
        executable='simple_path_planner_node',
        name='simple_path_planner',
        output='screen',
        remappings=[
            ('lidar', '/rs1_drone_1/lidar'),
            ('goal_pose', '/goal_pose'),
            ('planned_path', '/planned_path'),
            ('occupancy_grid', '/occupancy_grid'),
        ]
    )
    
    return LaunchDescription([
        planner_node,
    ])