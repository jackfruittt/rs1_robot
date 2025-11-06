#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    world_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share', 'rs1_robot', 'worlds', 'simple_trees_builtin.sdf'
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path, '--render-engine', 'optix'],
            output='screen'
        )
    ])
