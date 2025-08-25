#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction, 
    RegisterEventHandler,
    LogInfo,
    TimerAction,
    ExecuteProcess
)
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import math

def generate_launch_description():
    pkg_rs1_robot = get_package_share_directory('rs1_robot')
    world_file = os.path.join(pkg_rs1_robot, 'worlds', 'simple_trees_builtin.sdf')
    
    # Add max drones parameter
    max_drones = 6
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description=f'Number of drones to spawn (max {max_drones})'
    )

    def launch_setup(context):
        num_drones = min(int(context.launch_configurations['num_drones']), max_drones)
        nodes = []
        
        # Start Gazebo
        gazebo = ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_file],
            output='screen'
        )
        
        # Wait for Gazebo to be ready
        gazebo_ready = RegisterEventHandler(
            event_handler=OnProcessIO(
                target_action=gazebo,
                on_stdout=lambda output: [
                    LogInfo(msg='Gazebo is ready, starting drone spawning...'),
                    TimerAction(period=2.0, actions=[])
                ] if 'Gazebo web server running' in output or 'Serving on' in output else [],
                on_stderr=lambda output: []
            )
        )
        
        nodes.extend([gazebo, gazebo_ready])
        
        # Spawn drones with sequential timing
        for i in range(num_drones):
            namespace = f'rs1_drone_{i}'
            
            # Create robot description
            robot_description = ParameterValue(
                Command(['xacro ', 
                        os.path.join(pkg_rs1_robot, 'urdf', 'rs1_drone.urdf.xacro'),
                        ' drone_namespace:=', namespace]),
                value_type=str
            )
            
            # Robot state publisher
            robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': True
                }]
            )
            
            # Spawn drone
            robot_spawner = Node(
                package='ros_gz_sim',
                executable='create',
                namespace=namespace,
                arguments=[
                    '-topic', f'/{namespace}/robot_description',
                    '-name', namespace,
                    '-x', str(2.0 * math.cos(2 * math.pi * i / num_drones)),
                    '-y', str(2.0 * math.sin(2 * math.pi * i / num_drones)),
                    '-z', '1.0'
                ]
            )
            
            # Bridge
            bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=namespace,
                name='topic_bridge',
                parameters=[{
                    'config_file': os.path.join(pkg_rs1_robot, 'config', 'gazebo_bridge.yaml'),
                    'use_sim_time': True,
                    'name': str(i)
                }]
            )
            
            # Add delay between spawns
            spawn_timer = TimerAction(
                period=2.0 * i,
                actions=[
                    LogInfo(msg=f'Spawning drone {namespace}...'),
                    robot_state_publisher,
                    robot_spawner,
                    bridge
                ]
            )
            
            # Verify spawn
            spawn_event = RegisterEventHandler(
                OnProcessExit(
                    target_action=robot_spawner,
                    on_exit=[LogInfo(msg=f'Drone {namespace} spawned successfully')]
                )
            )
            
            nodes.extend([spawn_timer, spawn_event])
        
        return nodes

    return LaunchDescription([
        num_drones_arg,
        OpaqueFunction(function=launch_setup)
    ])