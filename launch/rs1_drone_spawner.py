#!/usr/bin/env python3
"""
Multi-drone launch file based on the working rs1_robot_ignition.py
Adapted to spawn multiple drones using OpaqueFunction
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import subprocess


def spawn_multiple_drones(context, *args, **kwargs):
    """Spawn multiple drones based on the working single drone pattern"""
    
    # Get launch configurations
    num_drones = int(context.launch_configurations['num_drones'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    print(f"Spawning {num_drones} drones using working pattern")
    
    # Get paths (same as working launch)
    pkg_path = FindPackageShare('rs1_robot')
    config_path = PathJoinSubstitution([pkg_path, 'config'])
    
    # Generate bridge config for multiple drones
    try:
        script_path = '/home/ace-rv/rs1_ros2_ws/src/rs1_robot/scripts/generate_dynamic_bridge.py'
        result = subprocess.run(['python3', script_path, str(num_drones), '-o', '/tmp/rs1_dynamic_bridge.yaml'], 
                              capture_output=True, text=True, check=True)
        print(f"Bridge config generated for {num_drones} drones")
    except Exception as e:
        print(f"Error generating bridge config: {e}")
        return []
    
    nodes = []
    
    # Create bridge node (adapted from working version)
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': '/tmp/rs1_dynamic_bridge.yaml',  # Use generated config
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    nodes.append(gazebo_bridge)
    
    # Create nodes for each drone (following working pattern exactly)
    for i in range(1, num_drones + 1):
        drone_name = f'rs1_drone_{i}'
        
        # Position drones in a line (same spacing as before)
        x_pos = (i - 1) * 3.0  # Start at 0, then 3, 6, 9...
        y_pos = 0.0
        z_pos = 1.0
        
        print(f"Creating drone {i}: {drone_name} at ({x_pos}, {y_pos}, {z_pos})")
        
        # Robot description (adapted from working version)
        robot_description_content = ParameterValue(
            Command(['xacro ',
                     PathJoinSubstitution([pkg_path,
                                           'urdf',
                                           'rs1_drone_adaptive.urdf.xacro']),
                     ' drone_name:=', drone_name,
                     ' drone_namespace:=', drone_name]),
            value_type=str)
        
        # Robot state publisher (same as working version but with namespace)
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{i}',
            namespace=drone_name,  # Add namespace for multi-drone
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        )
        
        # Spawn drone in Gazebo (same pattern as working version)
        robot_spawner = Node(
            package='ros_ign_gazebo',
            executable='create',
            name=f'spawn_drone_{i}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-topic', f'/{drone_name}/robot_description',  # Use namespaced topic
                '-name', drone_name,  # Unique name for each drone
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', str(z_pos)
            ]
        )
        
        # Add both nodes for this drone
        nodes.append(robot_state_publisher_node)
        nodes.append(robot_spawner)
    
    print(f"Created {len(nodes)} nodes for {num_drones} drones")
    return nodes


def generate_launch_description():
    """Generate launch description based on working rs1_robot_ignition.py"""

    ld = LaunchDescription()

    # Get paths (same as working launch)
    pkg_path = FindPackageShare('rs1_robot')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Launch arguments (based on working version + num_drones)
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    num_drones_launch_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones to spawn (1-10)'
    )
    ld.add_action(num_drones_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',  # Default to False for multi-drone
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='mountain_forest.sdf',
        description='Which world to load'
    )
    ld.add_action(world_launch_arg)

    # Start Gazebo (exactly same as working version)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([FindPackageShare('rs1_environment'),
                                               'worlds',
                                               [LaunchConfiguration('world')]]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn multiple drones using OpaqueFunction (after short delay)
    multiple_drones = OpaqueFunction(function=spawn_multiple_drones)
    drone_spawner = TimerAction(
        period=3.0,  # Short delay for Gazebo to start
        actions=[multiple_drones]
    )
    ld.add_action(drone_spawner)

    # RViz2 for visualization (same as working version)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, 'rs1_robot_config.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    return ld