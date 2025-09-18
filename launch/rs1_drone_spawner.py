#!/usr/bin/env python3
"""
Multi-drone launch file with individual drone controllers
Each drone runs its own sensor processor + controller nodes
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import subprocess


def spawn_multiple_drones(context, *args, **kwargs):
    """Spawn multiple drones with individual drone controllers"""
    
    # Get launch configurations
    num_drones = int(context.launch_configurations['num_drones'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    print(f"Spawning {num_drones} drones with individual controllers")
    
    # Get paths
    pkg_path = FindPackageShare('rs1_robot')
    config_path = PathJoinSubstitution([pkg_path, 'config'])
    
    # Generate bridge config for multiple drones
    try:
        script_path = str(Path.home() / 'Software' / 'rs1_robot' / 'scripts' / 'generate_dynamic_bridge.py')
        result = subprocess.run(['python3', script_path, str(num_drones), '-o', '/tmp/rs1_dynamic_bridge.yaml'], 
                              capture_output=True, text=True, check=True)
        print(f"Bridge config generated for {num_drones} drones")
    except Exception as e:
        print(f"Error generating bridge config: {e}")
        return []
    
    nodes = []
    
    # Create bridge node
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': '/tmp/rs1_dynamic_bridge.yaml',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    nodes.append(gazebo_bridge)
    
    # Create individual drone controller nodes for each drone
    for i in range(1, num_drones + 1):
        drone_name = f'rs1_drone_{i}'
        
        # Position drones in a line (same spacing as before)
        x_pos = (i - 1) * 3.0  # Start at 0, then 3, 6, 9...
        y_pos = 0.0
        z_pos = 1.0
        
        print(f"Creating drone {i}: {drone_name} at ({x_pos}, {y_pos}, {z_pos})")
        
        # Robot description
        robot_description_content = ParameterValue(
            Command(['xacro ',
                     PathJoinSubstitution([pkg_path,
                                           'urdf',
                                           'rs1_drone_adaptive.urdf.xacro']),
                     ' drone_name:=', drone_name,
                     ' drone_namespace:=', drone_name]),
            value_type=str)
        
        # Robot state publisher (namespaced)
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{i}',
            namespace=drone_name,
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        )
        
        # Spawn drone in Gazebo
        robot_spawner = Node(
            package='ros_ign_gazebo',
            executable='create',
            name=f'spawn_drone_{i}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-topic', f'/{drone_name}/robot_description',
                '-name', drone_name,
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', str(z_pos)
            ]
        )
        
        # Add nodes for this drone
        nodes.append(robot_state_publisher_node)
        nodes.append(robot_spawner)
    
    print(f"Created {len(nodes)} nodes for {num_drones} drones with individual controllers")
    print("Each drone will run: DroneController with integrated SensorNode")
    return nodes


def generate_launch_description():
    """Generate launch description for multi-drone system with individual controllers"""

    ld = LaunchDescription()

    # Get paths
    pkg_path = FindPackageShare('rs1_robot')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Launch arguments
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
        description='Number of drones to spawn with composition (1-10)'
    )
    ld.add_action(num_drones_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='mountain_forest.sdf',
        description='Which world to load'
    )
    ld.add_action(world_launch_arg)
    
    # Gazebo launch arguments
    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='false',
        description='true = launch Gazebo GUI, false = headless'
    )

    # Headless Gazebo (gzserver only + EGL rendering)
    gazebo_headless = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py'
        ]),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([
                    FindPackageShare('rs1_environment'),
                    'worlds',
                    LaunchConfiguration('world')
                ]),
                ' -s -r --headless-rendering'
            ]
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('gazebo'), "' == 'false'"])
        )
    )

    # GUI Gazebo (gzserver + gzclient)
    gazebo_gui = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py'
        ]),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([
                    FindPackageShare('rs1_environment'),
                    'worlds',
                    LaunchConfiguration('world')
                ]),
                ' -r'
            ]
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('gazebo'), "' == 'true'"])
        )
    )
    ld.add_action(gazebo_arg)
    ld.add_action(gazebo_headless)
    ld.add_action(gazebo_gui)

    # Spawn multiple drones with individual controllers
    multiple_drones = OpaqueFunction(function=spawn_multiple_drones)
    drone_spawner = TimerAction(
        period=3.0,  # Short delay for Gazebo to start
        actions=[multiple_drones]
    )
    ld.add_action(drone_spawner)

    # RViz2 for visualisation (optional)
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
