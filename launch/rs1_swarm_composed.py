#!/usr/bin/env python3
"""
Multi-drone launch file with composable mission planner + drone controller
Each drone runs composed nodes for optimal resource usage
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import subprocess
import yaml
import os

def load_waypoints_for_drone(waypoints_file_path, drone_name):
    """Load waypoints from YAML file for a specific drone"""
    try:
        # Resolve the file path
        if hasattr(waypoints_file_path, 'perform'):
            pkg_share = FindPackageShare('rs1_robot').find('rs1_robot')
            full_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
        else:
            full_path = str(waypoints_file_path)
        
        print(f"Loading waypoints from: {full_path}")
        
        with open(full_path, 'r') as file:
            waypoints_data = yaml.safe_load(file)
        
        # Extract waypoints for this specific drone
        drone_config = waypoints_data.get('waypoint_configs', {}).get(drone_name, {})
        
        if not drone_config:
            print(f"No waypoints found for {drone_name}, using fallback")
            return {}
        
        # Flatten the structure for ROS 2 parameters
        flattened_params = {}
        
        # Add mission name
        if 'mission_name' in drone_config:
            flattened_params['mission_name'] = drone_config['mission_name']
        
        # Add waypoints
        waypoints = drone_config.get('waypoints', [])
        for i, waypoint in enumerate(waypoints):
            position = waypoint.get('position', {})
            flattened_params[f'waypoints.{i}.position.x'] = position.get('x', 0.0)
            flattened_params[f'waypoints.{i}.position.y'] = position.get('y', 0.0)
            flattened_params[f'waypoints.{i}.position.z'] = position.get('z', 0.0)
            if 'dwell_time' in waypoint:
                flattened_params[f'waypoints.{i}.dwell_time'] = waypoint['dwell_time']
        
        # Add mission parameters
        mission_params = waypoints_data.get('mission_params', {})
        for key, value in mission_params.items():
            flattened_params[f'mission_params.{key}'] = value
        
        print(f"Loaded {len(waypoints)} waypoints for {drone_name}")
        return flattened_params
        
    except Exception as e:
        print(f"Error loading waypoints for {drone_name}: {e}")
        return {}


def spawn_multiple_drones_with_composition(context, *args, **kwargs):
    """Spawn multiple drones with composed mission planner + drone controller"""
    
    # Get launch configurations
    num_drones = int(context.launch_configurations['num_drones'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = context.launch_configurations['use_composition']
    use_perception = context.launch_configurations['use_perception']
    use_astar_planning = LaunchConfiguration('use_astar_planning')
    
    print(f"Spawning {num_drones} drones with composed controllers (composition: {use_composition}, perception: {use_perception})")
    
    # Get paths
    pkg_path = FindPackageShare('rs1_robot')
    config_path = PathJoinSubstitution([pkg_path, 'config'])
    
    # Get waypoints file path
    waypoints_file_path = PathJoinSubstitution([pkg_path, 'config', 'waypoints.yaml'])
    
    # Generate bridge config for multiple drones
    try:
        script_path = str(Path.home() / 'Software' / 'rs1_robot' / 'scripts' / 'generate_dynamic_bridge.py')
        result = subprocess.run(['python3', script_path, str(num_drones)], 
                              capture_output=True, text=True, check=True)
        print(f"Bridge config generated for {num_drones} drones")
    except Exception as e:
        print(f"Error generating bridge config: {e}")
        return []
    
    nodes = []
    
    # Create bridge node
    bridge_config_path = str(Path.home() / 'rs1_ws' / 'temp' / 'rs1_dynamic_bridge.yaml')
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    nodes.append(gazebo_bridge)
    
    # Create individual drone nodes for each drone
    for i in range(1, num_drones + 1):
        drone_name = f'rs1_drone_{i}'
        
        # Load waypoints for this specific drone
        waypoint_params = load_waypoints_for_drone(waypoints_file_path, drone_name)
        
        # Common parameters for both mission planner and drone controller
        mission_planner_params = {
            'use_sim_time': use_sim_time,
            'drone_namespace': drone_name,
            'mission_update_rate': 10.0,
            'waypoint_tolerance': 0.5,
            'use_astar_planning': use_astar_planning,
            # Add the loaded waypoint parameters
            **waypoint_params,
        }
        
        drone_controller_params = {
            'use_sim_time': use_sim_time,
            'drone_namespace': drone_name,
            'control_frequency': 50.0,
            'telemetry_frequency': 10.0,
            'kp_pos': 1.0,
            'ki_pos': 0.1,
            'kd_pos': 0.5,
            'kp_vel': 0.8,
            'ki_vel': 0.05,
            'kd_vel': 0.3,
            'kp_att': 0.8,
            'ki_att': 0.05,
            'kd_att': 0.3,
            'max_altitude': 50.0,
            'max_velocity': 5.0,
            'max_angular_velocity': 1.0,
            'safety_max_velocity': 10.0,
            'max_distance_from_home': 100.0
        }
        
        if use_composition == 'true':
            # Option 1: Use ComposableNodeContainer (separate containers per drone)
            drone_container = ComposableNodeContainer(
                name=f'drone_container_{i}',
                namespace=drone_name,
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='rs1_robot',
                        plugin='drone_swarm::MissionPlannerNode',
                        name='mission_planner',
                        parameters=[mission_planner_params],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='rs1_robot',
                        plugin='drone_swarm::DroneControllerNode', 
                        name='drone_controller',
                        parameters=[drone_controller_params],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='rs1_robot',
                        plugin='sensorNodeComponent',
                        name='sensor_processor',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'drone_namespace': drone_name
                        }],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ],
                output='screen'
            )
            nodes.append(drone_container)
            
            # Add perception node if enabled (always separate process, never composed)
            if use_perception == 'true':
                perception_node = Node(
                    package='rs1_perception',
                    executable='perception_node',
                    name=f'perception_node_{i}',
                    namespace=drone_name,
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'drone_namespace': drone_name,
                        'detector_type': 'umich',
                        'tag_family': 'tf36h11',
                        'image_transport': 'raw',
                        'front_camera_focal_length': 185.7,
                        'april_tag_size': 1.0,
                        'position_tolerance': 0.5,
                        'image_qos_profile': 'default'
                    }],
                    arguments=['--ros-args', '--log-level', 'info']
                )
                nodes.append(perception_node)
            
        elif use_composition == 'custom':
            # Option 2: Use custom composed executable
            drone_composed = Node(
                package='rs1_robot',
                executable='drone_mission_composed',  # New executable with mission + controller
                name=f'drone_mission_composed_{i}',
                output='screen',
                parameters=[{**mission_planner_params, **drone_controller_params}],
                arguments=['--ros-args', '--log-level', 'info']
            )
            nodes.append(drone_composed)
            
            # Separate sensor processor
            sensor_processor = Node(
                package='rs1_robot',
                executable='sensor_node',
                name=f'sensor_processor_{i}',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'drone_namespace': drone_name
                }],
                arguments=['--ros-args', '--log-level', 'info']
            )
            nodes.append(sensor_processor)
            
            # Add perception node if enabled (separate process, not composed)
            if use_perception == 'true':
                perception_node = Node(
                    package='rs1_perception',
                    executable='perception_node',
                    name=f'perception_node_{i}',
                    namespace=drone_name,
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'drone_namespace': drone_name,
                        'detector_type': 'umich',
                        'tag_family': 'tf36h11',
                        'image_transport': 'raw',
                        'front_camera_focal_length': 185.7,
                        'april_tag_size': 1.0,
                        'position_tolerance': 0.5,
                        'image_qos_profile': 'default'
                    }],
                    arguments=['--ros-args', '--log-level', 'info']
                )
                nodes.append(perception_node)
            
        else:
            # Option 3: Separate processes 
            mission_planner = Node(
                package='rs1_robot',
                executable='mission_planner_node',
                name=f'mission_planner_{i}',
                output='screen',
                parameters=[mission_planner_params],
                arguments=['--ros-args', '--log-level', 'info']
            )
            nodes.append(mission_planner)
            
            drone_controller = Node(
                package='rs1_robot',
                executable='drone_controller',
                name=f'drone_controller_{i}',
                output='screen',
                parameters=[drone_controller_params],
                arguments=['--ros-args', '--log-level', 'info']
            )
            nodes.append(drone_controller)
            
            sensor_processor = Node(
                package='rs1_robot',
                executable='sensor_node',
                name=f'sensor_processor_{i}',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'drone_namespace': drone_name
                }],
                arguments=['--ros-args', '--log-level', 'info']
            )
            nodes.append(sensor_processor)
            
            # Add perception node if enabled (separate process, not composed)
            if use_perception == 'true':
                perception_node = Node(
                    package='rs1_perception',
                    executable='perception_node',
                    name=f'perception_node_{i}',
                    namespace=drone_name,
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'drone_namespace': drone_name,
                        'detector_type': 'umich',
                        'tag_family': 'tf36h11',
                        'image_transport': 'raw',
                        'front_camera_focal_length': 185.7,
                        'april_tag_size': 1.0,
                        'position_tolerance': 0.5,
                        'image_qos_profile': 'default'
                    }],
                    arguments=['--ros-args', '--log-level', 'info']
                )
                nodes.append(perception_node)
    
    # Create robot description and spawner nodes for each drone
    for i in range(1, num_drones + 1):
        drone_name = f'rs1_drone_{i}'
        
        # Position drones in a line with 3m spacing
        x_pos = (i - 5) * 3.0  # Start at 0, then 3, 6, 9...
        y_pos = 0.0
        z_pos = 4.0
        
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
    
    print(f"Created {len(nodes)} nodes for {num_drones} drones")
    return nodes


def generate_launch_description():
    """Generate launch description for multi-drone system with composable nodes"""

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
        description='Number of drones to spawn (1-10)'
    )
    ld.add_action(num_drones_launch_arg)

    use_composition_launch_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Composition mode: true=container, custom=executable, false=separate'
    )
    ld.add_action(use_composition_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='world_1.sdf',
        description='Which world to load'
    )
    ld.add_action(world_launch_arg)
    
    # Perception launch argument
    use_perception_launch_arg = DeclareLaunchArgument(
        'use_perception',
        default_value='false',
        description='Flag to enable perception nodes for AprilTag detection'
    )
    ld.add_action(use_perception_launch_arg)
    
    # A* planning launch argument
    use_astar_planning_arg = DeclareLaunchArgument(
        'use_astar_planning',
        default_value='false',
        description='Flag to enable A* path planning with obstacle avoidance'
    )
    ld.add_action(use_astar_planning_arg)
    
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

    # Spawn multiple drones with composition options
    multiple_drones = OpaqueFunction(function=spawn_multiple_drones_with_composition)
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
