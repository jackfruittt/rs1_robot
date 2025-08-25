from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    num = LaunchConfiguration('num_drones')
    declare_num = DeclareLaunchArgument('num_drones', default_value='3')

    nodes = []
    for i in range(1, 7):
        nodes.append(
            PushRosNamespace(f"drone{i}")
        )
        nodes.append(
            Node(
                package="rs1_robot",
                executable="waypoint_nav",
                name="waypoint_nav",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "loop_hz": 50.0,
                    "kp_lin": 0.8,
                    "kp_yaw": 1.5,
                    "max_speed": 1.0,
                    "max_yaw_rate": 1.0,
                    "arrive_xy_tol": 0.15,
                    "arrive_yaw_tol": 0.2,
                    "frame_id": "world",
                }],
            )
        )

    return LaunchDescription([declare_num] + nodes[:2*int(str(LaunchConfiguration('num_drones')) or '3')])
