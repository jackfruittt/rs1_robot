import os
import time
import unittest
import pytest
import launch
import launch_ros
import launch_testing
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

# Global list to capture received messages
g_mission_assignment_messages = []
g_mission_state_messages = {}

# Test Node to interact with the swarm
class SwarmTestNode(launch_ros.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.spotter_drone_id = 1
        self.responder_drone_id = 2 # Assuming drone 2 will be selected

        # Subscribe to the responder's assignment topic
        self.assignment_sub = self.create_subscription(
            String,
            f'/rs1_drone_{self.responder_drone_id}/mission_assignment',
            self.assignment_callback,
            10
        )
        
        # Subscribe to all drone state topics
        self.state_subs = {}
        for i in range(1, 5):
            self.state_subs[i] = self.create_subscription(
                String,
                f'/rs1_drone_{i}/mission_state',
                lambda msg, drone_id=i: self.state_callback(msg, drone_id),
                10
            )

        # Publisher to trigger the scenario
        self.scenario_pub = self.create_publisher(
            String,
            f'/rs1_drone_{self.spotter_drone_id}/scenario_detection',
            10
        )

    def assignment_callback(self, msg):
        global g_mission_assignment_messages
        self.get_logger().info(f"TESTER NODE: Received mission assignment: '{msg.data}'")
        g_mission_assignment_messages.append(msg.data)

    def state_callback(self, msg, drone_id):
        global g_mission_state_messages
        # self.get_logger().info(f"TESTER NODE: Drone {drone_id} state: {msg.data}")
        g_mission_state_messages[drone_id] = msg.data
        
    def trigger_scenario(self):
        time.sleep(15) # Give time for nodes to initialize and discover each other
        msg = String()
        # Format: SCENARIO_NAME,severity,x,y,z,yaw,respond:1
        msg.data = "WILDFIRE,5,30.0,-15.0,20.0,0.0,respond:1"
        self.get_logger().info("TESTER NODE: Publishing mock WILDFIRE scenario...")
        self.scenario_pub.publish(msg)


@pytest.mark.launch_test
def generate_test_description():
    # Path to the launch file
    launch_file_path = os.path.join(
        get_package_share_directory('rs1_robot'),
        'launch',
        'rs1_swarm_composed.py'
    )

    # Launch the swarm with 4 drones, using separate processes for clear testing
    swarm_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments={
            'num_drones': '4',
            'use_composition': 'false', # Must be 'false' to ensure nodes have unique PIDs for testing
            'use_perception': 'false', # Disable perception to isolate mission logic
            'gazebo': 'false' # Run headless for CI/CD
        }.items()
    )

    # Create our test node
    test_node = SwarmTestNode(
        package='rs1_robot',
        executable='dummy_node', # A placeholder, since we are the node
        name='swarm_test_node',
        output='screen'
    )

    return launch.LaunchDescription([
        swarm_launch,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestSwarmCoordination(unittest.TestCase):
    def test_full_coordination_flow(self, test_node, proc_info, proc_output):
        # 1. Trigger the scenario from our test node
        test_node.trigger_scenario()
        
        # 2. Verify Ping/Pong Success
        # We check the output logs of the spotter drone for the "Ping complete" message
        try:
            success_msg = "Ping complete: 4/4 responses"
            self.assertTrue(
                proc_output.waitFor(success_msg, timeout=20.0),
                f"Did not find '{success_msg}' in logs. Communication failed."
            )
            print("✅ SUCCESS: Ping/Pong communication verified (4/4).")
        except Exception as e:
            pytest.fail(f"Ping/Pong test failed: {e}")

        # 3. Verify Mission Assignment
        # Check that our test node's subscriber received the assignment message
        start_time = time.time()
        assignment_received = False
        while time.time() - start_time < 10.0:
            if any("ASSIGN,FETCH_RT" in msg for msg in g_mission_assignment_messages):
                assignment_received = True
                break
            time.sleep(0.2)
        
        self.assertTrue(assignment_received, "Mission assignment message was not received by responder.")
        print("✅ SUCCESS: Mission assignment message was correctly sent and received.")
        
        # 4. Verify Mission Execution
        # Check that the responder drone switched to the correct state
        start_time = time.time()
        state_transitioned = False
        expected_state = "WAYPOINT_NAVIGATION"
        responder_id = test_node.responder_drone_id
        
        while time.time() - start_time < 10.0:
            if g_mission_state_messages.get(responder_id) == expected_state:
                state_transitioned = True
                break
            time.sleep(0.2)
            
        self.assertTrue(state_transitioned, f"Responder drone did not transition to '{expected_state}'.")
        print(f"✅ SUCCESS: Responder drone correctly transitioned to {expected_state}.")