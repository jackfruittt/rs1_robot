#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# 1. Add the necessary imports for QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty, String
import time
import threading

# --- Configuration ---
NUM_DRONES = 4
TIMEOUT_SEC = 2.0
# ---------------------

class SwarmStatusChecker(Node):
    def __init__(self, num_drones):
        super().__init__('swarm_status_checker')
        self.num_drones = num_drones
        self.drone_responses = {}
        self.response_lock = threading.Lock()

        # 2. Create a reliable QoS profile
        reliable_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publishers to ping each drone
        self.ping_publishers = []
        for i in range(1, self.num_drones + 1):
            topic = f'/rs1_drone_{i}/info_request'
            # 3. Use this profile in the publisher creation
            self.ping_publishers.append(self.create_publisher(Empty, topic, qos_profile=reliable_qos_profile))
            self.get_logger().info(f"Advertising on ping topic: {topic}")

        # Create subscribers to listen for manifest responses
        self.manifest_subscribers = []
        for i in range(1, self.num_drones + 1):
            topic = f'/rs1_drone_{i}/info_manifest'
            callback = lambda msg, drone_id=i: self.manifest_callback(msg, drone_id)
            # 4. Use this profile in the subscriber creation
            self.manifest_subscribers.append(self.create_subscription(String, topic, callback, qos_profile=reliable_qos_profile))
            self.get_logger().info(f"Listening on manifest topic: {topic}")

    def manifest_callback(self, msg, drone_id):
        with self.response_lock:
            if drone_id not in self.drone_responses:
                self.drone_responses[drone_id] = msg.data
                self.get_logger().info(f"Received response from Drone {drone_id}")

    def run_check(self):
        self.get_logger().info("Giving subscribers a moment to connect...")
        time.sleep(0.5)

        self.get_logger().info(f"Pinging {self.num_drones} drones...")
        ping_msg = Empty()
        for pub in self.ping_publishers:
            pub.publish(ping_msg)

        self.get_logger().info(f"Waiting {TIMEOUT_SEC} seconds for responses...")
        time.sleep(TIMEOUT_SEC)

        self.get_logger().info("Check complete. Shutting down.")
        self.print_summary()

    def print_summary(self):
        print("\n" + "="*60)
        print("          DRONE SWARM STATUS REPORT")
        print("="*60)
        
        if not self.drone_responses:
            print("No responses received from any drones.")
            print("="*60)
            return

        sorted_ids = sorted(self.drone_responses.keys())

        for drone_id in sorted_ids:
            manifest = self.drone_responses[drone_id]
            parts = manifest.split(',')
            status_data = {}
            for part in parts:
                key_val = part.split(':')
                if len(key_val) == 2:
                    status_data[key_val[0].strip()] = key_val[1].strip()
            
            print(f"\n--- Drone ID: {drone_id} ---")
            print(f"  - State:    {status_data.get('state', 'N/A')}")
            try:
                battery_val = float(status_data.get('battery', '0.0')) * 100
                print(f"  - Battery:  {battery_val:.1f}%")
            except (ValueError, TypeError):
                print(f"  - Battery:  N/A")

            print(f"  - Position: (x={status_data.get('x', 'N/A')}, y={status_data.get('y', 'N/A')}, z={status_data.get('z', 'N/A')})")
        
        print("\n" + "="*60)


def main(args=None):
    rclpy.init(args=args)
    status_checker = SwarmStatusChecker(num_drones=NUM_DRONES)
    
    check_thread = threading.Thread(target=status_checker.run_check)
    check_thread.start()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(status_checker)
    
    start_time = time.time()
    while rclpy.ok() and time.time() - start_time < TIMEOUT_SEC + 1.0:
        executor.spin_once(timeout_sec=0.1)

    check_thread.join()
    
    status_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()