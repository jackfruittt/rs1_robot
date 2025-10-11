#!/usr/bin/env python3
"""
Simple diagnostic script to check LocalCostmapManager behavior
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from grid_map_msgs.msg import GridMap
from visualization_msgs.msg import MarkerArray
import time

class CostmapDiagnostic(Node):
    def __init__(self):
        super().__init__('costmap_diagnostic')
        
        self.drone_ns = 'rs1_drone_1'
        
        # Counters
        self.lidar_count = 0
        self.costmap_count = 0
        self.markers_count = 0
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, f'/{self.drone_ns}/lidar', self.lidar_callback, 10)
        
        self.costmap_sub = self.create_subscription(
            GridMap, f'/{self.drone_ns}/local_costmap', self.costmap_callback, 10)
            
        self.markers_sub = self.create_subscription(
            MarkerArray, f'/{self.drone_ns}/local_obstacles', self.markers_callback, 10)
        
        # Timer for status
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info(f"Monitoring topics for {self.drone_ns}")
        
    def lidar_callback(self, msg):
        self.lidar_count += 1
        
        # Check for obstacles in scan
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        close_obstacles = [r for r in valid_ranges if r < 3.0]  # Within 3m
        
        if self.lidar_count % 10 == 0:  # Every 10th scan
            self.get_logger().info(f"LiDAR: {len(valid_ranges)} valid points, {len(close_obstacles)} close obstacles")
    
    def costmap_callback(self, msg):
        self.costmap_count += 1
        self.get_logger().info(f"Received costmap update #{self.costmap_count} - Frame: {msg.header.frame_id}, Layers: {len(msg.layers)}")
    
    def markers_callback(self, msg):
        self.markers_count += 1
        self.get_logger().info(f"Received marker update #{self.markers_count} - Markers: {len(msg.markers)}")
    
    def print_status(self):
        self.get_logger().info(f"Status: LiDAR={self.lidar_count}, Costmap={self.costmap_count}, Markers={self.markers_count}")

def main():
    rclpy.init()
    node = CostmapDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()