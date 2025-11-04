#!/bin/bash
# Script to start missions for all 4 drones using the mission planner services

NUM_DRONES=3

echo "Starting missions for $NUM_DRONES drones..."

# Start missions for all drones in parallel
ros2 service call /rs1_drone_1/start_mission std_srvs/srv/Trigger "{}" &

sleep 2

ros2 service call /rs1_drone_2/start_mission std_srvs/srv/Trigger "{}" & 

sleep 2

ros2 service call /rs1_drone_3/start_mission std_srvs/srv/Trigger "{}" 



