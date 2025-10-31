#!/bin/bash
# Script to start missions for all 4 drones using the mission planner services

NUM_DRONES=2

echo "Starting missions for $NUM_DRONES drones..."

# Start missions for all drones in parallel
for i in $(seq $NUM_DRONES -1 1); do
    echo "Starting mission for rs1_drone_$i"
    ros2 service call /rs1_drone_$i/start_mission std_srvs/srv/Trigger "{}" &
done

# Wait for all background processes to complete
wait

echo "All drone missions started!"
echo "Checking mission states..."

# Check mission states (from highest to lowest)
for i in $(seq $NUM_DRONES -1 1); do
    echo "rs1_drone_$i mission state:"
    ros2 topic echo /rs1_drone_$i/mission_state --once
done
