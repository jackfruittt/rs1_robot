#!/bin/bash
# Script to stop missions for all 4 drones

NUM_DRONES=4

echo "Stopping missions for $NUM_DRONES drones..."

# Stop missions for all drones in parallel
for i in $(seq 1 $NUM_DRONES); do
    echo "Stopping mission for rs1_drone_$i"
    ros2 service call /rs1_drone_$i/stop_mission std_srvs/srv/Trigger "{}" &
done

# Wait for all background processes to complete
wait

echo "All drone missions stopped!"
echo "Checking final mission states..."

# Check final mission states
for i in $(seq 1 $NUM_DRONES); do
    echo "rs1_drone_$i final state:"
    ros2 topic echo /rs1_drone_$i/mission_state --once
done
