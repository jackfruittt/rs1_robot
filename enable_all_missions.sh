#!/bin/bash
# Script to enable missions for all drones collectively

NUM_DRONES=3

echo "Enabling missions for $NUM_DRONES drones..."

# Enable missions for all drones in parallel
for i in $(seq 1 $NUM_DRONES); do
    echo "Enabling mission for rs1_drone_$i"
    ros2 service call /rs1_drone_$i/mission/control std_srvs/srv/SetBool "data: true" &
done

# Wait for all background processes to complete
wait

echo "All drone missions enabled!"
