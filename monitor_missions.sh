#!/bin/bash
# Script to monitor mission status for all 4 drones

NUM_DRONES=4

echo "Monitoring mission status for $NUM_DRONES drones..."
echo "Press Ctrl+C to stop monitoring"
echo ""

while true; do
    clear
    echo "=== DRONE MISSION STATUS ==="
    echo "$(date)"
    echo ""
    
    for i in $(seq 1 $NUM_DRONES); do
        echo "--- rs1_drone_$i ---"
        
        # Get mission state
        echo -n "Mission State: "
        timeout 2s ros2 topic echo /rs1_drone_$i/mission_state --once 2>/dev/null | grep "data:" | cut -d'"' -f2 || echo "No data"
        
        # Get current target pose
        echo -n "Target Position: "
        timeout 2s ros2 topic echo /rs1_drone_$i/target_pose --once 2>/dev/null | grep -A3 "position:" | tail -3 | tr '\n' ' ' | sed 's/[^0-9.-]/ /g' | awk '{print "["$1", "$2", "$3"]"}' || echo "No target"
        
        echo ""
    done
    
    echo "Available commands:"
    echo "  ./enable_all_missions.sh     - Start all missions"
    echo "  ./send_missions.sh [pattern] - Send waypoints"
    echo "  ./stop_all_missions.sh       - Stop all missions"
    echo ""
    
    sleep 3
done
