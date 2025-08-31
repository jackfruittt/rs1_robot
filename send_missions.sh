#!/bin/bash
# Script to send waypoints to all 4 drones

NUM_DRONES=4

echo "Sending waypoints to $NUM_DRONES drones..."

# Define waypoint patterns for each drone to avoid collisions
# Drone 1: Square pattern in positive X,Y quadrant
# Drone 2: Square pattern in negative X, positive Y quadrant  
# Drone 3: Square pattern in negative X,Y quadrant
# Drone 4: Square pattern in positive X, negative Y quadrant

send_waypoint() {
    local drone_id=$1
    local x=$2
    local y=$3
    local z=$4
    
    echo "Sending waypoint to rs1_drone_$drone_id: [$x, $y, $z]"
    
    ros2 topic pub --once /rs1_drone_$drone_id/waypoint_command geometry_msgs/msg/PoseStamped "{
        header: {
            stamp: {sec: 0, nanosec: 0},
            frame_id: 'map'
        },
        pose: {
            position: {x: $x, y: $y, z: $z},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
    }" &
}

# Send waypoints to each drone with different patterns
case "${1:-square}" in
    "square")
        echo "Sending square formation waypoints..."
        
        # Drone 1: Square in +X,+Y quadrant (5m x 5m square)
        send_waypoint 1 5.0 5.0 3.0
        sleep 1
        send_waypoint 1 10.0 5.0 3.0
        sleep 1  
        send_waypoint 1 10.0 10.0 3.0
        sleep 1
        send_waypoint 1 5.0 10.0 3.0
        
        # Drone 2: Square in -X,+Y quadrant
        send_waypoint 2 -5.0 5.0 3.0
        sleep 1
        send_waypoint 2 -10.0 5.0 3.0
        sleep 1
        send_waypoint 2 -10.0 10.0 3.0
        sleep 1
        send_waypoint 2 -5.0 10.0 3.0
        
        # Drone 3: Square in -X,-Y quadrant
        send_waypoint 3 -5.0 -5.0 3.0
        sleep 1
        send_waypoint 3 -10.0 -5.0 3.0
        sleep 1
        send_waypoint 3 -10.0 -10.0 3.0
        sleep 1
        send_waypoint 3 -5.0 -10.0 3.0
        
        # Drone 4: Square in +X,-Y quadrant
        send_waypoint 4 5.0 -5.0 3.0
        sleep 1
        send_waypoint 4 10.0 -5.0 3.0
        sleep 1
        send_waypoint 4 10.0 -10.0 3.0
        sleep 1
        send_waypoint 4 5.0 -10.0 3.0
        ;;
        
    "line")
        echo "Sending line formation waypoints..."
        
        # Line formation with 5m spacing
        send_waypoint 1 5.0 0.0 3.0
        send_waypoint 2 0.0 0.0 3.0
        send_waypoint 3 -5.0 0.0 3.0
        send_waypoint 4 -10.0 0.0 3.0
        ;;
        
    "diamond")
        echo "Sending diamond formation waypoints..."
        
        # Diamond formation centered at origin
        send_waypoint 1 0.0 8.0 3.0    # North
        send_waypoint 2 -8.0 0.0 3.0   # West
        send_waypoint 3 0.0 -8.0 3.0   # South
        send_waypoint 4 8.0 0.0 3.0    # East
        ;;
        
    "circle")
        echo "Sending circular formation waypoints..."
        
        # Circular formation with 8m radius
        send_waypoint 1 8.0 0.0 3.0     # 0 degrees
        send_waypoint 2 0.0 8.0 3.0     # 90 degrees
        send_waypoint 3 -8.0 0.0 3.0    # 180 degrees
        send_waypoint 4 0.0 -8.0 3.0    # 270 degrees
        ;;
        
    "test")
        echo "Sending simple test waypoints..."
        
        # Simple close waypoints for testing
        send_waypoint 1 2.0 2.0 2.0
        send_waypoint 2 -2.0 2.0 2.0
        send_waypoint 3 -2.0 -2.0 2.0
        send_waypoint 4 2.0 -2.0 2.0
        ;;
        
    *)
        echo "Usage: $0 [square|line|diamond|circle|test]"
        echo "  square  - Send square formation waypoints (default)"
        echo "  line    - Send line formation waypoints"
        echo "  diamond - Send diamond formation waypoints"
        echo "  circle  - Send circular formation waypoints"
        echo "  test    - Send simple test waypoints"
        exit 1
        ;;
esac

# Wait for all waypoint commands to be sent
wait

echo "All waypoints sent successfully!"
echo ""
echo "Monitor mission progress with:"
echo "  ros2 topic echo /rs1_drone_X/mission_state"
echo "  ros2 topic echo /rs1_drone_X/target_pose"
echo ""
echo "Stop all missions with:"
echo "  ./stop_all_missions.sh"
