#!/bin/bash

# Simple multi-drone launcher - just calls the self-contained spawner
# Usage: ./launch_multi_drone_simple.sh <num_drones>
# Example: ./launch_multi_drone_simple.sh 3

set -e

# Default to 1 drone if no argument provided
NUM_DRONES=${1:-1}

# Validate input
if ! [[ "$NUM_DRONES" =~ ^[1-9][0-9]*$ ]] || [ "$NUM_DRONES" -gt 10 ]; then
    echo "Error: Please provide a valid number of drones (1-10)"
    echo "Usage: $0 <num_drones>"
    exit 1
fi

echo "=================================="
echo "Multi-Drone Launcher (Simple)"
echo "Spawning $NUM_DRONES drones..."
echo "=================================="

# Clean up any existing processes
echo "Cleaning up existing processes..."
pkill -f "ign gazebo" || true
pkill -f "parameter_bridge" || true
pkill -f "robot_state_publisher" || true
pkill -f "controller_node" || true
sleep 2

source install/setup.bash

echo "Launching multi-drone system..."
echo "World: simple_trees_builtin (built-in)"
echo "Drones: $NUM_DRONES"
echo ""

# Launch the self-contained spawner
ros2 launch rs1_robot rs1_drone_spawner.py num_drones:=$NUM_DRONES

echo "Multi-drone launch completed!"
