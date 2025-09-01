# NAV2 RRT Path Planner Integration Guide

This guide explains how to integrate the NAV2 toolbox with RRT path planning for autonomous drone navigation. The implementation will allow drones to plan their own paths without manual waypoint publishing - this ensures we only need to run the start mission service.

## Overview

The integration involves modifying the mission planning node to use NAV2's RRT planner for autonomous path generation, replacing the current manual waypoint system.

## Key Changes Required

## Install NAV2 Dependencies
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 1. Dependencies and Package Updates

#### CMakeLists.txt
Add these dependencies:
```cmake
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp_action REQUIRED)
```

#### package.xml
Add these dependencies:
```xml
<depend>nav2_msgs</depend>
<depend>nav2_util</depend>
<depend>nav2_core</depend>
<depend>rclcpp_action</depend>
<depend>nav_msgs</depend>
```

### 2. Launch Configuration

We'll need to launch the NAV2 stack alongside our drone system:

#### Create nav2_params.yaml
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # Or use RRT planner plugin
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

costmap_2d:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: True
    resolution: 0.05
```

### 3. Implementation Steps

Follow the TODO comments added to the source files:

#### A. Mission Node Changes (`mission_node.h` & `mission_node.cpp`)

1. **Add NAV2 includes** (already marked in mission_node.h)
2. **Initialize path planner client** in constructor
3. **Replace manual waypoint logic** with autonomous path planning
4. **Implement path planning request methods**
5. **Add replanning logic** for dynamic environments

#### B. Path Planner Updates (`path_planner.h` & `path_planner.cpp`)

1. **Add nav_msgs::Path support** for direct NAV2 integration
2. **Implement path conversion methods** to subsample waypoints
3. **Add logging** to distinguish autonomous vs manual waypoints

#### C. Optional: Drone Node Integration (`drone_node.h`)

1. **Add NAV2 controller subscriptions** for local obstacle avoidance
2. **Integrate with NAV2 local planner** for dynamic replanning

## Usage Flow

1. **System Startup**: Launch NAV2 stack with map server, costmap, and planner server
2. **Mission Start**: Call `/rs1_drone/start_mission` service
3. **Autonomous Planning**: Mission node requests path from start to goal using RRT
4. **Path Execution**: Planned path is converted to waypoints and followed
5. **Dynamic Replanning**: System replans if obstacles or environment changes

## Configuration Parameters

Add these parameters to your launch files:

```yaml
mission_planner:
  ros__parameters:
    # Existing parameters...
    replanning_frequency: 1.0      # Hz - how often to check for replanning
    planning_timeout: 10.0         # seconds - timeout for planning requests  
    planner_id: "RRT*"            # RRT planner variant to use
    global_frame: "map"           # Global coordinate frame
    min_waypoint_distance: 1.0    # meters - spacing between waypoints
```

## Key Benefits

- **Autonomous Navigation**: No manual waypoint publishing required
- **Obstacle Avoidance**: RRT naturally handles static obstacles
- **Dynamic Replanning**: Adapts to environment changes
- **Optimal Paths**: RRT* provides near-optimal path solutions
- **Scalable**: Works with multiple drones independently

## Testing Procedure

1. Start with simple open environments
2. Verify path planning requests are successful
3. Test waypoint conversion and following
4. Add obstacles and test replanning
5. Validate with multiple goal positions

## Notes

- Ensure map server is running with appropriate occupancy grid
- Costmap configuration is critical for safe navigation
- Consider altitude planning for 3D drone navigation
- Test with simulation before real-world deployment

The TODO comments in the source files provide specific implementation details and code examples for each step.
