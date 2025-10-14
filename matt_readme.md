Of course. Here is a comprehensive report detailing the current state of the drone incident response system for your team to continue development.

## **Project Handover Report: Drone Swarm Incident Response System**

**Date:** 15 October 2025
**Author:** Gemini AI
**Project:** RS1 Forest Management Drone Swarm
**Subject:** Status and handover of the autonomous incident response module.

### **1.0 Introduction**

This document outlines the design, implementation, and current status of the incident response system for the RS1 drone swarm project. The system's primary goal is to enable a swarm of autonomous drones to patrol a simulated national park, detect predefined incidents via perception, and coordinate an intelligent, multi-drone response.

The core of this work resides within the `rs1_robot` package, specifically in the `MissionPlannerNode` class (`mission_node.cpp` and `mission_node.h`). This node acts as the "brain" for each drone, managing its mission state, handling incoming incident data, and collaborating with its peers to delegate tasks efficiently. This report will detail the data flow from perception to action, break down the functionality of key C++ components and ROS 2 topics, and provide a clear path forward for completing the required features.

---

### **2.0 System Architecture and Data Flow**

The incident response logic follows a decoupled, event-driven architecture that leverages the ROS 2 topic system for communication. The general flow of data is designed to be modular and scalable, as illustrated in the provided diagram and implemented in the code.



The process can be summarized in four main stages:

1.  **Detection:** The `rs1_perception` package detects an incident (e.g., a wildfire represented by an AR tag) and publishes a notification.
2.  **Coordination:** The drone that detected the incident (the "spotter") receives this notification. It then initiates a coordination protocol by pinging all available drones in the swarm to gather critical information (e.g., location, battery level, current mission state).
3.  **Decision:** The spotter drone analyzes the data from its peers and uses a selection algorithm to determine the "best responder" for the task. This decision is based on factors like proximity, availability, and battery life.
4.  **Delegation & Action:** The spotter sends a mission assignment command to the chosen responder. The responder accepts the task and begins executing a predefined sequence of actions (e.g., fly to a depot, collect an item, fly to the incident). Simultaneously, the spotter drone enters a monitoring state, such as orbiting the incident area.

This entire workflow is managed within each drone's `MissionPlannerNode` instance, making the swarm a decentralized, collaborative system.

---

### **3.0 The Incident Response Workflow: A Code-Level Deep Dive**

The logic is almost entirely contained within `mission_node.cpp`. Understanding the sequence of function calls and ROS topic interactions is crucial for future development.

#### **Phase 1: Detection and Parsing**

* **Entry Point:** The process begins when a message is published on the `/rs1_drone_X/scenario_detection` topic.
* **Message Format:** The perception package sends this data as a `std_msgs::msg::String` containing a comma-separated value (CSV) string. The format is rigid:
    `"SCENARIO_NAME,severity,x,y,z,yaw,respond:1"`
    Example: `"WILDFIRE,7,55.3,-21.8,30.0,0.0,respond:1"`
* **Handling Function:** The `scenarioDetectionCallback` function subscribes to this topic. Upon receiving a message, it immediately calls the `parseScenarioMessage` utility function, which tokenizes the CSV string and populates a `ScenarioData` struct. This struct provides a clean, type-safe way to access the incident details. To avoid blocking the main ROS executor, the callback then spawns a new thread to handle the more complex coordination logic.

#### **Phase 2: Coordination and Responder Selection**

This is the most critical phase, where the swarm collaborates to make an intelligent decision.

* **Coordination Function:** The `sendMissionToLowestDrone` function is called within the new thread. It orchestrates the entire response.
* **Pinging Peers:** Its first action is to call `pingDronesForInfo`. This function sends an empty message on the `/rs1_drone_Y/info_request` topic for every known peer drone in the swarm.
* **Broadcasting Status:** When a drone receives this "ping" on its `info_request` topic, the `infoRequestPingCallback` is triggered. This callback calls `buildInfoManifestCsv` to generate a CSV string of its current status and broadcasts it on its `/rs1_drone_Y/info_manifest` topic. The format is:
    `"id:Y,battery:0.85,state:WAYPOINT_NAVIGATION,x:...,y:...,z:...,t:..."`
* **Gathering Responses:** The original spotter drone's `pingDronesForInfo` function listens on the `/info_manifest` topics of its peers for a set timeout period (5000ms), collecting the responses. The data from these manifests is parsed into a map of `DroneInfo` structs.
* **Making the Decision:** The collected `DroneInfo` data is passed to `selectBestResponderDrone`. This function filters out any drones that are unsuitable (e.g., low battery, in an uninterruptible state) and then sorts the remaining candidates by their distance to a reference point (currently the helipad). **The drone with the shortest distance is selected as the best responder.**

#### **Phase 3: Mission Delegation and Execution**

* **Assigning the Task:** Once the best responder is identified, `sendMissionToLowestDrone` formats another CSV string for mission assignment. This message is published on the responder's `/rs1_drone_Z/mission_assignment` topic. The format depends on the mission:
    * **Wildfire:** `"ASSIGN,FETCH_RT,depot_x,y,z,fire_x,y,z"`
    * **Hiker:** `"ASSIGN,HIKER_RESCUE,depot_x,y,z,hiker_x,y,z"`
* **Starting the Mission:** The responder drone receives this message in its `assignmentCallback` function. This function parses the command and mission coordinates, sets the appropriate internal state flags (e.g., `in_fetch_rt_ = true;`), populates its `PathPlanner` with the first leg of the journey (flying to the depot), and transitions its state machine into `WAYPOINT_NAVIGATION`. From this point, the drone's main `missionTimerCallback` loop takes over to execute the flight path.

---

### **4.0 Current Implementation Status and Path Forward**

While the core coordination framework is robust and functional, the specific, nuanced behaviors for each scenario are incomplete.

#### **What's Working ✅**

* **Core Infrastructure:** The system for scenario detection, inter-drone communication (ping/pong), best responder selection, and mission delegation is fully implemented and tested.
* **Responder's Initial Action:** For both the **Wildfire** and **Stranded Hiker** scenarios, the assigned responder drone correctly flies to the specified depot, simulates resource collection by landing and waiting for 2 seconds, and then takes off to proceed to the incident location.

#### **What Needs to Be Tested **
- I'm unsure if the delegation works properly or not. It's been on and off. Some things would work, then I needed to fix other things and it would break. I'm fairly sure there's an issue with timing, locks, or race conditions, specifically during the delegation process and receiving the information from the drones after the ping.

#### **What Needs to Be Implemented ⏳**
0.  The gui publish needs to be added. I can't remember when it disappeared from my code lol. It will be in one of the previous commits, but in all fairness, it needed to be revamped anyway.

1.  **Spotter Drone Behavior (Hiker & Wildfire):**
    * **Requirement:** After delegating a task, the spotter drone must transition into an `ORBIT_INCIDENT` state and circle above the scene.
    * **Action:** The `orbitIncident()` function in `mission_node.cpp` is currently an empty stub. This function needs to be implemented to generate a circular flight path around the incident coordinates and feed these waypoints to the `PathPlanner`. The call to change the spotter's state should be added in `sendMissionToLowestDrone` after a mission has been successfully sent to a peer.

2.  **Responder's Final Actions (Hiker):**
    * **Requirement:** After reaching the hiker, the responder must descend to 0.5m above them, wait for 10 seconds (simulating a medkit drop), and then return to its original patrol route.
    * **Action:** The logic in `waypointNavigation()` needs to be extended. When the hiker waypoint is reached, it should trigger this final sequence instead of simply hovering.

3.  **Responder's Looping Behavior (Wildfire):**
    * **Requirement:** The responder must continuously loop between the retardant depot and the fire until the incident is cleared.
    * **Action:** The logic in `waypointNavigation()` needs modification. When the fire waypoint is reached, instead of completing the mission, it should check a state flag (e.g., `wildfire_mission_active_`) and, if true, re-populate the `PathPlanner` with the depot and fire waypoints again to create the loop.

4.  **Debris Obstruction Scenario:**
    * **Requirement:** This scenario is simpler: notify the GUI, simulate taking a picture, and then ignore the same AR tag if seen again.
    * **Action:** An `else if` block for `"DEBRIS_OBSTRUCTION"` must be added to `scenarioDetectionCallback`. This block should call the existing `alertIncidentGui` function and add the incident's location to a `std::set` member variable to track and ignore previously reported debris.

5.  **GUI "Clear" Command:**
    * **Requirement:** A ROS topic must be created to allow a GUI to signal that an incident is resolved, causing all involved drones to return to their standard patrol.
    * **Action:** A new ROS subscriber and callback function (e.g., `clearIncidentCallback`) must be created. This callback would reset the relevant state flags (e.g., set `wildfire_mission_active_` to `false`) and transition the drones from their special states (`ORBIT_INCIDENT`, etc.) back to `WAYPOINT_NAVIGATION`.

6. **AR TAG PLACEMENT** 
    * The placement of the tags hasn't been done because I was occupied with getting the drone reactions to work. Alas, they did not and I perhaps bit off more than I could chew. 
    

# IF you need more context, here's some GPT stuff:
Multi-Drone Mission Planner: Data Flow, Behaviors, and Current Status (Team Report)
1) End-to-end data flow and how drone responses work
Node topology at a glance

Each vehicle runs a MissionPlannerNode inside its own namespace (/rs1_drone_<N>). The planner collaborates with a PathPlanner, a StateMachine, and a simple peer-discovery/selection layer. We launch multiple identical stacks via the Python launch file, which also optionally spins up perception and sensor components and generates a dynamic Gazebo bridge configuration.

Parameters and configuration

On startup the planner declares and reads operational parameters:

Core: drone_namespace, mission_update_rate, waypoint_tolerance, battery_level.

Fixed points: helipad_location.(x|y|z), retardant_depot.(x|y|z), medkit_depot.(x|y|z).

Mission YAML: waypoints are loaded by loadWaypointsFromParams() (flattened parameter keys like waypoints.0.position.x). If absent, loadFallbackWaypoints() installs a reasonable default per drone. Additional mission_params.* (e.g., takeoff_altitude, max_velocity) are logged and can override defaults.

The launch file (generate_launch_description() and spawn_multiple_drones_with_composition) ensures unique node names per drone, sets namespaces, loads flattened waypoint params per drone, and optionally enables perception.

Topics (per drone namespace)

Subscriptions

/<ns>/odom (nav_msgs/Odometry) → odomCallback updates current_pose_ for navigation and status reporting.

/<ns>/scenario_detection (std_msgs/String) → scenarioDetectionCallback parses perception alerts and triggers mission coordination in a detached thread.

/<ns>/info_request (std_msgs/Empty) → infoRequestPingCallback responds immediately with a status CSV.

/<ns>/mission_assignment (std_msgs/String) → assignmentCallback consumes assignments from peers.

Publishers

/<ns>/cmd_vel (geometry_msgs::Twist) → publishMissionCommand sends a simple P-controller velocity toward the active waypoint.

/<ns>/target_pose (geometry_msgs::PoseStamped) → current waypoint broadcast for visualization/diagnostics.

/<ns>/mission_state (std_msgs::String) → textual state from the StateMachine each timer tick.

/<ns>/info_manifest (std_msgs::String) → CSV status payload described below.

Peer wiring (created on demand)
For each discovered peer k, the planner creates:

Sub: /rs1_drone_k/info_manifest (fresh peer status).

Sub: /rs1_drone_k/odom (peer pose cache).

Pub: /rs1_drone_k/mission_assignment (to send missions).

Pub: /rs1_drone_k/info_request (to ping peers for a fresh manifest).

Discovery itself is handled by discoverPeerDrones(), which scans the ROS graph for topics matching ^/rs1_drone_([0-9]+)/odom$ and then calls createPeerSubscriptionForId(id) to wire up the endpoints safely with a mutex.

Wire formats (strings/CSV)

Scenario detection (from perception):
SCENARIO_NAME,severity,x,y,z,yaw,respond:1|0|true|false
Parsed by parseScenarioMessage(), which validates field count, converts numerics, and sets .valid.

Assignment commands (peer→peer):

Hiker rescue: ASSIGN,HIKER_RESCUE,depot_x,depot_y,depot_z,hiker_x,hiker_y,hiker_z

Fire retardant: ASSIGN,FETCH_RT,depot_x,depot_y,depot_z,fire_x,fire_y,fire_z
Parsed by assignmentCallback() with splitCSV, parseDouble, and guardrails on token counts.

Info manifest (status heartbeat):
id:<n>,battery:<0-1>,state:<STATE>,x:<m>,y:<m>,z:<m>,t:<ns>
Built by buildInfoManifestCsv() for self, parsed by infoManifestCallback() into PeerInfo and then used by pingDronesForInfo() to build DroneInfo for selection. missionStateToString() maps enums to strings.

Planner control loop and behaviors

The timer missionTimerCallback() (rate from mission_update_rate) orchestrates the mission:

State step (executeMission()):
Switches on MissionState and calls the per-state handler:

takeoff() simulates a 5-second ascent before transitioning to WAYPOINT_NAVIGATION or HOVERING if no path.

waypointNavigation() checks PathPlanner::hasNextWaypoint() and distance via isWaypointReached() (planner computes range from current_pose_). When a waypoint is reached it either advances, hovers at completion, or—if in special missions—initiates a landing at depots.

landing() simulates touchdown and holds briefly to model “pickup” events at depots (see “two-leg missions” below).

hovering(), manualControl(), emergency() exist; reaction states are stubs for now.

Special two-leg missions (implemented via flags & timestamps):

Stranded hiker: fly to medkit_depot_xyz_ → land and “collect” (pause) → set waypoint to hiker (hiker_target_xyz_) → set state to TAKEOFF → resume nav. Flags: in_hiker_rescue_, medkit_collected_, in_hiker_rescue_awaiting_takeoff_ and medkit_collect_stamp_.

Fetch retardant: fly to depot_xyz_ → land and “collect” → set waypoint to fire target (fetch_fire_target_) → TAKEOFF. Flags: in_fetch_rt_, fetch_landed_, fetch_land_stamp_.

Command publication:
publishMissionCommand() publishes target_pose and a clamped P-only cmd_vel while in WAYPOINT_NAVIGATION. In other states it publishes zero velocity only if the state implies the planner owns motion (IDLE/HOVERING/EMERGENCY) to avoid fighting external controllers.

Mission assignment and peer selection

scenarioDetectionCallback() detaches a thread so the executor stays responsive, then calls sendMissionToLowestDrone() for actionable scenarios.

sendMissionToLowestDrone():

Maps scenario → required state (targetStateForScenario).

Calls pingDronesForInfo(all_ids, 5000) to request fresh manifests; this marks old stamps as stale and waits (sleeping in small slices so callbacks run).

Filters candidates by response validity, battery (>= 0.5), and state transitions (canStateTransitionTo(curr, required)).

Sorts by Euclidean distance to the helipad and chooses the minimum.

If self: set flags + depot waypoint and transition to WAYPOINT_NAVIGATION.
If peer: publish an ASSIGN,... string to that peer’s /mission_assignment.

State machine discipline

canStateTransitionTo(current, target) encodes the allowed transitions among IDLE, TAKEOFF, WAYPOINT_NAVIGATION, HOVERING, LANDING, MANUAL_CONTROL, EMERGENCY, and the reaction states. The team chose the strict original matrix (e.g., IDLE → TAKEOFF or MANUAL_CONTROL only). If we later want to allow IDLE → WAYPOINT_NAVIGATION for direct “go there now” commands, a one-line change (already commented in code) enables it.

2) Deep-dive on current issues, troubleshooting, and project status
A. Peer info ingestion & ID mismatches

Symptom: During pings, some peers reported an id: that did not match the integer extracted from the subscription’s namespace (e.g., a drone publishing a stale or wrong id field). This caused us to discard otherwise good manifests.

Actions taken:

Logging was added in infoManifestCallback() to print both the peer (from wiring) and the msg id.

We changed the acceptance policy to trust the subscription wiring’s peer_id as the ground truth and merely warn on mismatch. We still store the manifest against peer_id.

Result: The log shows consistent “🔵 CB fired … ✅ Stored peer_info[k]” for all peers, and pingDronesForInfo() now returns valid, fresh DroneInfo for selection.

Next step: Optionally harden the manifest format or add a version/nonce so consumers can sanity-check freshness beyond timestamps.

B. Freshness during ping and blocking risks

Symptom: Selection requires fresh peer data, but waiting while holding locks or inside callbacks can starve the executor.

Actions taken:

pingDronesForInfo() marks existing stamps stale, sends pings, and then spins a wait loop with short sleeps so the subscription callbacks can run; selection is not run inside a subscription callback.

Scenario coordination is offloaded to a detached thread from scenarioDetectionCallback() to avoid blocking the executor.

Status: Working as intended; we’re getting timely “All responses received early!” events when all peers reply quickly.

C. Static timers in takeoff() and landing()

Symptom: takeoff() uses a function-static takeoff_start, and landing() uses a static std::map<drone_id_, time_point>. In a composable process (multiple nodes in one binary), function-statics are shared across instances, which can cause cross-talk if two drones hit the same function concurrently.

Actions taken: None yet—behavior appears correct because values are reset and the map is keyed by drone_id_.

Recommendation: Convert both to member fields so each node instance has its own timers. It’s a mechanical change and removes a subtle concurrency footgun.

D. CSV parsing robustness

Symptom: We output CSV with a proper csv_escape(), but input parsing for assignments/manifests uses naive splitCSV() (split on commas). If we ever include commas or quotes in free-text fields, input parsing will break.

Actions taken: Kept formats simple (no commas in inputs). Ensured assignment strings are numeric-only after the verb.

Recommendation: If we expand the protocol (e.g., free-text descriptions), either (1) use a minimal CSV reader that handles quoting or (2) switch to a semi-structured format (YAML-inline or JSON) and keep std_msgs::String.

E. Transition policy vs. UX

Symptom: Operators sometimes expect that dropping a waypoint should “just go” even if the drone is IDLE.

Actions taken: Left the original strict matrix in canStateTransitionTo(), with a commented hint to allow IDLE → WAYPOINT_NAVIGATION.

Decision for the team: If we want that UX, enable the extra transition and ensure controllers/guards (arming/altitude fences) still make sense.

F. Namespacing and composition hygiene

Symptom (past): Duplicate node names across drones when using composition.

Fix: Launch file now assigns unique names per drone for mission_planner, drone_controller, sensor_processor, etc., and puts everything under the correct namespace. This eliminated name collisions.

G. Reaction states are stubs

Status: wildFireReaction(), debrisReaction(), strandedHikerReaction(), and orbitIncident() are currently placeholders. The behavior for the two-leg missions is implemented at the planner level via flags and state transitions, not inside these handlers.

Recommendation: Implement one reaction state end-to-end (e.g., orbit: loiter with a radius around ScenarioData target, altitude bands by severity, and an abort path). This will serve as the pattern for the others.

3) Where we are right now

✅ Peer discovery & status exchange: Solid. We see reliable “CB fired” and “Stored peer_info[…]” logs across all drones.

✅ Mission selection: sendMissionToLowestDrone() filters by battery/state, sorts by distance to helipad, and picks a responder consistently.

✅ Assignment pipeline: Both self-assign and peer-assign paths function. Assignment strings parse correctly and set depot/target waypoints as intended.

✅ Navigation loop: Waypoint progression, depot landing, timed “collection” pause, and subsequent takeoff behave as designed. Velocity and target pose publishing are gated to avoid controller fighting.

✅ Launch/tooling: N-drone spawning, unique names, optional perception, dynamic bridge generation, and per-drone waypoint loading are in place.

⚠️ Technical debt: Static timers should become members; input CSV parsing is simplistic; reaction handlers need real behaviors; consider whether to enable IDLE → WAYPOINT_NAVIGATION.

4) Immediate action items (proposed)

Refactor timers to members in MissionPlannerNode (takeoff start time, landing timers), remove function-statics.

Implement one reaction state (recommend ORBIT_INCIDENT): accept a center and radius, publish a generated loiter path via PathPlanner, and add an exit condition/timer.

Decide on transition policy for operator UX (allow or disallow IDLE → WAYPOINT_NAVIGATION).

Harden inputs if we plan free-text: either proper CSV parsing with quotes or switch to JSON in std_msgs/String.