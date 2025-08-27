#include "drone_node.h"
#include "sensor_node.h"


using namespace std::chrono_literals;

DroneNode::DroneNode()
    : Node("drone_controller"),
      should_terminate_(false),
      sensor_node_(nullptr)  // Initialise to nullptr
{
    // Declare parameters
    this->declare_parameter("road_gradient", 3.0); // Default 3%
    max_gradient_ = this->get_parameter("road_gradient").as_double();

    // Declare advanced parameter
    this->declare_parameter("advanced", false);
    advanced_mode_ = this->get_parameter("advanced").as_bool();

    // Declare drone namespace parameter
    this->declare_parameter("drone_namespace", std::string("rs1_drone"));
    drone_namespace_ = this->get_parameter("drone_namespace").as_string();

    // Initialise subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + drone_namespace_ + "/odom", 10, std::bind(&DroneNode::odomCallback, this, std::placeholders::_1));

    goals_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/" + drone_namespace_ + "/mission/goals", 10, std::bind(&DroneNode::goalCallback, this, std::placeholders::_1));

    grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "/grid_map", 10, std::bind(&DroneNode::gridMapCallback, this, std::placeholders::_1));

    // Subscribe to laser and sonar (For RTP)
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/" + drone_namespace_ + "/laserscan", 10, 
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(laser_mutex_);
            latest_laser_ = msg;
        });

    sonar_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/" + drone_namespace_ + "/sonar", 10,
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
            // Store sonar data
            {
                std::lock_guard<std::mutex> lock(sonar_mutex_);
                latest_sonar_ = msg;
            }
            
            // If the drone is flying, update altitude adjustment
            if (!vehicles_.empty() && vehicles_[0]->takeoff_complete) {
                // Get current altitude
                double current_altitude = 0.0;
                {
                    std::lock_guard<std::mutex> lock(vehicles_[0]->odom_mutex);
                    current_altitude = vehicles_[0]->current_odom.pose.pose.position.z;
                }
                
                // Calculate and set altitude adjustment
                double adjustment = controller_->calculateAltitudeAdjustment(
                    current_altitude, msg->range);
                controller_->setAltitudeAdjustment(adjustment);
                
                // Debug output with proper formatting
                static int count = 0;
                if (++count % 10 == 0) {  // Log every 10 messages
                    RCLCPP_INFO(this->get_logger(), 
                               "Sonar: alt=%.2fm, range=%.2fm, adjustment=%.2fm/s",
                               current_altitude, static_cast<double>(msg->range), adjustment);
                }
            }
        });

    // Initialise publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + drone_namespace_ + "/cmd_vel", 10);
    takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("/" + drone_namespace_ + "/takeoff", 10);
    land_pub_ = this->create_publisher<std_msgs::msg::Empty>("/" + drone_namespace_ + "/landing", 10);
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/" + drone_namespace_ + "/mission/path", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/" + drone_namespace_ + "/visualization_marker", 10);

    // Initialise services
    mission_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/" + drone_namespace_ + "/mission/control",
        std::bind(&DroneNode::missionControlCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialise timer for periodic updates
    timer_ = this->create_wall_timer(
        500ms, std::bind(&DroneNode::timerCallback, this));

    // Create controller instance
    controller_ = std::make_unique<Controller>(cmd_vel_pub_, takeoff_pub_, land_pub_, this->get_logger());
    
    // TODO: Re-enable when waypoint_manager.h is available
    // // Create waypoint manager
    // waypoint_manager_ = std::make_unique<WaypointManager>(
    //     this->get_logger(),
    //     max_gradient_,
    //     waypoints_pub_,
    //     markers_pub_);
    // 
    // // Set the elevation function to use map data
    // waypoint_manager_->setElevationFunction(
    //     [this](const geometry_msgs::msg::Point& point) { 
    //         return this->getElevationAtPoint(point); 
    //     });

    // // FIXED: Using real sensor data for gradient calculation
    // waypoint_manager_->setGradientFunction([this](double x, double y) {
    //     if (sensor_node_) {
    //         // Get real gradient data from sensor node
    //         return sensor_node_->getGradientMagnitudeAtPoint(x, y);
    //     } else {
    //         // Fallback to default if sensor node not available yet
    //         RCLCPP_DEBUG(this->get_logger(), "Sensor node not available, using default gradient");
    //         return 0.02;  // 2% gradient - safe default
    //     }
    // });

    // // Create TSP solver for advanced mode
    // tsp_solver_ = std::make_unique<TSPSolver>(this->get_logger());

    // Create main drone vehicle
    std::unique_ptr<VehicleData> main_drone = std::make_unique<VehicleData>("main_drone");
    vehicles_.push_back(std::move(main_drone));

    // Start controller thread
    controller_thread_ = new std::thread(&DroneNode::controllerThread, this);

    RCLCPP_INFO(this->get_logger(), "Drone controller initialized with max gradient: %.1f%%, advanced mode: %s", 
                max_gradient_, advanced_mode_ ? "ENABLED" : "DISABLED");
}

DroneNode::~DroneNode()
{
    // Set termination flag and wait for controller thread to finish
    should_terminate_ = true;

    // Notify all waiting threads
    for (auto &vehicle : vehicles_)
    {
        vehicle->queue_cv.notify_all();
    }

    if (controller_thread_ != nullptr && controller_thread_->joinable())
    {
        controller_thread_->join();
        delete controller_thread_;
    }

    // Land all drones that are still flying
    for (auto &vehicle : vehicles_)
    {
        if (vehicle->mission_active || vehicle->takeoff_complete)
        {
            controller_->land(*vehicle);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Drone controller shutting down");
}

void DroneNode::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    if (vehicles_.empty())
    {
        return;
    }

    // Update main drone odometry (can update this later if doing multi-drone, i.e. specifying drone id)
    auto &vehicle = vehicles_[0];
    std::lock_guard<std::mutex> lock(vehicle->odom_mutex);
    vehicle->current_odom = *msg;
}

void DroneNode::gridMapCallback(const std::shared_ptr<grid_map_msgs::msg::GridMap> msg)
{
    // Store the received grid map
    std::lock_guard<std::mutex> lock(grid_map_mutex_);
    latest_grid_map_ = *msg;
    has_grid_map_ = true;
}

void DroneNode::missionControlCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (vehicles_.empty())
    {
        response->success = false;
        response->message = "No vehicles available";
        return;
    }

    auto &vehicle = vehicles_[0];

    if (request->data)
    {
        // START MISSION
        if (!vehicle->mission_active)
        {
            // Check if we have goals to work with
            {
                std::lock_guard<std::mutex> lock(vehicle->goals_mutex);
                if (vehicle->goals.empty()) {
                    response->success = false;
                    response->message = "No goals available. Please publish goals to /mission/goals first.";
                    return;
                }
            }

            // Store starting position
            {
                std::lock_guard<std::mutex> lock_odom(vehicle->odom_mutex);
                vehicle->start_position.position = vehicle->current_odom.pose.pose.position;
                vehicle->start_position.orientation = vehicle->current_odom.pose.pose.orientation;
            }
            
            // Validate goals are within terrain map boundaries
            if (!validateAllGoals(*vehicle)) {
                response->success = false;
                response->message = "Goals are outside terrain map boundaries (20x20x20m). Mission aborted.";
                return;
            }

            // TODO: Re-enable when Graph class is implemented
            // // Build traversability graph for route planning
            // auto graph = buildTraversabilityGraph(*vehicle);
            // graph->printGraphInfo(this->get_logger());

            // Start mission
            controller_->startMission(*vehicle);

            response->success = true;
            response->message = "Mission started - drone will survey terrain and navigate goals";
            RCLCPP_INFO(this->get_logger(), "Mission mode activated for %s with %zu goals", 
                       vehicle->id.c_str(), vehicle->goals.size());
        }
        else
        {
            // Already in mission mode
            response->success = true;
            response->message = "Mission mode already active - progress: " + 
                               std::to_string(static_cast<int>(vehicle->mission_progress)) + "%";
        }
    }
    else
    {
        // STOP MISSION
        if (vehicle->mission_active)
        {
            controller_->stopMission(*vehicle);
            response->success = true;
            response->message = "Mission stopped - drone will return and land";
            RCLCPP_INFO(this->get_logger(), "Mission mode deactivated for %s", vehicle->id.c_str());
        }
        else
        {
            response->success = true;
            response->message = "Mission mode already inactive";
        }
    }
}

void DroneNode::goalCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg)
{
    if (vehicles_.empty())
    {
        return;
    }

    auto &vehicle = vehicles_[0];

    RCLCPP_INFO(this->get_logger(), "Received %zu new goals - mode: %s",
                msg->poses.size(), advanced_mode_ ? "ADVANCED (TSP)" : "BASIC (Sequential)");

    // Store new goals in the vehicle's goals vector
    storeMissionGoals(*vehicle, msg->poses);

    // Log altitude mode for each goal
    for (size_t i = 0; i < msg->poses.size(); i++) {
        const auto& goal = msg->poses[i];
        if (goal.position.z > 0.1) {
            RCLCPP_INFO(this->get_logger(), "Goal %zu: (%.2f, %.2f, %.2f) - MANUAL ALTITUDE", 
                       i, goal.position.x, goal.position.y, goal.position.z);
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal %zu: (%.2f, %.2f, terrain) - TERRAIN FOLLOWING", 
                       i, goal.position.x, goal.position.y);
        }
    }

    // ADVANCED MODE: Use TSP to optimise goal order
    if (advanced_mode_ && msg->poses.size() > 1) {
        RCLCPP_INFO(this->get_logger(), "ADVANCED MODE: Running TSP solver for %zu goals...", msg->poses.size());
        
        // TODO: Implement TSP solver
        // if (!solveTSPAndReorderGoals(*vehicle)) {
        //     RCLCPP_ERROR(this->get_logger(), "TSP solver failed - falling back to sequential order");
        // }
    } else if (advanced_mode_) {
        RCLCPP_INFO(this->get_logger(), "ADVANCED MODE: Single goal, no TSP optimization needed");
    }

    // HANDLE ALL MISSION STATES
    if (vehicle->mission_active && vehicle->takeoff_complete)
    {
        // Validate if goals are within terrain map boundaries
        if (!validateAllGoals(*vehicle)) {
            RCLCPP_ERROR(this->get_logger(), "Some goals are outside terrain map boundaries. Returning to origin and landing.");
            returnToOriginAndLand(*vehicle);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "MISSION ACTIVE - Adding new goals to queue");

        // ALWAYS add new goals to the command queue
        {
            std::lock_guard<std::mutex> lock(vehicle->queue_mutex);
            
            // Clear existing commands and add new goals
            std::queue<VehicleData::Command> empty;
            std::swap(vehicle->command_queue, empty);
            
            // Add new goal commands in order
            std::vector<geometry_msgs::msg::Pose> ordered_goals;
            {
                std::lock_guard<std::mutex> goals_lock(vehicle->goals_mutex);
                ordered_goals = vehicle->goals;
            }
            
            for (const auto &goal : ordered_goals)
            {
                vehicle->command_queue.push(
                    VehicleData::Command(VehicleData::Command::Type::FLYING, goal));
            }
            
            RCLCPP_INFO(this->get_logger(), "Added %zu new goals to command queue", ordered_goals.size());
        }

        // Reset mission progress for new goals
        vehicle->current_goal_index = 0;
        vehicle->mission_progress = 0.0;

        // Notify controller thread
        vehicle->queue_cv.notify_one();
        
        RCLCPP_INFO(this->get_logger(), "New goals queued for immediate execution");
    }
    else if (vehicle->mission_active && !vehicle->takeoff_complete)
    {
        // Mission active but not taken off yet - goals will be used when takeoff completes
        RCLCPP_INFO(this->get_logger(), "Mission active but not airborne - goals will be used after takeoff");
    }
    else
    {
        // Mission not active - just store the goals for future use
        RCLCPP_INFO(this->get_logger(), "Goals stored. Start mission with service call to begin navigation.");
    }
}

bool DroneNode::validateAllGoals(VehicleData& vehicle)
{
    std::vector<geometry_msgs::msg::Pose> goals;
    {
        std::lock_guard<std::mutex> lock(vehicle.goals_mutex);
        goals = vehicle.goals;
    }

    if (goals.empty()) {
        RCLCPP_WARN(this->get_logger(), "No goals available to validate");
        return false;
    }

    // Check if goals are within terrain map boundaries (40x40x20m centered at origin)
    const double MAX_X = 100.0;  // -100 to +100
    const double MAX_Y = 100.0;  // -100 to +100
    const double MAX_Z = 100.0;  // 0 to +100

    for (size_t i = 0; i < goals.size(); i++) {
        const auto& goal = goals[i];
        
        if (std::abs(goal.position.x) > MAX_X || 
            std::abs(goal.position.y) > MAX_Y || 
            goal.position.z < 0 || goal.position.z > MAX_Z) {
            
            RCLCPP_ERROR(this->get_logger(), 
                        "Goal %zu at (%.2f, %.2f, %.2f) is outside terrain map boundaries (±10m x,y; 0-20m z)",
                        i, goal.position.x, goal.position.y, goal.position.z);
            return false;
        }
    }

    RCLCPP_INFO(this->get_logger(), "All %zu goals are within terrain map boundaries", goals.size());
    return true;
}

void DroneNode::returnToOriginAndLand(VehicleData& vehicle) {
    RCLCPP_WARN(this->get_logger(), "Returning to origin (0,0) and landing due to untraversable goals");
    
    // Clear existing command queue
    {
        std::lock_guard<std::mutex> lock(vehicle.queue_mutex);
        std::queue<VehicleData::Command> empty;
        std::swap(vehicle.command_queue, empty);
        
        // Create return-to-origin goal
        geometry_msgs::msg::Pose origin_pose;
        origin_pose.position.x = 0.0;
        origin_pose.position.y = 0.0;
        origin_pose.position.z = 2.0;  // 2m altitude for safe flight
        
        // Set orientation to face forward (0 degrees)
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        origin_pose.orientation = tf2::toMsg(q);
        
        // Add command to return to origin
        vehicle.command_queue.push(
            VehicleData::Command(VehicleData::Command::Type::FLYING, origin_pose));
        
        // Add landing command
        vehicle.command_queue.push(
            VehicleData::Command(VehicleData::Command::Type::LANDING));
        
        // Add stop command
        vehicle.command_queue.push(
            VehicleData::Command(VehicleData::Command::Type::STOP));
        
        // Mark mission as failed
        vehicle.mission_active = false;
    }
    
    // Notify controller thread
    vehicle.queue_cv.notify_one();
}

void DroneNode::timerCallback() {
    if (vehicles_.empty()) {
        return;
    }

    auto &vehicle = vehicles_[0];
    
    // Always publish waypoints and markers if we have goals, regardless of mission state
    std::vector<geometry_msgs::msg::Pose> current_goals;
    {
        std::lock_guard<std::mutex> lock(vehicle->goals_mutex);
        current_goals = vehicle->goals;
    }
    
    if (!current_goals.empty()) {
        // TODO: Re-enable when waypoint_manager is implemented
        // // Always publish waypoints and markers when we have goals
        // waypoint_manager_->publishWaypoints(*vehicle, this->now());
        // waypoint_manager_->publishMarkers(*vehicle, this->now());
        
        RCLCPP_DEBUG(this->get_logger(), "Would publish waypoints and markers for %zu goals", current_goals.size());
    }
    
    // Update mission progress if active
    if (vehicle->mission_active) {
        // Calculate progress based on completed goals
        std::lock_guard<std::mutex> lock(vehicle->goals_mutex);
        if (!vehicle->goals.empty()) {
            double progress = (static_cast<double>(vehicle->current_goal_index) / vehicle->goals.size()) * 100.0;
            vehicle->mission_progress = std::min(progress, 100.0);
        }
        
        // Log status periodically
        static int status_counter = 0;
        if (++status_counter % 10 == 0) {  // Every 5 seconds
            RCLCPP_INFO(this->get_logger(), 
                       "Mission Status: ACTIVE, Progress: %.1f%%, Goal: %zu/%zu",
                       vehicle->mission_progress.load(),
                       static_cast<size_t>(vehicle->current_goal_index.load()),
                       current_goals.size());
        }
    }
}

void DroneNode::controllerThread()
{
    RCLCPP_INFO(this->get_logger(), "Controller thread started");
    rclcpp::Rate rate(10); // 10Hz for responsive control
    
    while (!should_terminate_)
    {
        for (auto &vehicle : vehicles_)
        {
            // Get sensor data
            sensor_msgs::msg::LaserScan::SharedPtr laser;
            sensor_msgs::msg::Range::SharedPtr sonar;
            {
                std::lock_guard<std::mutex> lock(laser_mutex_);
                laser = latest_laser_;
            }
            {
                std::lock_guard<std::mutex> lock(sonar_mutex_);
                sonar = latest_sonar_;
            }
           
            // Continuous obstacle detection
            if (laser && vehicle->mission_active && vehicle->takeoff_complete) {
                geometry_msgs::msg::Point current_pos;
                {
                    std::lock_guard<std::mutex> lock(vehicle->odom_mutex);
                    current_pos = vehicle->current_odom.pose.pose.position;
                }
                
                geometry_msgs::msg::Point target_pos = current_pos;
                bool has_target = false;
                
                {
                    std::lock_guard<std::mutex> lock(vehicle->goals_mutex);
                    int current_goal = vehicle->current_goal_index.load();
                    if (current_goal < static_cast<int>(vehicle->goals.size())) {
                        target_pos = vehicle->goals[current_goal].position;
                        has_target = true;
                    }
                }
                
                if (has_target) {
                    bool obstacle_in_path = controller_->checkObstaclesInDirection(
                        laser, current_pos, target_pos);
                    
                    // Obstacle avoidance state machine
                    if (obstacle_in_path && !vehicle->avoiding_obstacle) {
                        vehicle->avoiding_obstacle = true;
                        vehicle->obstacle_avoidance_start_altitude = current_pos.z;
                        
                        RCLCPP_ERROR(this->get_logger(), 
                                   "STARTING OBSTACLE AVOIDANCE - ascending from %.2fm!", 
                                   current_pos.z);
                    }
                    
                    if (vehicle->avoiding_obstacle) {
                        bool obstacle_cleared = controller_->isObstacleClearanceAchieved(
                            laser, current_pos, target_pos, 4.0);
                        
                        bool altitude_sufficient = (current_pos.z > vehicle->obstacle_avoidance_start_altitude + 3.0);
                        
                        if (obstacle_cleared && altitude_sufficient) {
                            vehicle->avoiding_obstacle = false;
                            RCLCPP_INFO(this->get_logger(), 
                                       "OBSTACLE CLEARED - resuming normal flight at %.2fm", 
                                       current_pos.z);
                        } else {
                            geometry_msgs::msg::Twist ascent_cmd;
                            ascent_cmd.linear.x = 0.0;
                            ascent_cmd.linear.y = 0.0;
                            ascent_cmd.linear.z = 2.0;
                            cmd_vel_pub_->publish(ascent_cmd);
                            
                            controller_->setAltitudeAdjustment(2.0);
                            
                            continue;
                        }
                    }
                }
            }
            
            // Normal altitude adjustment
            if (sonar && vehicle->takeoff_complete && !vehicle->avoiding_obstacle) {
                double current_altitude;
                {
                    std::lock_guard<std::mutex> lock(vehicle->odom_mutex);
                    current_altitude = vehicle->current_odom.pose.pose.position.z;
                }
                
                double altitude_adjustment = controller_->calculateAltitudeAdjustment(
                    current_altitude, sonar->range);
                
                controller_->setAltitudeAdjustment(altitude_adjustment);
            }

            // Command processing
            if (!vehicle->avoiding_obstacle) {
                VehicleData::Command cmd;
                bool has_cmd = false;

                {
                    std::unique_lock<std::mutex> lock(vehicle->queue_mutex);
                    if (!vehicle->command_queue.empty())
                    {
                        cmd = vehicle->command_queue.front();
                        vehicle->command_queue.pop();
                        has_cmd = true;
                    }
                }

                if (has_cmd)
                {
                    bool success = controller_->processCommand(*vehicle, cmd);
                    
                    if (success && cmd.type == VehicleData::Command::Type::FLYING) {
                        // Successfully completed a flying command - increment goal index
                        vehicle->current_goal_index++;
                        
                        RCLCPP_INFO(this->get_logger(), 
                                   "Goal %d completed. Moving to next goal.", 
                                   vehicle->current_goal_index.load() - 1);
                        
                        // Check if we have more goals to process
                        {
                            std::lock_guard<std::mutex> goals_lock(vehicle->goals_mutex);
                            int current_goal_idx = vehicle->current_goal_index.load();
                            
                            if (current_goal_idx >= static_cast<int>(vehicle->goals.size())) {
                                RCLCPP_INFO(this->get_logger(), 
                                           "All goals completed! Drone will hover and wait for new goals.");
                                // Mission remains active, drone hovers waiting for new goals
                            }
                        }
                    }
                    
                    if (!success && cmd.type == VehicleData::Command::Type::FLYING) {
                        RCLCPP_ERROR(this->get_logger(), 
                                   "Failed to reach goal. Aborting mission.");
                        controller_->abortMission(*vehicle);
                    }
                } else {
                    // No commands in queue - check if mission is active and we're waiting for new goals
                    if (vehicle->mission_active && vehicle->takeoff_complete) {
                        // Hover in place while waiting for new goals with real-time altitude control
                        geometry_msgs::msg::Twist hover_cmd;
                        hover_cmd.linear.x = 0.0;
                        hover_cmd.linear.y = 0.0;
                        
                        // Calculate real-time altitude adjustment using sonar
                        if (sonar) {
                            double current_altitude;
                            {
                                std::lock_guard<std::mutex> lock(vehicle->odom_mutex);
                                current_altitude = vehicle->current_odom.pose.pose.position.z;
                            }
                            
                            // Get real-time altitude adjustment based on current sonar reading
                            double altitude_adjustment = controller_->calculateAltitudeAdjustment(
                                current_altitude, sonar->range);
                            hover_cmd.linear.z = altitude_adjustment;
                            
                            // Debug every 2 seconds during hover
                            static int hover_debug_counter = 0;
                            if (++hover_debug_counter % 20 == 0) {
                                RCLCPP_DEBUG(this->get_logger(), 
                                           "Hovering: altitude=%.2fm, sonar=%.2fm, adjustment=%.2fm/s", 
                                           current_altitude, static_cast<double>(sonar->range), altitude_adjustment);
                            }
                        } else {
                            // No sonar available - maintain current altitude
                            hover_cmd.linear.z = 0.0;
                            
                            static int no_sonar_warning_counter = 0;
                            if (++no_sonar_warning_counter % 100 == 0) {  // Every 10 seconds
                                RCLCPP_WARN(this->get_logger(), "Hovering without sonar - maintaining current altitude");
                            }
                        }
                        
                        cmd_vel_pub_->publish(hover_cmd);
                    }
                }
            }
        }

        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Controller thread stopped");
}

double DroneNode::getElevationAtPoint(const geometry_msgs::msg::Point& point) {
    std::lock_guard<std::mutex> lock(grid_map_mutex_);
    
    if (!has_grid_map_) {
        return 0.0;  // Default elevation if no grid map is available
    }
    
    try {
        // Convert the grid map message to grid_map::GridMap
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(latest_grid_map_, map);
        
        // Get elevation at specified point
        grid_map::Position position(point.x, point.y);
        if (map.isInside(position)) {
            return map.atPosition("elevation", position);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error getting elevation: %s", e.what());
    }
    
    return 0.0;  // Default elevation if point not in map
}

/*
std::unique_ptr<Graph> DroneNode::buildTraversabilityGraph(const VehicleData& vehicle) {
    std::vector<geometry_msgs::msg::Pose> goals;
    {
        std::lock_guard<std::mutex> lock(const_cast<VehicleData&>(vehicle).goals_mutex);
        goals = vehicle.goals;
    }
    
    size_t num_goals = goals.size();
    auto graph = std::make_unique<Graph>(num_goals);
    
    RCLCPP_INFO(this->get_logger(), "Building traversability graph for %zu goals", num_goals);
    
    // Build edges between all pairs of goals
    for (size_t i = 0; i < num_goals; i++) {
        for (size_t j = i + 1; j < num_goals; j++) {
            // For drone operations, all goals within terrain bounds are traversable (unless below ground)
            // (unlike ground vehicles which need to check gradients)
            double distance = controller_->calculateDistance(
                goals[i].position, goals[j].position);
            
            graph->addEdge(i, j, distance);
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "Added edge from goal %zu to goal %zu with distance %.2fm", 
                        i, j, distance);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Graph construction complete - all goals are traversable for drone");
    return graph;
}
*/

void DroneNode::setSensorNode(std::shared_ptr<SensorNode> sensor_node) {
    sensor_node_ = sensor_node;
    RCLCPP_INFO(this->get_logger(), "Sensor node connected to drone node");
}

void DroneNode::storeMissionGoals(VehicleData& vehicle, const std::vector<geometry_msgs::msg::Pose>& new_goals) {
    std::lock_guard<std::mutex> lock(vehicle.goals_mutex);
    vehicle.goals.clear();
    vehicle.goals = new_goals;
    
    RCLCPP_INFO(this->get_logger(), "Stored %zu mission goals in vehicle data structure", 
                new_goals.size());
}

// bool DroneNode::solveTSPAndReorderGoals(VehicleData& vehicle) {
    // TODO: Re-enable when TSP solver is implemented
 //   RCLCPP_INFO(this->get_logger(), "TSP solver not implemented - using goals in original order");
 //   return false;  // Return false to indicate TSP was not performed
    
    /*
    // Get current goals and drone position
    std::vector<geometry_msgs::msg::Pose> original_goals;
    geometry_msgs::msg::Point start_position;
    
    {
        std::lock_guard<std::mutex> lock(vehicle.goals_mutex);
        original_goals = vehicle.goals;
    }
    
    {
        std::lock_guard<std::mutex> lock(vehicle.odom_mutex);
        start_position = vehicle.current_odom.pose.pose.position;
    }
    
    if (original_goals.empty()) {
        RCLCPP_WARN(this->get_logger(), "TSP: No goals to optimize");
        return false;
    }
    
    // Cost function (3D Euclidean distance)
    auto cost_function = [this](const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to) -> double {
        return controller_->calculateDistance(from, to);
    };
    
    // Traversability function 
    auto traversability_function = [this](const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to) -> bool {
        // For drone missions, check basic bounds and reasonable altitude changes
        if (std::abs(from.x) > 20.0 || std::abs(from.y) > 20.0 || from.z < 0 || from.z > 20.0 ||
            std::abs(to.x) > 20.0 || std::abs(to.y) > 20.0 || to.z < 0 || to.z > 20.0) {
            return false;
        }
        
        // Check for reasonable altitude changes
        double altitude_diff = std::abs(to.z - from.z);
        double horizontal_distance = std::sqrt(std::pow(to.x - from.x, 2) + std::pow(to.y - from.y, 2));
        
        if (horizontal_distance > 0.1) {
            double climb_angle = std::atan(altitude_diff / horizontal_distance) * 180.0 / M_PI;
            if (climb_angle > 45.0) {  // Very steep climb/descent
                return false;
            }
        }
        
        return true;  // Path is traversable for drone
    };
    
    // Calculate baseline cost (sequential order) manually
    double baseline_cost = cost_function(start_position, original_goals[0].position);
    for (size_t i = 0; i < original_goals.size() - 1; ++i) {
        baseline_cost += cost_function(original_goals[i].position, original_goals[i+1].position);
    }
    
    RCLCPP_INFO(this->get_logger(), "TSP: Starting optimization for %zu goals, baseline cost: %.2fm", 
                original_goals.size(), baseline_cost);
    
    // Solve TSP
    auto solution = tsp_solver_->solveTSP(original_goals, start_position, cost_function, traversability_function);
    
    if (!solution.solution_found) {
        RCLCPP_ERROR(this->get_logger(), "TSP: No valid solution found!");
        return false;
    }
    
    // Reorder goals based on TSP solution
    std::vector<geometry_msgs::msg::Pose> optimized_goals;
    for (size_t idx : solution.path) {
        optimized_goals.push_back(original_goals[idx]);
    }
    
    // Update vehicle goals with optimized order
    {
        std::lock_guard<std::mutex> lock(vehicle.goals_mutex);
        vehicle.goals = optimized_goals;
    }
    
    // Publish visualisation of all valid TSP paths
    publishTSPPathVisualisation(solution, original_goals, start_position);
    
    // Calculate optimisation benefit
    double optimization_percentage = ((baseline_cost - solution.total_cost) / baseline_cost) * 100.0;
    
    RCLCPP_INFO(this->get_logger(), "TSP: Goals reordered successfully!");
    RCLCPP_INFO(this->get_logger(), "TSP: Baseline cost: %.2fm, Optimized cost: %.2fm", 
                baseline_cost, solution.total_cost);
    RCLCPP_INFO(this->get_logger(), "TSP: Optimization benefit: %.1f%% (saved %.2fm)", 
                optimization_percentage, baseline_cost - solution.total_cost);
    
    return true;
    */

bool DroneNode::isPathTraversable(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to) {
    // TODO: Re-enable when sensor data integration is complete
    // For now, assume all paths are traversable
    return true;
    
    /*
    // For drone operations, most paths are traversable unless terrain is too steep
    // Check terrain gradients along the path using the waypoint manager's gradient function
    
    // Sample points along the path
    const int num_samples = 10;
    for (int i = 0; i <= num_samples; i++) {
        double t = static_cast<double>(i) / num_samples;
        double x = from.x + t * (to.x - from.x);
        double y = from.y + t * (to.y - from.y);
        
        // Get gradient at this point
        if (sensor_node_) {
            double gradient = sensor_node_->getGradientMagnitudeAtPoint(x, y);
            if (gradient > max_gradient_ / 100.0) {  // Convert percentage to decimal
                RCLCPP_DEBUG(this->get_logger(), "TSP: Path untraversable due to steep gradient %.1f%% at (%.2f, %.2f)", 
                           gradient * 100.0, x, y);
                return false;
            }
        }
    }
    
    return true;  // Path is traversable
    */
}

void DroneNode::publishTSPPathVisualisation(
    const int& solution,  // Temporary placeholder
    const std::vector<geometry_msgs::msg::Pose>& goals,
    const geometry_msgs::msg::Point& start_position) {
    
    // TODO: Re-enable when TSP solver is implemented
    RCLCPP_DEBUG(this->get_logger(), "TSP visualization not implemented");
    
    /*
    const TSPSolver::TSPSolution& solution,
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Publish all valid paths with different colors
    for (size_t path_idx = 0; path_idx < solution.all_valid_paths.size(); path_idx++) {
        const auto& path = solution.all_valid_paths[path_idx];
        double path_cost = solution.path_costs[path_idx];
        bool is_optimal = (path == solution.path);
        
        // Generate waypoints for this path
        std::vector<geometry_msgs::msg::Point> path_waypoints;
        path_waypoints.push_back(start_position);
        
        for (size_t goal_idx : path) {
            path_waypoints.push_back(goals[goal_idx].position);
        }
        
        // Create intermediate waypoints every 1m along the path
        std::vector<geometry_msgs::msg::Point> intermediate_waypoints;
        for (size_t i = 0; i < path_waypoints.size() - 1; i++) {
            auto from = path_waypoints[i];
            auto to = path_waypoints[i + 1];
            
            double dx = to.x - from.x;
            double dy = to.y - from.y;
            double dz = to.z - from.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            int num_segments = std::max(1, static_cast<int>(std::ceil(distance / 1.0)));  // 1m spacing
            
            for (int j = 0; j <= num_segments; j++) {
                double t = static_cast<double>(j) / num_segments;
                geometry_msgs::msg::Point waypoint;
                waypoint.x = from.x + t * dx;
                waypoint.y = from.y + t * dy;
                waypoint.z = from.z + t * dz;
                
                // Ensure minimum 2m altitude for visualization
                if (waypoint.z < 2.0) {
                    waypoint.z = 2.0;
                }
                
                intermediate_waypoints.push_back(waypoint);
            }
        }
        
        // Create markers for intermediate waypoints
        for (size_t wp_idx = 0; wp_idx < intermediate_waypoints.size(); wp_idx++) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = is_optimal ? "tsp_optimal_path" : "tsp_alternative_path";
            marker.id = static_cast<int>(path_idx * 1000 + wp_idx);  // Unique ID
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position = intermediate_waypoints[wp_idx];
            marker.pose.orientation.w = 1.0;
            
            // Size
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            
            // Color based on path type
            if (is_optimal) {
                // Optimal path: bright green
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.9;
            } else {
                // Alternative paths: different colors based on relative cost
                double cost_ratio = path_cost / solution.total_cost;
                marker.color.r = std::min(1.0, cost_ratio);  // More red for expensive paths
                marker.color.g = 0.0;
                marker.color.b = std::max(0.0, 2.0 - cost_ratio);  // More blue for cheaper paths
                marker.color.a = 0.5;  // Semi-transparent
            }
            
            marker.lifetime = rclcpp::Duration(0, 0);  // Persistent
            marker_array.markers.push_back(marker);
        }
    }
    
    // Publish the marker array
    markers_pub_->publish(marker_array);
    
    RCLCPP_INFO(this->get_logger(), "TSP: Published visualization of %zu valid paths (%zu total waypoints)", 
                solution.all_valid_paths.size(), marker_array.markers.size());
    */
}