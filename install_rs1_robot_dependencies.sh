#!/bin/bash

################################################################################
# ROS2 Humble Dependencies Installation Script for rs1_robot Package
# 
# This script installs all required dependencies for the rs1_robot package
# Target: Ubuntu 22.04 with ROS2 Humble
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Function to check if ROS2 is installed
check_ros2_installation() {
    print_info "Checking for ROS2 Humble installation..."
    
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        print_info "ROS2 Humble detected at /opt/ros/humble"
        source /opt/ros/humble/setup.bash
    else
        print_error "ROS2 Humble not found. Please install ROS2 Humble first."
        print_info "Visit: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
}

# Function to update package lists
update_package_lists() {
    print_step "Updating package lists..."
    sudo apt-get update
    print_info "Package lists updated"
}

# Function to install system dependencies
install_system_dependencies() {
    print_step "Installing system dependencies..."
    
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool
    
    print_info "System dependencies installed"
}

# Function to initialise rosdep if needed
initialize_rosdep() {
    print_step "Checking rosdep initialization..."
    
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        print_info "Initializing rosdep..."
        sudo rosdep init
    else
        print_info "rosdep already initialized"
    fi
    
    print_info "Updating rosdep..."
    rosdep update
    print_info "rosdep updated"
}

# Function to install ROS2 core dependencies
install_ros2_core_dependencies() {
    print_step "Installing ROS2 core dependencies..."
    
    sudo apt-get install -y \
        ros-humble-rclcpp \
        ros-humble-rclcpp-components \
        ros-humble-std-msgs \
        ros-humble-std-srvs \
        ros-humble-geometry-msgs \
        ros-humble-sensor-msgs \
        ros-humble-nav-msgs \
        ros-humble-visualization-msgs
    
    print_info "ROS2 core dependencies installed"
}

# Function to install TF2 dependencies
install_tf2_dependencies() {
    print_step "Installing TF2 dependencies..."
    
    sudo apt-get install -y \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs
    
    print_info "TF2 dependencies installed"
}

# Function to install Gazebo and ros_gz packages
install_gazebo_dependencies() {
    print_step "Installing Gazebo and ros_gz bridge packages..."
    
    # Insta
    sudo apt-get install -y \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-interfaces \
        ros-humble-ros-gz-image
    
    print_info "Gazebo and ros_gz packages installed"
}

# Function to install Ignition Fortress and ros_ign packages
install_ignition_dependencies() {
    print_step "Installing Ignition Fortress and ros_ign bridge packages..."
    
    # Install Ignition Fortress packages
    sudo apt-get install -y \
        ignition-fortress \
        ros-humble-ros-ign \
        ros-humble-ros-ign-bridge \
        ros-humble-ros-ign-gazebo \
        ros-humble-ros-ign-image \
        ros-humble-ros-ign-interfaces
    
    print_info "Ignition Fortress and ros_ign packages installed"
}

# Function to install grid_map packages
install_grid_map_packages() {
    print_step "Installing grid_map packages..."
    
    sudo apt-get install -y \
        ros-humble-grid-map \
        ros-humble-grid-map-ros \
        ros-humble-grid-map-msgs
    
    print_info "grid_map packages installed"
}

# Function to install rosidl generators
install_rosidl_generators() {
    print_step "Installing rosidl interface generators..."
    
    sudo apt-get install -y \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime
    
    print_info "rosidl generators installed"
}

# Function to install testing dependencies
install_testing_dependencies() {
    print_step "Installing testing dependencies..."
    
    sudo apt-get install -y \
        ros-humble-ament-cmake-gtest \
        ros-humble-ament-lint-auto \
        ros-humble-ament-lint-common \
        ros-humble-launch-testing \
        ros-humble-launch-testing-ros \
        python3-pytest
    
    print_info "Testing dependencies installed"
}

# Function to install additional useful tools
install_additional_tools() {
    print_step "Installing additional useful tools..."
    
    sudo apt-get install -y \
        ros-humble-rviz2 \
        ros-humble-rqt \
        ros-humble-rqt-common-plugins \
    
    print_info "Additional tools installed"
}

# Function to check for rs1_environment package
check_rs1_environment() {
    print_step "Checking for rs1_environment package..."
    
    print_warning "The package depends on 'rs1_environment' which needs to be available."
    print_warning "Please ensure rs1_environment is either:"
    print_warning "  1. Already installed in your workspace, or"
    print_warning "  2. Available in the same workspace source directory"
    
    read -p "Press Enter to continue..."
}

# Function to verify installation
verify_installation() {
    print_step "Verifying installation..."
    
    source /opt/ros/humble/setup.bash
    
    # Check if key packages are available
    local packages_to_check=(
        "rclcpp"
        "ros_gz_sim"
        "grid_map_ros"
        "robot_state_publisher"
    )
    
    local all_found=true
    for pkg in "${packages_to_check[@]}"; do
        if ros2 pkg prefix "$pkg" &> /dev/null; then
            print_info "✓ Package found: $pkg"
        else
            print_warning "✗ Package not found: $pkg"
            all_found=false
        fi
    done
    
    if [ "$all_found" = true ]; then
        print_info "✓ All key packages verified successfully"
    else
        print_warning "Some packages were not found. You may need to source your workspace."
    fi
}


# Main installation flow
main() {
    echo ""
    echo "================================================================================"
    echo "  RS1 ROBOT PACKAGE - DEPENDENCY INSTALLATION SCRIPT"
    echo "  Ubuntu 22.04 | ROS2 Humble"
    echo "================================================================================"
    echo ""
    
    # Check if running with sudo (we'll use sudo when needed)
    if [ "$EUID" -eq 0 ]; then 
        print_warning "Please run this script as a normal user (not with sudo)"
        print_warning "The script will ask for sudo password when needed"
        exit 1
    fi
    
    # Confirmation
    read -p "This script will install ROS2 dependencies. Continue? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Installation cancelled"
        exit 0
    fi
    
    # Run installation steps
    check_ros2_installation
    update_package_lists
    install_system_dependencies
    initialize_rosdep
    install_ros2_core_dependencies
    install_tf2_dependencies
    install_gazebo_dependencies
    install_ignition_dependencies
    install_grid_map_packages
    install_rosidl_generators
    install_testing_dependencies
    install_additional_tools
    check_rs1_environment
    verify_installation
    
    echo ""
    echo "================================================================================"
    print_info "Installation completed successfully!"
    echo "================================================================================"
    echo ""
    print_info "Next steps:"
    print_info "  1. Set up workspace rs1_ws and build the package"
    print_info "  1. Source workspace: source ~/rs1_ws/install/setup.bash"
    echo ""
}

# Run main function
main "$@"
