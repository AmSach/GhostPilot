#!/bin/bash
# GhostPilot Jetson Orin / Raspberry Pi 5 Setup Script

set -e

echo "GhostPilot Setup for Edge Devices"
echo "=================================="

# Detect platform
if [ -f /etc/nv_tegra_release ]; then
    PLATFORM="jetson"
    echo "Detected: NVIDIA Jetson (Orin)"
elif [ "$(uname -m)" = "aarch64" ]; then
    PLATFORM="pi5"
    echo "Detected: Raspberry Pi 5 (64-bit)"
else
    PLATFORM="generic"
    echo "Detected: Generic x86_64"
fi

# Install ROS2 Humble if not present
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS2 Humble..."
    sudo apt update
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
    wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install -y ros-humble-ros-base
    sudo apt install -y python3-colcon-common-extensions python3-rosdep
else
    echo "ROS2 already installed"
fi

# Install Nav2
echo "Installing Nav2..."
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# Install VINS-Mono dependencies
echo "Installing VINS-Mono dependencies..."
sudo apt install -y libopencv-dev libeigen3-dev libcxsparse-dev libspdlog-dev

# Install realsense SDK if Jetson
if [ "$PLATFORM" = "jetson" ]; then
    echo "Installing RealSense SDK for Jetson..."
    sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
fi

# Create workspace
mkdir -p ~/ghostpilot_ws/src
cd ~/ghostpilot_ws
colcon build

# Clone VINS-Mono (placeholder - actual repo)
# git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git

echo ""
echo "Setup complete!"
echo "To build GhostPilot packages:"
echo "  cd ~/ghostpilot_ws && colcon build --packages-select ghostpilot_core ghostpilot_agent"
echo ""
echo "To launch:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 launch ghostpilot_core bringup.launch.py"