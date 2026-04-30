#!/bin/bash
# GhostPilot Jetson Orin setup script

set -e

echo "Setting up GhostPilot on Jetson Orin..."

# Flash JetPack if needed
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS2 Humble..."
    sudo apt update
    sudo apt install -y curl gnupg lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
    sudo apt update
    sudo apt install -y ros-humble-desktop
fi

# Install VINS-Mono dependencies
echo "Installing VINS-Mono dependencies..."
sudo apt install -y \
    libopencv-dev \
    libeigen3-dev \
    libcxsparse-dev \
    libsuitesparse-dev \
    libv4l-dev \
    librealsense2-dev

# Install Nav2
echo "Installing Nav2..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-tf2-geometry-msgs

# Install GhostPilot
echo "Installing GhostPilot..."
cd "$(dirname "$0")/.."
pip3 install -e .

# Configure camera
echo "Configuring Realsense camera..."
sudo mkdir -p /etc/udev/rules.d/
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="8087", MODE="0666"' | sudo tee /etc/udev/rules.d/99-realsense.rules

echo "GhostPilot setup complete!"
echo "Run: ros2 launch ghostpilot_core bringup.launch.py"