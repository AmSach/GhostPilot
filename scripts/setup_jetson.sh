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
    sudo apt install -y software-properties-common curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-ros-base
    sudo apt install -y python3-colcon-common-extensions python3-rosdep
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
else
    echo "ROS2 already installed"
fi

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true

# Install Nav2
echo "Installing Nav2..."
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# Install core dependencies
echo "Installing core dependencies..."
sudo apt install -y libopencv-dev libeigen3-dev libcxsparse-dev libspdlog-dev

# Install RealSense SDK for Jetson
if [ "$PLATFORM" = "jetson" ]; then
    echo "Installing RealSense SDK for Jetson..."
    sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description || {
        echo "NOTE: RealSense packages may need manual installation from Intel's repo"
        echo "See: https://github.com/IntelRealSense/realsense-ros"
    }
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install numpy opencv-python pyyaml requests scipy --break-system-packages 2>/dev/null || \
pip3 install numpy opencv-python pyyaml requests scipy

# Install Ollama for local LLM (optional but recommended)
if ! command -v ollama &> /dev/null; then
    echo ""
    echo "NOTE: Ollama not installed. For agentic AI features, install with:"
    echo "  curl -fsSL https://ollama.com/install.sh | sh"
    echo "  ollama pull llama3"
fi

# Create workspace
WORKSPACE="$HOME/ghostpilot_ws"
mkdir -p "$WORKSPACE/src"

# Clone GhostPilot
if [ ! -d "$WORKSPACE/src/GhostPilot" ]; then
    echo "Cloning GhostPilot..."
    git clone https://github.com/AmSach/GhostPilot.git "$WORKSPACE/src/GhostPilot" || {
        echo "NOTE: If repo doesn't exist yet, copy your local GhostPilot to $WORKSPACE/src/"
    }
fi

# VINS-Mono setup (NOT YET INTEGRATED - HONEST STATUS)
echo ""
echo "=========================================="
echo "IMPORTANT: VINS-Mono Integration Status"
echo "=========================================="
echo "VINS-Mono is NOT yet integrated into GhostPilot."
echo ""
echo "Current status:"
echo "  - SLAM node provides framework, but requires VINS-Mono library"
echo "  - VINS-Mono repo: https://github.com/HKUST-Aerial-Robotics/VINS-Mono"
echo ""
echo "To integrate VINS-Mono manually:"
echo "  1. git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git"
echo "  2. Follow VINS-Mono build instructions"
echo "  3. Update slam_node.py to call VINS-Mono API"
echo ""
echo "Alternatively, use ORB-SLAM3 or loop back to external SLAM."
echo "=========================================="
echo ""

# Build GhostPilot packages
cd "$WORKSPACE"
if [ -d "src/GhostPilot" ]; then
    echo "Building GhostPilot packages..."
    colcon build --packages-select ghostpilot_core ghostpilot_agent ghostpilot_gazebo || {
        echo "Build failed - check ROS2 setup and dependencies"
    }
else
    echo "GhostPilot source not found in $WORKSPACE/src/"
    echo "Copy or clone the repo to that location"
fi

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "To use GhostPilot:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WORKSPACE/install/setup.bash"
echo "  ros2 launch ghostpilot_core bringup.launch.py"
echo ""
echo "For agentic AI features, first run:"
echo "  ollama pull llama3"
echo ""
echo "Hardware requirements:"
echo "  - Intel RealSense D435i (or similar stereo+IMU camera)"
echo "  - PX4 or ArduPilot flight controller (MAVLink)"
echo ""