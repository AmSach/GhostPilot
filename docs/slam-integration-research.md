# GhostPilot SLAM Integration Research

## Options for GPS-Denied Navigation

### 1. VINS-Mono (ROS2)
**GitHub**: https://github.com/dongbo19/VINS-MONO-ROS2  
**Alternative**: https://github.com/labust/ROS2-VINS-Mono-GTSAM

**Pros**:
- Designed specifically for drones
- Monocular + IMU (lighter weight)
- Good for indoor environments

**Cons**:
- ROS2 version less mature than ROS1
- Requires careful camera-IMU calibration

### 2. ORB-SLAM3 (ROS2 Humble)
**GitHub**: https://github.com/Mechazo11/ros2_orb_slam3  
**Tutorial**: https://medium.com/@akbedaka/a-complete-guide-to-installing-orb-slam3-with-ros2-humble-213d691c67e4

**Pros**:
- Full ROS2 Humble support
- Works with monocular, stereo, RGB-D
- Active community

**Cons**:
- Heavier compute requirements
- Not specifically designed for drones

### 3. OpenVINS
**Docs**: https://docs.openvins.com/

**Pros**:
- Open-source, well-documented
- Filter-based (lower latency than optimization)
- Supports multiple camera configurations

**Cons**:
- Less drone-specific than VINS

## Recommended Path

For GhostPilot, **VINS-Mono ROS2** is the best choice because:
1. Designed for drones (state estimation + feedback control)
2. Monocular + IMU = lighter payload
3. Works well in indoor/GPS-denied environments

## Installation Steps (ROS2 Humble)

```bash
# Install dependencies
sudo apt install -y \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools

# Clone VINS-Mono ROS2
cd ~/ros2_ws/src
git clone https://github.com/dongbo19/VINS-MONO-ROS2.git
git clone https://github.com/dongbo19/camera_models.git

# Build
cd ~/ros2_ws
colcon build --symlink-install
```

## Hardware Requirements

- **Camera**: Intel RealSense D435i (stereo + IMU)
- **Compute**: NVIDIA Jetson Orin or Raspberry Pi 5
- **Flight Controller**: PX4 (MAVLink compatible)

## Next Steps

1. Test VINS-Mono ROS2 in simulation (Gazebo)
2. Integrate with existing slam_node.py wrapper
3. Test with RealSense D435i on actual drone

## Experts to Contact

- **HKUST Aerial Robotics Group** - Original VINS-Mono authors
  - Website: https://ri.hkust.edu.hk/vins-mono
  - Email: Check publications for contact info

- **ROS2 Discourse** - Community forums for integration help
  - https://discourse.ros.org/

- **GitHub Issues** - Report bugs on respective repos
