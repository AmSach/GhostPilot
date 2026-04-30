# GhostPilot Demo Guide

## Demo 1: Indoor Warehouse Inspection

### Scenario
Autonomous inspection of a warehouse with GPS-denied environment.

### Setup
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch ghostpilot_gazebo indoor_warehouse.launch.py

# Terminal 2: Launch core navigation
ros2 launch ghostpilot_core bringup.launch.py

# Terminal 3: Launch agent
ros2 run ghostpilot_agent mission_parser_node
```

### Run Mission
```bash
# Send mission command
ros2 topic pub /ghostpilot/mission std_msgs/msg/String \
  "{data: 'Fly to the northeast corner, inspect each aisle, return to start'}"
```

### Expected Behavior
1. Agent parses mission into waypoints
2. SLAM builds map of warehouse
3. Nav2 navigates to each waypoint
4. Agent detects and reports obstacles
5. Drone returns to launch position

## Demo 2: GPS Jamming Resilience

### Scenario
Compare standard GPS-dependent flight vs GhostPilot during simulated jamming.

### Setup
```bash
# Launch with GPS-denied environment
ros2 launch ghostpilot_gazebo jammed_environment.launch.py
```

### Run Comparison
```bash
# Standard drone (should drift/lose position)
ros2 topic pub /drone/cmd std_msgs/msg/String "{data: 'hover'}"

# GhostPilot (should maintain position using SLAM)
ros2 topic pub /ghostpilot/cmd std_msgs/msg/String "{data: 'hover'}"
```

### Expected Behavior
- Standard drone: Position error accumulates, eventually drifts off course
- GhostPilot: Maintains position using visual odometry

## Demo 3: Natural Language Mission Control

### Scenario
Complex multi-step mission via natural language.

### Command
```
ros2 topic pub /ghostpilot/mission std_msgs/msg/String \
  "{data: 'Inspect the third floor rooms, avoid any personnel, report damage to infrastructure'}"
```

### Expected Parsed Goals
```
1. NavigateToFloor(floor=3)
2. InspectArea(rooms=all, avoid=personnel)
3. ReportDamage(infrastructure=true)
```

## Demo 4: Gazebo World Walkthrough

### Worlds Available
- `indoor_warehouse.world` — Shelving, boxes, narrow aisles
- `office_building.world` — Multiple floors, rooms, furniture
- `disaster_site.world` — Rubble, debris, partial structures

### Change World
```bash
export GAZEBO_WORLD=office_building
ros2 launch ghostpilot_gazebo indoor_warehouse.launch.py
```

## Simulation Troubleshooting

### Gazebo crashes on startup
```bash
# Kill stray processes
pkill -9 gzserver; pkill -9 gzclient
# Re-launch
ros2 launch ghostpilot_gazebo indoor_warehouse.launch.py
```

### Drone falls through floor
- Check Gazebo physics timestep matches ROS2 timestep
- Verify drone URDF has correct inertial parameters

### SLAM not initializing
- Ensure camera feed is publishing
- Check IMU data is available: `ros2 topic echo /imu/data`
- Increase lighting in simulation world

## Hardware Demo Setup

### Required Hardware
- Jetson Orin AGX (or Pi 5)
- RealSense D435i camera
- PX4 flight controller
- MAVLink-capable quadcopter frame

### Calibration Steps
```bash
# Camera-IMU calibration
./scripts/calibrate_camera.sh

# Verify extrinsic parameters
ros2 param get /ghostpilot/slam_node extrinsics

# Test SLAM in static position
ros2 launch ghostpilot_core bringup.launch.py
rviz2
# Add TF display, verify camera->IMU transform
```

### Flight Test Sequence
1. **Hover test**: 1m altitude hover for 30 seconds
2. **Waypoint test**: Fly to 4 cardinal directions, return
3. **Obstacle test**: Fly toward obstacle, verify avoidance
4. **Mission test**: Full natural language command execution

## Performance Benchmarks

### Simulation Targets
- SLAM pose latency: < 50ms
- Nav2 planning time: < 200ms
- Total loop time: < 100ms (10 Hz control)

### Hardware Targets
- On Jetson Orin: 30 FPS SLAM
- On Raspberry Pi 5: 15 FPS SLAM
- Power consumption: < 30W peak

## Monitoring Tools

```bash
# Watch pose accuracy
ros2 topic echo /ghostpilot/pose -n 1

# Monitor SLAM health
ros2 topic echo /ghostpilot/health

# Plot trajectory
ros2 run rqt_plot rqt_plot /ghostpilot/pose/position/...

# View costmap
ros2 run rviz2 rviz2
# Add ObstacleLayer, verify obstacle inflation
```