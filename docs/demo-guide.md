# GhostPilot Demo Guide

## Demo 1: Indoor Warehouse Navigation

### Setup

```bash
# Launch Gazebo simulation
ros2 launch ghostpilot_gazebo warehouse_world.launch.py

# In another terminal, launch navigation
ros2 launch ghostpilot_core bringup.launch.py

# Launch agentic planner
ros2 run ghostpilot_agent mission_executor
```

### Run the Demo

```bash
# Send mission command
ros2 topic pub /mission_command std_msgs/msg/String "data: 'Navigate to storage zone A, scan for obstacles, return home'"
```

Expected behavior:
1. Drone takes off (if not already flying)
2. Nav2 navigates to zone A waypoint
3. Obstacles detected via costmap
4. Re-planning around obstacles
5. Return to launch point
6. Land and generate mission report

## Demo 2: GPS-Denied Comparison

### Setup

```bash
# Standard GPS mode (simulated)
ros2 launch ghostpilot_gazebo gps_mode.launch.py

# GPS-denied mode
ros2 launch ghostpilot_gazebo gps_denied_mode.launch.py
```

### Run Comparison

1. **GPS Mode**: Fly through waypoints → note ~2m accuracy
2. **GPS-Denied Mode**: Same trajectory → GhostPilot maintains accuracy via VINS

## Demo 3: Natural Language Mission

### Interactive Mode

```bash
ros2 run ghostpilot_agent interactive_mission
```

At the prompt:
```
> Inspect the east wing, avoid the central pillar, report any people detected
[Agent] Parsing mission...
[Agent] Generated 5 goals:
  1. Navigate to east_wing_entrance
  2. Scan room_1 (occupancy check)
  3. Navigate around central_pillar
  4. Scan room_2 (occupancy check)
  5. Return to home and land
[Agent] Executing goal 1...
```

## Troubleshooting

### SLAM Initialization Fails
- Ensure camera has sufficient texture (not a blank wall)
- Check IMU data is publishing: `ros2 topic echo /imu/data`
- Increase `max_cnt` in `vins_params.yaml`

### Navigation Stalls
- Check costmap has obstacles: `ros2 topic echo /costmap`
- Verify planner can find path: `ros2 topic echo /planned_path`
- Try recovery: `ros2 service call /recover nav2_srv/SimpleCharge`

### Gazebo Crashes
- Ensure sufficient RAM (8GB+ recommended)
- Reduce simulation speed: edit `gz_params.yaml` set `max_sim_time=0.5`

## Performance Metrics

Track these to evaluate GhostPilot:

| Metric | Target | How to Measure |
|--------|--------|----------------|
| SLAM FPS | 30+ | `ros2 topic hz /odometry/filtered` |
| Pose accuracy | <10cm | Compare to OptiTrack if available |
| Navigation success | >90% | Count successful missions |
| Battery drain | Track per-mission | Monitor `/battery_state` |

## Next Steps After Demo

1. Calibrate your camera-IMU: `bash scripts/calibrate_camera.sh`
2. Tune VINS params for your environment (lighting, texture)
3. Customize behavior tree for your mission types
4. Integrate with your hardware platform
