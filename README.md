# GhostPilot

**Open-source visual SLAM + agentic AI navigation stack for GPS-denied drone flight.**

> "Fly to the third floor, check each room for occupants, land at the helipad." — Done.

GhostPilot lets any drone fly indoors, in jammed environments, or contested airspace without GPS. Built on proven robotics standards (ROS2, Nav2), with a natural language agentic layer for mission control.

## The Problem

Drones are GPS-dependent. GPS is fragile:
- **Jamming**: Russia jammed 85% of drones in some Ukraine sectors
- **Urban canyons**: Signals bounce, accuracy drops to meters
- **Indoors**: GPS simply doesn't work
- **Forests**: Canopy disrupts signals

Current solutions are $50K+ military systems or unmaintained academic code. GhostPilot is the **open-source answer**.

## Key Features

- **Visual-Inertial SLAM**: Camera + IMU fusion for 6DOF pose estimation (VINS-Mono integration planned)
- **Agentic Mission Planner**: Natural language commands → executable navigation goals
- **Nav2 Integration**: Industry-standard path planning + obstacle avoidance
- **Edge-Native**: Runs on Jetson Orin / Raspberry Pi 5, no cloud dependency
- **ROS2 Native**: Full integration with the robotics ecosystem

## Current Status

**Early Development (v0.1.0)**

| Component | Status | Notes |
|-----------|--------|-------|
| Nav2 Integration | ✅ Working | Path planning + obstacle avoidance |
| Agentic AI Parser | ✅ Working | LLM-based natural language → goals |
| Mission Executor | ✅ Working | Nav2 action client with async completion |
| VINS-Mono SLAM | ⚠️ Framework Only | Requires VINS-Mono library integration |
| Gazebo Simulation | ⚠️ Basic | Indoor warehouse world |
| Hardware Testing | ❌ Not Yet | Need RealSense + PX4 setup |

**VINS-Mono Integration**: The `slam_node.py` provides the ROS2 wrapper framework, but the actual VINS-Mono library integration is NOT complete. You need to:
1. Build VINS-Mono from source: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
2. Link it to the slam_node, OR
3. Use an alternative SLAM (ORB-SLAM3, LIO-SAM, etc.)

## Quick Start

```bash
# Clone the repo
git clone https://github.com/AmSach/GhostPilot.git
cd GhostPilot

# Install dependencies (Ubuntu 22.04 + ROS2 Humble)
./scripts/setup_jetson.sh

# Run simulation (requires ROS2 Humble + Gazebo)
ros2 launch ghostpilot_gazebo indoor_warehouse.launch.py

# In another terminal, run the agentic planner
ros2 run ghostpilot_agent mission_parser
```

## Architecture

```
┌─────────────────────────────────────────────┐
│  Agentic Mission Planner (LLM-based)         │
│  "Inspect building B, report damage"        │
├─────────────────────────────────────────────┤
│  Visual-Inertial SLAM (VINS-Mono)           │
│  Camera + IMU → 6DOF pose                   │
│  ⚠️ Requires VINS-Mono integration          │
├─────────────────────────────────────────────┤
│  Nav2 Navigation Stack                      │
│  Path planning + obstacle avoidance         │
├─────────────────────────────────────────────┤
│  Edge Runtime (Jetson Orin / Pi 5)          │
└─────────────────────────────────────────────┘
```

## Packages

| Package | Description | Status |
|---------|-------------|--------|
| `ghostpilot_core` | SLAM + Nav2 bridge | Framework ready, needs VINS-Mono |
| `ghostpilot_agent` | LLM mission parser + executor | ✅ Working |
| `ghostpilot_gazebo` | Gazebo simulation | ⚠️ Basic |

## Mission Command Examples

```
"Fly to the third floor, check each room for occupants"
"Navigate around the blocked corridor, resume path at waypoint B"
"Inspect the roof, avoid personnel, land at helipad"
"Follow the pipeline east for 200m, report anomalies"
```

## Hardware Requirements

- **Compute**: NVIDIA Jetson Orin AGX or Raspberry Pi 5
- **Camera**: Intel RealSense D435i (or equivalent stereo+IMU)
- **Flight Controller**: PX4 or similar (MavLink compatible)
- **Frame**: Any MAVLink-capable quadcopter

## Documentation

- [Architecture](docs/architecture.md)
- [Why GPS-Denied Matters](docs/gps-denied-explained.md)
- [Demo Guide](docs/demo-guide.md)

## Roadmap

- [ ] Complete VINS-Mono integration
- [ ] Add ORB-SLAM3 as alternative SLAM backend
- [ ] Multi-drone coordination
- [ ] Real hardware testing with RealSense + PX4
- [ ] Simulation environments for common scenarios

## Contributing

Pull requests welcome. Priority areas:
1. VINS-Mono / ORB-SLAM3 integration
2. Hardware testing and calibration guides
3. Simulation scenarios

## License

Apache 2.0