# GhostPilot

**Open-source visual SLAM + agentic AI navigation stack for GPS-denied drone flight.**

> "Fly to the third floor, check each room for occupants, land at the helipad." — Done.

GhostPilot lets any drone fly indoors, in jammed environments, or contested airspace without GPS. Built on proven robotics standards (ROS2, Nav2, VINS-Mono), with a natural language agentic layer for mission control.

## The Problem

Drones are GPS-dependent. GPS is fragile:
- Jamming: Russia jammed 85% of drones in some Ukraine sectors
- Urban canyons: Signals bounce, accuracy drops to meters
- Indoors: GPS simply doesn't work
- Forests: Canopy disrupts signals

Current solutions are $50K+ military systems or unmaintained academic code. GhostPilot is the **open-source answer**.

## Key Features

- **Visual-Inertial SLAM**: Camera + IMU fusion for 6DOF pose estimation
- **Agentic Mission Planner**: Natural language commands → executable navigation goals
- **Nav2 Integration**: Industry-standard path planning + obstacle avoidance
- **Edge-Native**: Runs on Jetson Orin / Raspberry Pi 5, no cloud dependency
- **ROS2 Native**: Full integration with the robotics ecosystem

## Quick Start

```bash
# Clone the repo
git clone https://github.com/amsach/GhostPilot.git
cd GhostPilot

# Install dependencies (Ubuntu 22.04 + ROS2 Humble)
./scripts/setup_jetson.sh

# Run simulation
ros2 launch ghostpilot_gazebo indoor_warehouse.launch.py

# In another terminal, run the agentic planner
ros2 run ghostpilot_agent mission_parser_node
```

## Architecture

```
┌─────────────────────────────────────────────┐
│  Agentic Mission Planner (LLM-based)         │
│  "Inspect building B, report damage"        │
├─────────────────────────────────────────────┤
│  Visual-Inertial SLAM (VINS-Mono)           │
│  Camera + IMU → 6DOF pose                   │
├─────────────────────────────────────────────┤
│  Nav2 Navigation Stack                      │
│  Path planning + obstacle avoidance         │
├─────────────────────────────────────────────┤
│  Edge Runtime (Jetson Orin / Pi 5)          │
└─────────────────────────────────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `ghostpilot_core` | VINS-Mono SLAM + Nav2 integration |
| `ghostpilot_agent` | LLM-based mission parser + executor |
| `ghostpilot_gazebo` | Gazebo simulation world + models |

## Mission Command Examples

```
"Fly to the third floor, check each room for occupants"
"Navigate around the blocked corridor, resume path at waypoint B"
"Inspect the roof, avoid personnel, land at helipad"
"Follow the pipeline east for 200m, report anomalies"
```

## Hardware Requirements

- **Compute**: NVIDIA Jetson Orin AGX or Raspberry Pi 5
- **Camera**: Intel RealSense D435i (or equivalent stereo/IMU)
- **Flight Controller**: PX4 or similar (MavLink compatible)
- **Frame**: Any MAVLink-capable quadcopter

## Documentation

- [Architecture](docs/architecture.md)
- [Why GPS-Denied Matters](docs/gps-denied-explained.md)
- [Demo Guide](docs/demo-guide.md)

## Comparison

| Feature | GhostPilot | Skydio | Military Systems |
|---------|-----------|--------|-----------------|
| Cost | $0 (open-source) | $5K+ | $50K+ |
| GPS-denied | ✅ Native | ⚠️ Limited | ✅ Yes |
| Agentic AI | ✅ Natural language | ❌ Waypoints | ❌ Pre-programmed |
| Edge-only | ✅ No cloud | ❌ Cloud | ❌ Proprietary |
| ROS2-native | ✅ Full | ❌ Closed | ❌ Proprietary |

## Status

Early development. Core SLAM + Nav2 bridge working in simulation. Agentic layer in progress.

## Contributing

Pull requests welcome. See issues for TODO list.

## License

Apache 2.0