# GhostPilot — GPS-Denied Drone Navigation with Agentic AI

**Open-source visual SLAM + agentic AI navigation stack for any drone — flies indoors, in jammed environments, or contested airspace without GPS or pilot expertise.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-v2.0-green.svg)](https://navigation.ros.org/)

## Why GhostPilot?

- **GPS is fragile** — Russia jammed 85% of drones in some Ukraine sectors
- **Indoors = GPS blind** — warehouses, forests, urban canyons all fail GPS
- **Current solutions are broken** — $50K military systems, unmaintained academic code, or fragmented toolchains

GhostPilot gives you **open-source, agentic AI navigation** that works anywhere.

## Features

- **Visual-Inertial SLAM** — Camera + IMU fusion via VINS-Mono/ORB-SLAM3 for 6DOF pose estimation
- **Agentic Mission Planner** — Natural language commands like *"Inspect building B, avoid people"* → executable navigation
- **ROS2-Native** — Full Nav2 integration with path planning, obstacle avoidance, and recovery behaviors
- **Edge Runtime** — Runs on NVIDIA Jetson Orin or Raspberry Pi 5 at 30+ FPS on-device
- **No Cloud** — 100% local inference, privacy-preserving, battlefield-ready

## Quick Start

### Prerequisites

- ROS2 Humble ([install guide](https://docs.ros.org/en/humble/Installation.html))
- Ubuntu 22.04+
- Python 3.10+

### Installation

```bash
# Clone the repository
git clone https://github.com/AmSach/GhostPilot.git
cd GhostPilot

# Install dependencies
pip install -r requirements.txt

# Build
colcon build
source install/setup.bash

# Launch bringup
ros2 launch ghostpilot_core bringup.launch.py
```

### Run a Mission

```python
from ghostpilot_agent import MissionExecutor

executor = MissionExecutor()
executor.execute("Fly to the third floor, check each room for occupants, land at the helipad")
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Agentic Mission Planner (LLM-based)                    │
│  "Inspect building B, avoid people, report damage"      │
├─────────────────────────────────────────────────────────┤
│  Visual-Inertial SLAM (VINS-Mono / ORB-SLAM3)         │
│  Camera + IMU fusion → 6DOF pose, no GPS required      │
├─────────────────────────────────────────────────────────┤
│  Edge Runtime (NVIDIA Jetson Orin / Raspberry Pi 5)    │
│  Real-time inference at 30+ FPS on-device              │
├─────────────────────────────────────────────────────────┤
│  Nav2 Integration (path planning, obstacle avoidance)   │
│  ROS2-native navigation stack with recovery behaviors   │
└─────────────────────────────────────────────────────────┘
```

## Comparison

| Feature | GhostPilot | Skydio Enterprise | Military Systems |
|---------|-----------|-------------------|-----------------|
| Cost | $0 (open-source) | $5K+ per drone | $50K+ per unit |
| GPS-denied | ✅ Native | ✅ Limited | ✅ Yes |
| Agentic AI | ✅ Natural language | ❌ Waypoints only | ❌ Pre-programmed |
| Edge-only | ✅ No cloud | ❌ Cloud required | ❌ Proprietary |
| ROS2-native | ✅ Full integration | ❌ Closed | ❌ Proprietary |

## Project Structure

```
GhostPilot/
├── README.md
├── docs/
│   ├── architecture.md
│   ├── gps-denied-explained.md
│   └── demo-guide.md
├── src/
│   ├── ghostpilot_core/       # Main navigation stack
│   ├── ghostpilot_agent/     # Agentic AI layer
│   └── ghostpilot_gazebo/    # Simulation
├── scripts/
│   ├── setup_jetson.sh
│   └── calibrate_camera.sh
└── requirements.txt
```

## Documentation

- [Architecture Overview](docs/architecture.md)
- [Why GPS-Denied Matters](docs/gps-denied-explained.md)
- [Demo Guide](docs/demo-guide.md)

## License

MIT License — free for commercial, academic, and defense use.

## Contributing

PRs welcome! See [CONTRIBUTING.md](CONTRIBUTING.md) for setup instructions.
