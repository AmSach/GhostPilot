# GhostPilot: GPS-Denied Drone Navigation with Visual SLAM and Agentic AI

**By [Aman Sachan](https://linkedin.com/in/theamansach)** | [GitHub](https://github.com/AmSach) | [Instagram](https://instagram.com/i.amsach)

---

## The Problem: GPS is Fragile

Drones depend on GPS. But GPS fails in:
- **Indoors** — No signal
- **Urban canyons** — 10-50m error
- **Forests** — Canopy blocks signal
- **Contested airspace** — Jammed/spoofed (Russia jammed 85% of drones in Ukraine)

Current GPS-denied solutions cost **$50,000+**. GhostPilot is the **open-source answer**.

**GitHub**: [github.com/AmSach/GhostPilot](https://github.com/AmSach/GhostPilot)

---

## What GhostPilot Does

GhostPilot is a drone navigation stack that works **without GPS**:

1. **Visual-Inertial SLAM** — Camera + IMU fusion for 6DOF pose
2. **Agentic Mission Parser** — Natural language → flight goals
3. **Nav2 Integration** — Industry-standard path planning

**Example command**: *"Fly to the 2nd floor, inspect area, avoid personnel, report anomaly"*

The parser converts this to structured goals, SLAM estimates pose from camera/IMU, and Nav2 executes navigation.

---

## Architecture

```
┌─────────────────────────────────────────────┐
│  Agentic Mission Planner (LLM-based)         │
│  "Fly to third floor, inspect rooms"         │
├─────────────────────────────────────────────┤
│  Visual-Inertial SLAM (VINS-Mono)            │
│  Camera + IMU → 6DOF pose                    │
├─────────────────────────────────────────────┤
│  Nav2 Navigation Stack                       │
│  Path planning + Obstacle avoidance          │
└─────────────────────────────────────────────┘
```

---

## Key Features

| Feature | Description |
|---------|-------------|
| **Mission Parser** | Natural language → JSON goals (LLM + regex fallback) |
| **VINS-Mono Pipeline** | Feature tracking, IMU pre-integration, sliding window optimization |
| **Pose Bridge** | SLAM → Nav2 translation with jump rejection (safety!) |
| **Headless Simulation** | Test without ROS2 or hardware |

---

## Quick Start

```bash
git clone https://github.com/AmSach/GhostPilot.git
cd GhostPilot
pip install -r requirements.txt
python3 simulate.py
```

**63 tests pass, 2 skipped** (ROS2-only tests).

---

## Production Status

| Component | Status |
|-----------|--------|
| Mission Parser | ✅ Working |
| Mission Executor | ✅ Working |
| SLAM Pipeline | ✅ Tested |
| Pose Bridge | ✅ Working |
| Real Hardware | ⚠️ Needs field testing |

**Not flight-certified yet** — but a solid foundation for development and simulation.

---

## About the Author

**Aman Sachan** builds open-source robotics and AI systems.

- **LinkedIn**: [linkedin.com/in/theamansach](https://linkedin.com/in/theamansach)
- **GitHub**: [github.com/AmSach](https://github.com/AmSach)
- **Instagram**: [instagram.com/i.amsach](https://instagram.com/i.amsach)

---

## Links

- **GitHub Repo**: [github.com/AmSach/GhostPilot](https://github.com/AmSach/GhostPilot)
- **Research Paper**: See `research-paper.md` in repo
- **Technical Handbook**: See `technical-paper.md` in repo

Star the repo if you found this useful!

---

**Keywords**: GPS-denied navigation, visual SLAM, VINS-Mono, ROS2, Nav2, drone autonomy, agentic AI, indoor drone, UAV navigation, open source robotics
