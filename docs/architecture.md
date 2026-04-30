# GhostPilot Architecture

## Overview

GhostPilot is an open-source GPS-denied drone navigation stack combining:
- Visual-inertial SLAM for pose estimation
- ROS2 Nav2 for navigation
- Agentic AI for natural language mission commands

## System Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                      GhostPilot System                           │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌───────────────┐    ┌──────────────────┐ │
│  │  Realsense   │───▶│  VINS-Mono    │───▶│    Pose Bridge   │ │
│  │  Camera +    │    │  Visual SLAM  │    │  (SLAM → Nav2)   │ │
│  │  IMU         │    │               │    │                  │ │
│  └──────────────┘    └───────────────┘    └────────┬─────────┘ │
│                                                     │           │
│                    ┌────────────────────────────────▼───────┐   │
│                    │           Nav2 Navigation Stack          │   │
│                    │  ┌─────────┐ ┌──────────┐ ┌──────────┐  │   │
│                    │  │ AMCL    │ │ Planner  │ │ Controller│  │   │
│                    │  │         │ │ Server   │ │ Server   │  │   │
│                    │  └─────────┘ └──────────┘ └──────────┘  │   │
│                    └──────────────────────────────────────────┘   │
│                                    │                              │
│  ┌──────────────────┐    ┌─────────▼─────────┐                    │
│  │  Agentic AI      │───▶│   Mission         │                    │
│  │  (LLM Planner)   │    │   Executor        │                    │
│  └──────────────────┘    └───────────────────┘                    │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

## Components

### 1. ghostpilot_core

Main navigation stack package.

**Nodes:**
- `slam_node` — VINS-Mono wrapper, publishes `/slam/pose`
- `pose_bridge` — Converts SLAM pose to Nav2 format

**Launch files:**
- `bringup.launch.py` — Starts SLAM + Nav2 + pose bridge

### 2. ghostpilot_agent

Agentic AI layer for mission planning.

**Nodes:**
- `mission_parser` — Parses NL commands to goals
- `executor` — Sends goals to Nav2

### 3. ghostpilot_gazebo

Gazebo simulation for testing.

**Worlds:**
- `indoor_warehouse.world` — Indoor GPS-denied test environment

## Data Flow

1. **Camera + IMU** → VINS-Mono → 6DOF pose estimate
2. **Pose** → Pose Bridge → Nav2 AMCL
3. **Mission command** → Agent parser → Goal list
4. **Goals** → Executor → Nav2 action server
5. **Nav2** → Velocity commands → Drone

## Topic Map

| Topic | Type | Direction | Purpose |
|-------|------|----------|---------|
| `/camera/image_raw` | Image | in | Camera input |
| `/imu/data` | Imu | in | IMU input |
| `/slam/pose` | PoseStamped | out | SLAM pose |
| `/goal_pose` | PoseStamped | out | Nav2 goal |
| `/cmd_vel` | Twist | out | Motor commands |

## Configuration

### VINS Parameters (`vins_params.yaml`)
- Camera-IMU extrinsic calibration
- Feature tracking thresholds
- Loop closure settings

### Nav2 Parameters (`nav2_params.yaml`)
- AMCL configuration
- Controller gains
- Costmap inflation
- Behavior tree

## Hardware Support

| Platform | Status | Notes |
|----------|--------|-------|
| Jetson Orin | ✅ Tested | 30+ FPS |
| Jetson Nano | ✅ Tested | 15 FPS |
| Raspberry Pi 5 | ⚠️ WIP | Needs optimization |
| x86_64 | ✅ Tested | For simulation |