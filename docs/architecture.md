# GhostPilot System Architecture

## Overview

GhostPilot is a GPS-denied drone navigation stack combining visual-inertial SLAM, ROS2 Nav2, and an agentic LLM layer for natural language mission control.

## System Diagram

```
┌──────────────────────────────────────────────────────────┐
│                   Operator Interface                      │
│         Natural Language Mission Commands                 │
└──────────────────────────┬───────────────────────────────┘
                           │
┌──────────────────────────▼───────────────────────────────┐
│               ghostpilot_agent                            │
│  ┌─────────────────┐    ┌────────────────┐              │
│  │  Mission Parser │───▶│   Executor     │              │
│  │  (LLM-based)    │    │ (Behavior Tree)│              │
│  └─────────────────┘    └───────┬────────┘              │
└─────────────────────────────────┼────────────────────────┘
                                  │ Action Goals
┌─────────────────────────────────▼────────────────────────┐
│                   ghostpilot_core                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │              Nav2 Navigation Stack                │   │
│  │  ┌──────────┐  ┌──────────┐  ┌────────────────┐  │   │
│  │  │ Planner  │──│Tracker  │──│ Controller    │  │   │
│  │  └──────────┘  └──────────┘  └────────────────┘  │   │
│  └────────────────────────┬─────────────────────────┘   │
│  ┌────────────────────────▼─────────────────────────┐   │
│  │              Pose Bridge (SLAM → Nav2)            │   │
│  └────────────────────────┬─────────────────────────┘   │
│  ┌────────────────────────▼─────────────────────────┐   │
│  │           Visual-Inertial SLAM                    │   │
│  │     Camera + IMU → 6DOF Pose Estimation           │   │
│  │              (VINS-Mono / ORB-SLAM3)              │   │
│  └──────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
                                  │ Pose / Odometry
┌─────────────────────────────────▼────────────────────────┐
│                    Hardware Layer                         │
│  ┌──────────┐   ┌───────────┐   ┌─────────────────┐      │
│  │ Camera   │   │   IMU     │   │ Flight Ctrl     │      │
│  │ RealSense│   │  (built-in)│   │ (MAVLink/PX4)  │      │
│  └──────────┘   └───────────┘   └─────────────────┘      │
└──────────────────────────────────────────────────────────┘
```

## Package Descriptions

### ghostpilot_core

Core navigation stack wrapping VINS-Mono and bridging to Nav2.

| Node | Description |
|------|-------------|
| `slam_node` | VINS-Mono wrapper, publishes `/pose` (geometry_msgs/PoseStamped) |
| `pose_bridge` | Converts SLAM pose to Nav2 localization format |

**Topics:**
- `/camera/image_raw` — Input camera feed
- `/imu/data` — Input IMU data
- `/pose` — SLAM pose output
- `/goal_pose` — Nav2 goal target

### ghostpilot_agent

LLM-based mission decomposition and execution.

| Node | Description |
|------|-------------|
| `mission_parser` | Parses natural language → structured goals |
| `executor` | Behavior tree executing parsed goals |

**Actions:**
- `NavigateToPose` — Move to a 3D waypoint
- `InspectArea` — Systematic area scan
- `AvoidObstacle` — Reactive obstacle avoidance
- `LandAtPosition` — Controlled landing

### ghostpilot_gazebo

Gazebo simulation for testing without hardware.

- `indoor_warehouse.world` — Simulation environment
- `iris_with_slamba` — Drone model with SLAM sensors

## Data Flow

1. **Mission Input**: "Check the third floor, report occupants"
2. **Parse**: LLM extracts goals → `[Inspect(floor=3), Report(occupants=true)]`
3. **Plan**: Executor builds Nav2 waypoint sequence
4. **Navigate**: SLAM provides pose, Nav2 computes trajectory
5. **Feedback**: Occupant detection results streamed back to operator

## Configuration Files

| File | Purpose |
|------|---------|
| `vins_params.yaml` | VINS-Mono intrinsic/extrinsic parameters |
| `nav2_params.yaml` | Nav2 planner, controller, costmap settings |
| `agent_config.yaml` | LLM provider, model, behavior tree parameters |

## ROS2 Interface

```bash
# Subscribe to pose
ros2 topic echo /ghostpilot/pose

# Send mission
ros2 topic pub /ghostpilot/mission std_msgs/msg/String "{data: 'Fly to waypoint B'}"

# Monitor health
ros2 topic echo /ghostpilot/health
```

## Edge Deployment

GhostPilot runs on:
- **Jetson Orin AGX** (recommended, 275 AI TOPS)
- **Raspberry Pi 5** (lower performance, 8GB RAM recommended)

No cloud connectivity required. All inference is on-device.

## Safety

- Failsafe behaviors built into Nav2 recovery actions
- Geofencing via dynamic reconfiguration
- Manual override via standard MAVLink RC failover