#!/usr/bin/env python3
"""
GhostPilot Simulation — headless end-to-end demo, no ROS2 or Gazebo needed.

Demonstrates the full pipeline:
  1. MissionParser   — natural-language → goal queue
  2. MissionExecutor — goal queue → action dispatch
  3. VINSMono SLAM   — synthetic camera + IMU → pose estimates
  4. PoseBridge      — pose validation + velocity estimation
"""

import sys, os, time
import numpy as np
import cv2

ROOT = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(ROOT, 'src', 'ghostpilot_agent', 'ghostpilot_agent'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'ghostpilot_core',  'ghostpilot_core'))

from mission_parser import MissionParser
from executor      import MissionExecutor, _OBSTACLE_INFLATION
from vins_mono     import VINSMono
from pose_bridge   import PoseBridge


# ─────────────────────────────────────────────────────────────────────────────
# Synthetic sensor helpers
# ─────────────────────────────────────────────────────────────────────────────

def _synthetic_frame(h=240, w=320, offset_x=0) -> np.ndarray:
    """Checkerboard + noise — trackable by FAST + LK."""
    img = np.zeros((h, w), dtype=np.uint8)
    sq  = 30
    for r in range(0, h, sq):
        for c in range(0, w, sq):
            if ((r // sq) + (c // sq)) % 2 == 0:
                img[r:r+sq, c:c+sq] = 210
    noise = np.random.randint(0, 25, img.shape, dtype=np.uint8)
    img   = cv2.add(img, noise)
    return np.roll(img, offset_x, axis=1)

def _imu_packet(acc=(0,0,9.81), gyro=(0,0,0)):
    return np.array(acc, dtype=float), np.array(gyro, dtype=float)


# ─────────────────────────────────────────────────────────────────────────────
# Part 1 — Mission parsing
# ─────────────────────────────────────────────────────────────────────────────

def demo_parsing():
    print('\n' + '═'*62)
    print('  PART 1 — MISSION PARSING')
    print('═'*62)

    parser = MissionParser()
    missions = [
        'Fly to the third floor and inspect the area',
        'Navigate to 5th floor, avoid personnel, land at base',
        'Inspect the area and report damage',
        'Fly to floor 2, avoid machinery, report status',
    ]

    for cmd in missions:
        result = parser._parse_with_regex(cmd)
        print(f'\n  CMD : "{cmd}"')
        for i, g in enumerate(result['goals'], 1):
            print(f'  [{i}] {g["type"]:20s}  {dict((k,v) for k,v in g.items() if k!="type")}')


# ─────────────────────────────────────────────────────────────────────────────
# Part 2 — Executor goal dispatch (headless)
# ─────────────────────────────────────────────────────────────────────────────

def demo_executor():
    print('\n' + '═'*62)
    print('  PART 2 — EXECUTOR GOAL DISPATCH')
    print('═'*62)

    ex = MissionExecutor()
    goals = [
        {'type': 'NavigateToFloor', 'floor': 3},
        {'type': 'InspectArea',     'area': 'north_wing'},
        {'type': 'AvoidObstacle',   'obstacle_type': 'personnel'},
        {'type': 'AvoidObstacle',   'obstacle_type': 'machinery'},
        {'type': 'Report',          'data': 'structural damage on floor 3'},
        {'type': 'LandAt',          'position': [0, 0, 0]},
    ]

    for g in goals:
        ok = ex._execute_single_goal(g)
        print(f'  {"✓" if ok else "✗"} {g["type"]:20s}  '
              f'inflation={_OBSTACLE_INFLATION.get(g.get("obstacle_type",""), "N/A")}m'
              if g["type"] == "AvoidObstacle" else
              f'  {"✓" if ok else "✗"} {g["type"]}')

    print(f'\n  Mission log: {len(ex._mission_log)} entries | '
          f'{sum(1 for e in ex._mission_log if e["success"])} succeeded | '
          f'{sum(1 for e in ex._mission_log if not e["success"])} failed')


# ─────────────────────────────────────────────────────────────────────────────
# Part 3 — VINS-Mono SLAM (synthetic frames)
# ─────────────────────────────────────────────────────────────────────────────

def demo_vins():
    print('\n' + '═'*62)
    print('  PART 3 — VINS-MONO SLAM (synthetic sensor stream)')
    print('═'*62)

    cfg = {
        'camera': {'resolution': [320, 240], 'fx': 300, 'fy': 300,
                   'cx': 160.0, 'cy': 120.0, 'k1': 0.0, 'k2': 0.0},
        'slam':   {'sliding_window_size': 8, 'max_features': 80,
                   'min_features_for_tracking': 8,
                   'keyframe_distance_threshold': 0.1,
                   'optimization_iterations': 3},
        'extrinsics': {'camera_to_imuTranslation': [0,0,0],
                       'camera_to_imuOrientation': [0,0,0,1]},
    }
    vins = VINSMono(cfg)

    print(f'\n  Streaming {30} frames @ synthetic 30 Hz ...\n')
    t = 0.0
    poses_received = 0

    for frame_i in range(30):
        # 5 IMU samples between camera frames (~150 Hz IMU)
        for _ in range(5):
            acc, gyro = _imu_packet(acc=(0, 0, 9.81))
            vins.process_imu(acc, gyro, t)
            t += 1.0 / 150

        # Camera frame — shift by 3px each frame (simulates slow forward motion)
        img = _synthetic_frame(offset_x=frame_i * 3)
        pose = vins.process_image(img, t)
        t += 1.0 / 30

        if pose is not None:
            poses_received += 1
            if poses_received <= 3 or frame_i == 29:
                p  = pose[:3]
                q  = pose[3:]
                qn = np.linalg.norm(q)
                print(f'  frame {frame_i:3d} | '
                      f'pos ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) | '
                      f'|q|={qn:.4f} | '
                      f'kfs={vins._kf_count}')
        elif frame_i < 5:
            print(f'  frame {frame_i:3d} | initialising…')

    final = vins.get_pose()
    print(f'\n  Initialised : {vins.initialized}')
    print(f'  Poses out   : {poses_received}')
    print(f'  Keyframes   : {vins._kf_count}')
    if final is not None:
        print(f'  Final pose  : xyz=({final[0]:+.3f},{final[1]:+.3f},{final[2]:+.3f})')
        print(f'  |q|         : {np.linalg.norm(final[3:]):.6f}')


# ─────────────────────────────────────────────────────────────────────────────
# Part 4 — PoseBridge validation (headless)
# ─────────────────────────────────────────────────────────────────────────────

def demo_pose_bridge():
    print('\n' + '═'*62)
    print('  PART 4 — POSE BRIDGE VALIDATION')
    print('═'*62)

    bridge = PoseBridge()

    def _fake_pose(x, y, z, sec, nsec=0):
        class Msg:
            class header:
                frame_id = 'map'
            class pose:
                class position: pass
                class orientation:
                    x=0; y=0; z=0; w=1
        Msg.header.stamp = type('S',(),{'sec':sec,'nanosec':nsec})()
        Msg.pose.position.x = x
        Msg.pose.position.y = y
        Msg.pose.position.z = z
        return Msg()

    poses = [
        (0.0, 0.0, 0.0, 0),
        (0.1, 0.0, 0.0, 1),    # valid: 0.1m in 1s
        (0.2, 0.0, 0.0, 2),    # valid
        (20.0, 0.0, 0.0, 3),   # JUMP: 19.8m in 1s → rejected
        (0.3, 0.0, 0.0, 4),    # valid again
    ]

    for x, y, z, sec in poses:
        bridge._slam_callback(_fake_pose(x, y, z, sec))

    vel = bridge._velocity
    print(f'\n  Poses accepted : {bridge._pose_count}')
    print(f'  Poses rejected : {bridge._reject_count}')
    print(f'  Final velocity : ({vel[0]:+.3f}, {vel[1]:+.3f}, {vel[2]:+.3f}) m/s')
    assert bridge._reject_count == 1, 'Expected exactly 1 rejected pose'
    assert bridge._pose_count   == 4, 'Expected 4 accepted poses'
    print('  Jump-rejection logic: ✓')


# ─────────────────────────────────────────────────────────────────────────────
# Part 5 — Full end-to-end mission
# ─────────────────────────────────────────────────────────────────────────────

def demo_full_mission():
    print('\n' + '═'*62)
    print('  PART 5 — FULL END-TO-END MISSION')
    print('═'*62)

    # Parse
    parser = MissionParser()
    cmd    = 'Fly to the 2nd floor, inspect the area, avoid personnel, report anomaly'
    goals  = parser._parse_with_regex(cmd)

    print(f'\n  Command : "{cmd}"')
    print(f'  Goals   : {len(goals["goals"])}')

    # Execute against simulated drone
    class Drone:
        pos = np.zeros(3)
        battery = 100.0

        def navigate(self, pos):
            d = np.linalg.norm(np.array(pos) - self.pos)
            self.pos = np.array(pos, dtype=float)
            self.battery -= d * 0.1
            return True

    drone = Drone()
    ex    = MissionExecutor()

    for g in goals['goals']:
        t  = g['type']
        ok = ex._execute_single_goal(g)
        if t == 'NavigateToFloor':
            drone.navigate([0, 0, g['floor']*3.0])
        elif t == 'LandAt':
            drone.navigate(g.get('position', [0,0,0]))
        mark = '✓' if ok else '✗'
        print(f'  {mark} {t}')

    print(f'\n  Final position : {drone.pos.tolist()}')
    print(f'  Battery        : {drone.battery:.1f}%')


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    np.random.seed(42)

    demo_parsing()
    demo_executor()
    demo_vins()
    demo_pose_bridge()
    demo_full_mission()

    print('\n' + '═'*62)
    print('  ✅  Simulation complete — all subsystems operational')
    print('═'*62 + '\n')


if __name__ == '__main__':
    main()
