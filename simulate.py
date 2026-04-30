#!/usr/bin/env python3
"""GhostPilot Simulation - Run without ROS2/Gazebo to test core logic."""

import sys
import os
import json
import time
import numpy as np

# Add source paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'ghostpilot_agent', 'ghostpilot_agent'))

from mission_parser import MissionParser

class SimulatedDrone:
    """Simulated drone state for testing."""
    
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.battery = 100.0
        self.mission_active = False
        
    def navigate_to(self, target_pos, speed=1.0):
        """Simulate navigation to target."""
        target = np.array(target_pos)
        distance = np.linalg.norm(target - self.position)
        duration = distance / speed
        
        print(f"  📍 Navigating from {self.position.tolist()} to {target_pos}")
        print(f"  ⏱️  Distance: {distance:.2f}m, ETA: {duration:.2f}s")
        
        # Simulate movement
        self.position = target
        self.battery -= distance * 0.1  # 0.1% per meter
        
        return True
    
    def inspect_area(self, area_name):
        """Simulate area inspection."""
        print(f"  🔍 Inspecting area: {area_name}")
        print(f"  📸 Capturing images...")
        time.sleep(0.1)  # Simulate processing
        return True
    
    def land(self):
        """Simulate landing."""
        print(f"  🛬 Landing at current position: {self.position.tolist()}")
        self.position[2] = 0.0
        return True
    
    def report_status(self):
        """Report drone status."""
        return {
            'position': self.position.tolist(),
            'battery': self.battery,
            'mission_active': self.mission_active
        }


class MissionSimulator:
    """Simulate full mission execution."""
    
    def __init__(self):
        self.drone = SimulatedDrone()
        self.parser = MissionParser()
        
    def run_mission(self, command: str):
        """Parse and execute a mission command."""
        print("\n" + "="*60)
        print(f"🚁 GHOSTPILOT MISSION SIMULATOR")
        print("="*60)
        print(f"\n📝 Command: \"{command}\"")
        
        # Parse command
        goals = self.parser._parse_with_regex(command)
        print(f"\n📋 Parsed Goals:")
        for i, goal in enumerate(goals['goals'], 1):
            print(f"  {i}. {goal['type']}: {goal}")
        
        # Execute goals
        print(f"\n🚀 Executing Mission:")
        self.drone.mission_active = True
        
        for i, goal in enumerate(goals['goals'], 1):
            print(f"\n[Goal {i}/{len(goals['goals'])}]")
            success = self._execute_goal(goal)
            
            if not success:
                print(f"  ❌ Goal failed, aborting mission")
                break
        
        self.drone.mission_active = False
        
        # Report final status
        print(f"\n📊 Mission Complete!")
        status = self.drone.report_status()
        print(f"  Position: {status['position']}")
        print(f"  Battery: {status['battery']:.1f}%")
        
    def _execute_goal(self, goal: dict) -> bool:
        """Execute a single goal."""
        goal_type = goal.get('type')
        
        if goal_type == 'NavigateTo':
            pos = goal.get('position', [0, 0, 1])
            return self.drone.navigate_to(pos)
        
        elif goal_type == 'NavigateToFloor':
            floor = goal.get('floor', 1)
            z = floor * 3.0
            return self.drone.navigate_to([0, 0, z])
        
        elif goal_type == 'InspectArea':
            area = goal.get('area', 'current')
            return self.drone.inspect_area(area)
        
        elif goal_type == 'AvoidObstacle':
            obstacle = goal.get('obstacle_type', 'unknown')
            print(f"  ⚠️  Configuring avoidance for: {obstacle}")
            return True
        
        elif goal_type == 'LandAt':
            pos = goal.get('position', self.drone.position.tolist())
            self.drone.navigate_to(pos)
            return self.drone.land()
        
        elif goal_type == 'Report':
            data = goal.get('data', 'status')
            print(f"  📄 Reporting: {data}")
            return True
        
        else:
            print(f"  ⚠️  Unknown goal type: {goal_type}")
            return False


def main():
    """Run simulation demos."""
    sim = MissionSimulator()
    
    # Demo missions
    missions = [
        "Fly to floor 3 and inspect the area",
        "Navigate to floor 5, avoid personnel, and land at base",
        "Inspect the area and report damage",
    ]
    
    for mission in missions:
        sim.run_mission(mission)
        time.sleep(0.5)
    
    print("\n" + "="*60)
    print("✅ Simulation Complete - All systems operational")
    print("="*60)


if __name__ == '__main__':
    main()