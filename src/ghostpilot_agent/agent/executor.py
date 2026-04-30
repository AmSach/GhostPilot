#!/usr/bin/env python3
"""
GhostPilot Mission Executor

Executes parsed mission goals using Nav2 navigation stack.
Handles goal lifecycle, failure recovery, and mission reporting.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from actionlib_msgs.msg import GoalStatus
import time
from typing import List, Optional
from dataclasses import dataclass

from .mission_parser import Goal, MissionParser


@dataclass
class MissionResult:
    """Result of a mission execution."""
    success: bool
    goals_completed: int
    goals_failed: int
    total_goals: int
    failure_reason: Optional[str] = None


class MissionExecutor(Node):
    """Executes mission commands on the drone navigation stack."""
    
    def __init__(self):
        super().__init__('ghostpilot_mission_executor')
        
        self.parser = MissionParser()
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'NavigateToPose')
        
        # Mission state
        self.current_goal_index = 0
        self.mission_goals: List[Goal] = []
        self.mission_results: List[bool] = []
        
        # Configuration
        self.declare_parameter('mission_timeout', 120.0)  # seconds per goal
        self.declare_parameter('max_retry', 2)
        self.declare_parameter('return_home_on_failure', True)
        
        self.timeout = self.get_parameter('mission_timeout').value
        self.max_retry = self.get_parameter('max_retry').value
        self.return_on_failure = self.get_parameter('return_home_on_failure').value
        
        self.get_logger().info('Mission Executor initialized')
    
    def execute(self, mission_command: str) -> MissionResult:
        """
        Execute a natural language mission command.
        
        Args:
            mission_command: Natural language mission description
            
        Returns:
            MissionResult with execution status
        """
        self.get_logger().info(f"Executing mission: {mission_command}")
        
        # Parse mission into goals
        goals = self.parser.parse(mission_command)
        is_valid, error = self.parser.validate_goals(goals)
        
        if not is_valid:
            return MissionResult(
                success=False,
                goals_completed=0,
                goals_failed=0,
                total_goals=len(goals),
                failure_reason=error
            )
        
        self.mission_goals = goals
        self.mission_results = []
        self.current_goal_index = 0
        
        # Execute each goal
        for i, goal in enumerate(goals):
            self.current_goal_index = i
            self.get_logger().info(f"Executing goal {i+1}/{len(goals)}: {goal.action}")
            
            success = self._execute_goal(goal)
            self.mission_results.append(success)
            
            if not success and self.return_on_failure:
                self.get_logger().warn("Goal failed, returning home")
                self._return_home()
                break
        
        # Generate mission report
        result = MissionResult(
            success=all(self.mission_results),
            goals_completed=sum(self.mission_results),
            goals_failed=len(self.mission_results) - sum(self.mission_results),
            total_goals=len(self.mission_results)
        )
        
        self.get_logger().info(f"Mission complete: {result.goals_completed}/{result.total_goals} goals")
        return result
    
    def _execute_goal(self, goal: Goal) -> bool:
        """Execute a single goal with retry logic."""
        for attempt in range(self.max_retry + 1):
            if attempt > 0:
                self.get_logger().info(f"Retry attempt {attempt}")
            
            success = self._navigate_to_goal(goal)
            if success:
                return True
            
            # Execute post-goal action if applicable
            if goal.action == "inspect":
                self._perform_inspection(goal)
            
            time.sleep(1)  # Brief pause between attempts
        
        return False
    
    def _navigate_to_goal(self, goal: Goal) -> bool:
        """Send navigation goal to Nav2."""
        if goal.action == "land":
            return self._land()
        
        if goal.action == "return":
            return self._return_home()
        
        # Build pose for navigation
        pose = self._build_pose(goal)
        
        if not pose:
            self.get_logger().warn("Could not build pose for goal")
            return False
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            return False
        
        # Send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info("Sending navigation goal...")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=self.timeout)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout)
        
        result = result_future.result()
        return result and result.status == GoalStatus.STATUS_SUCCEEDED
    
    def _build_pose(self, goal: Goal) -> Optional[PoseStamped]:
        """Build a PoseStamped message from goal location."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Waypoint lookup or default positions
        # In production, this would use a location database or SLAM map
        location_poses = {
            'home': (0.0, 0.0, 0.0),
            'building_b': (10.0, 5.0, 3.0),  # floor 3
            'zone_a': (5.0, 0.0, 1.5),
            'warehouse': (15.0, -5.0, 2.0),
            'helipad': (0.0, 0.0, 0.0),
            'room_1': (11.0, 5.5, 3.0),
            'room_2': (11.0, 6.5, 3.0),
        }
        
        location_key = (goal.location or '').lower()
        
        if location_key in location_poses:
            x, y, z = location_poses[location_key]
        elif goal.waypoint:
            x = goal.waypoint.get('x', 0.0)
            y = goal.waypoint.get('y', 0.0)
            z = goal.waypoint.get('z', 1.5)
        else:
            # Default position
            x = 2.0 * (self.current_goal_index + 1)
            y = 0.0
            z = goal.constraints.get('altitude', 1.5)
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Default orientation (facing forward)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        return pose
    
    def _perform_inspection(self, goal: Goal):
        """Perform inspection action at current location."""
        self.get_logger().info(f"Performing inspection: {goal.location or 'current area'}")
        # In production: activate sensors, scan, collect data
        time.sleep(2)
    
    def _land(self) -> bool:
        """Execute landing maneuver."""
        self.get_logger().info("Executing landing")
        # In production: send land command to flight controller
        return True
    
    def _return_home(self) -> bool:
        """Return to home position."""
        self.get_logger().info("Returning home")
        home_goal = Goal(action='navigate', location='home')
        return self._navigate_to_goal(home_goal)


def main(args=None):
    rclpy.init(args=args)
    executor = MissionExecutor()
    
    try:
        # Demo execution
        result = executor.execute(
            "Fly to the third floor, check each room for occupants, land at the helipad"
        )
        print(f"Mission result: {result}")
    except KeyboardInterrupt:
        print("Mission executor interrupted")
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()