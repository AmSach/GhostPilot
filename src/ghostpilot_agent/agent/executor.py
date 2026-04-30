#!/usr/bin/env python3
"""Mission executor - converts parsed goals to navigation actions."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json


class MissionExecutor(Node):
    """Executes parsed mission goals via Nav2."""

    def __init__(self):
        super().__init__('mission_executor')
        
        self.goals_sub = self.create_subscription(
            String, '/ghostpilot/goals', self._goals_callback, 10
        )
        self.nav_pub = self.create_publisher(PoseStamped, '/ghostpilot/goal_pose', 10)
        
        self._action_client = ActionClient(self, NavigateToPose, '/NavigateToPose')
        self._current_goal = None
        self._goal_index = 0
        
        self.get_logger().info('Mission executor initialized')

    def _goals_callback(self, msg: String):
        """Execute incoming goal list."""
        try:
            goals = json.loads(msg.data)
            self._execute_goals(goals.get('goals', []))
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid goals JSON: {e}')

    def _execute_goals(self, goals: list):
        """Execute goals in sequence."""
        if not goals:
            self.get_logger().info('No goals to execute')
            return
        
        for i, goal in enumerate(goals):
            self.get_logger().info(f'Executing goal {i+1}/{len(goals)}: {goal["type"]}')
            success = self._execute_single_goal(goal)
            
            if not success:
                self.get_logger().warn(f'Goal {i+1} failed, stopping execution')
                break
        
        self.get_logger().info('Goal execution complete')

    def _execute_single_goal(self, goal: dict) -> bool:
        """Execute a single goal based on type."""
        goal_type = goal.get('type')
        
        if goal_type == 'NavigateTo':
            return self._navigate_to(goal.get('position', [0, 0, 1]))
        elif goal_type == 'NavigateToFloor':
            return self._navigate_to_floor(goal.get('floor', 1))
        elif goal_type == 'InspectArea':
            return self._inspect_area(goal.get('area', 'current'))
        elif goal_type == 'AvoidObstacle':
            return self._avoid_obstacle(goal.get('obstacle_type', 'unknown'))
        elif goal_type == 'LandAt':
            return self._land_at(goal.get('position', [0, 0, 0]))
        elif goal_type == 'Report':
            return self._send_report(goal.get('data', ''))
        else:
            self.get_logger().warn(f'Unknown goal type: {goal_type}')
            return False

    def _navigate_to(self, position: list) -> bool:
        """Navigate to a 3D position."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(position[0])
        goal_msg.pose.position.y = float(position[1])
        goal_msg.pose.position.z = float(position[2])
        goal_msg.pose.orientation.w = 1.0
        
        self.nav_pub.publish(goal_msg)
        self.get_logger().info(f'Navigating to {position}')
        return True

    def _navigate_to_floor(self, floor: int) -> bool:
        """Navigate to a specific floor (z = floor * 3m)."""
        z_position = floor * 3.0
        self.get_logger().info(f'Navigating to floor {floor} (z={z_position}m)')
        return self._navigate_to([0.0, 0.0, z_position])

    def _inspect_area(self, area: str) -> bool:
        """Perform area inspection sweep."""
        self.get_logger().info(f'Inspecting area: {area}')
        waypoints = [
            [-2.0, 0.0, 1.5],
            [-2.0, 2.0, 1.5],
            [2.0, 2.0, 1.5],
            [2.0, -2.0, 1.5],
            [-2.0, -2.0, 1.5],
            [0.0, 0.0, 1.5],
        ]
        for wp in waypoints:
            self._navigate_to(wp)
        return True

    def _avoid_obstacle(self, obstacle_type: str) -> bool:
        """Configure obstacle avoidance for specific type."""
        self.get_logger().info(f'Configuring avoidance for: {obstacle_type}')
        return True

    def _land_at(self, position: list) -> bool:
        """Execute landing at position."""
        self.get_logger().info(f'Landing at {position}')
        return self._navigate_to(position)

    def _send_report(self, data: str) -> bool:
        """Generate mission report."""
        self.get_logger().info(f'Report: {data}')
        return True


def main():
    rclpy.init()
    node = MissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()