#!/usr/bin/env python3
"""Mission executor - converts parsed goals to navigation actions."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import json
import asyncio


class MissionExecutor(Node):
    """Executes parsed mission goals via Nav2 with proper async goal completion."""

    def __init__(self):
        super().__init__('mission_executor')
        
        # Reentrant callback group for async action handling
        self._callback_group = ReentrantCallbackGroup()
        
        self.goals_sub = self.create_subscription(
            String, '/ghostpilot/goals', self._goals_callback, 10
        )
        
        # Nav2 NavigateToPose action client
        self._nav2_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._callback_group
        )
        
        self._current_goal_handle = None
        self._goal_queue = []
        self._executing = False
        
        self.get_logger().info('Mission executor initialized')

    def _goals_callback(self, msg: String):
        """Queue incoming goals for execution."""
        try:
            goals_data = json.loads(msg.data)
            goals = goals_data.get('goals', [])
            
            if not goals:
                self.get_logger().warn('Received empty goals list')
                return
            
            self._goal_queue.extend(goals)
            self.get_logger().info(f'Queued {len(goals)} goals, queue size: {len(self._goal_queue)}')
            
            # Start execution if not already running
            if not self._executing:
                self._execute_next_goal()
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid goals JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Goals callback error: {e}')

    def _execute_next_goal(self):
        """Execute the next goal in the queue."""
        if not self._goal_queue:
            self._executing = False
            self.get_logger().info('All goals executed')
            return
        
        self._executing = True
        goal = self._goal_queue.pop(0)
        
        self.get_logger().info(f'Executing goal: {goal.get("type", "unknown")}')
        
        success = self._execute_single_goal(goal)
        if not success:
            self.get_logger().warn('Goal execution failed, continuing to next')
            self._execute_next_goal()

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
        """Navigate to a 3D position using Nav2 action."""
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(position[0])
        goal_msg.pose.pose.position.y = float(position[1])
        goal_msg.pose.pose.position.z = float(position[2])
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending NavigateToPose goal: {position}')
        
        send_goal_future = self._nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 server')
            self._execute_next_goal()
            return
        
        self.get_logger().info('Goal accepted by Nav2 server')
        self._current_goal_handle = goal_handle
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Handle goal completion."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal was aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled')
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')
        
        self._current_goal_handle = None
        self._execute_next_goal()

    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        # Log progress periodically
        self.get_logger().debug(
            f'Navigation feedback: current_pose={feedback.current_pose.pose.position}',
            throttle_duration_sec=2.0
        )

    def _navigate_to_floor(self, floor: int) -> bool:
        """Navigate to a specific floor (z = floor * 3m)."""
        z_position = floor * 3.0
        self.get_logger().info(f'Navigating to floor {floor} (z={z_position}m)')
        return self._navigate_to([0.0, 0.0, z_position])

    def _inspect_area(self, area: str) -> bool:
        """Perform area inspection sweep."""
        self.get_logger().info(f'Inspecting area: {area}')
        
        # Add inspection waypoints to front of queue
        waypoints = [
            [-2.0, 0.0, 1.5],
            [-2.0, 2.0, 1.5],
            [2.0, 2.0, 1.5],
            [2.0, -2.0, 1.5],
            [-2.0, -2.0, 1.5],
            [0.0, 0.0, 1.5],
        ]
        
        for wp in waypoints:
            self._goal_queue.insert(0, {'type': 'NavigateTo', 'position': wp})
        
        self._execute_next_goal()
        return True

    def _avoid_obstacle(self, obstacle_type: str) -> bool:
        """Configure obstacle avoidance for specific type."""
        self.get_logger().info(f'Configuring avoidance for: {obstacle_type}')
        # This would configure Nav2 costmap layers dynamically
        return True

    def _land_at(self, position: list) -> bool:
        """Execute landing at position."""
        self.get_logger().info(f'Landing at {position}')
        return self._navigate_to(position)

    def _send_report(self, data: str) -> bool:
        """Generate mission report."""
        self.get_logger().info(f'Report generated: {data}')
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