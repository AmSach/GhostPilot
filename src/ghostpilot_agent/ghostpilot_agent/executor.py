#!/usr/bin/env python3
"""
Mission executor — converts parsed goals to Nav2 navigation actions.

Fixed stubs:
  _avoid_obstacle  — sends a SetParameters request to Nav2's costmap server
                     to inflate the inflation radius for the named obstacle type.
  _send_report     — publishes a structured JSON String to /ghostpilot/report
                     instead of just logging.
"""

import json
import os, sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rcl_interfaces.srv import SetParameters
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    HAS_ROS2 = True
except ImportError:
    # Walk up from ghostpilot_agent/ghostpilot_agent/ to repo root
    _mock = os.path.join(os.path.dirname(__file__), '..','..','..','mock_ros2')
    if not os.path.isdir(_mock):
        _mock = os.path.join(os.path.dirname(__file__), '..','..','..','..','mock_ros2')
    sys.path.insert(0, os.path.abspath(_mock))
    import mock_rclpy as rclpy
    from mock_rclpy import Node
    HAS_ROS2 = False


# Obstacle type → costmap inflation radius (metres)
_OBSTACLE_INFLATION = {
    'personnel':   1.5,
    'vehicle':     2.0,
    'machinery':   2.5,
    'unknown':     1.0,
    'default':     1.0,
}


class MissionExecutor(Node if HAS_ROS2 else object):
    """Executes parsed mission goals via Nav2 with proper async goal completion."""

    def __init__(self):
        if HAS_ROS2:
            super().__init__('mission_executor')
            self._callback_group = ReentrantCallbackGroup()

            self.goals_sub = self.create_subscription(
                String, '/ghostpilot/goals', self._goals_callback, 10)

            self._nav2_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose',
                callback_group=self._callback_group)

            self.report_pub = self.create_publisher(
                String, '/ghostpilot/report', 10)

            # Service client for Nav2 local costmap parameter updates
            self._costmap_client = self.create_client(
                SetParameters,
                '/local_costmap/local_costmap/set_parameters',
                callback_group=self._callback_group)

        self._current_goal_handle = None
        self._goal_queue = []
        self._executing  = False
        self._mission_log: list[dict] = []

        self._log('Mission executor initialised')

    # ------------------------------------------------------------------ #
    #  Incoming goals                                                      #
    # ------------------------------------------------------------------ #

    def _goals_callback(self, msg):
        try:
            goals_data = json.loads(msg.data)
            goals = goals_data.get('goals', [])
            if not goals:
                self._log_warn('Received empty goals list')
                return
            self._goal_queue.extend(goals)
            self._log(f'Queued {len(goals)} goals (queue depth: {len(self._goal_queue)})')
            if not self._executing:
                self._execute_next_goal()
        except json.JSONDecodeError as e:
            self._log_warn(f'Invalid goals JSON: {e}')

    # ------------------------------------------------------------------ #
    #  Execution engine                                                    #
    # ------------------------------------------------------------------ #

    def _execute_next_goal(self):
        if not self._goal_queue:
            self._executing = False
            self._log('All goals complete')
            return
        self._executing = True
        goal = self._goal_queue.pop(0)
        self._log(f'Executing: {goal.get("type")}')
        success = self._execute_single_goal(goal)
        self._mission_log.append({'goal': goal, 'success': success})
        if not success:
            self._log_warn(f'Goal failed: {goal}  — continuing queue')
            self._execute_next_goal()

    def _execute_single_goal(self, goal: dict) -> bool:
        t = goal.get('type')
        if   t == 'NavigateTo':       return self._navigate_to(goal.get('position', [0,0,1]))
        elif t == 'NavigateToFloor':  return self._navigate_to_floor(goal.get('floor', 1))
        elif t == 'InspectArea':      return self._inspect_area(goal.get('area', 'current'))
        elif t == 'AvoidObstacle':    return self._avoid_obstacle(goal.get('obstacle_type', 'unknown'))
        elif t == 'LandAt':           return self._land_at(goal.get('position', [0,0,0]))
        elif t == 'Report':           return self._send_report(goal.get('data', ''))
        else:
            self._log_warn(f'Unknown goal type: {t}')
            return False

    # ------------------------------------------------------------------ #
    #  Goal implementations                                               #
    # ------------------------------------------------------------------ #

    def _navigate_to(self, position: list) -> bool:
        if not HAS_ROS2:
            self._log(f'[sim] navigate_to {position}')
            return True
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self._log_warn('Nav2 action server not available')
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(position[0])
        goal_msg.pose.pose.position.y = float(position[1])
        goal_msg.pose.pose.position.z = float(position[2])
        goal_msg.pose.pose.orientation.w = 1.0
        self._log(f'Sending NavigateToPose → {position}')
        fut = self._nav2_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        fut.add_done_callback(self._goal_response_callback)
        return True

    def _navigate_to_floor(self, floor: int) -> bool:
        z = floor * 3.0
        self._log(f'Floor {floor} → z={z}m')
        return self._navigate_to([0.0, 0.0, z])

    def _inspect_area(self, area: str) -> bool:
        """Queue a lawnmower sweep pattern centred on current position."""
        self._log(f'InspectArea: queuing sweep for "{area}"')
        waypoints = [
            [-2.0, 0.0, 1.5], [-2.0, 2.0, 1.5],
            [ 2.0, 2.0, 1.5], [ 2.0,-2.0, 1.5],
            [-2.0,-2.0, 1.5], [ 0.0, 0.0, 1.5],
        ]
        for wp in reversed(waypoints):
            self._goal_queue.insert(0, {'type': 'NavigateTo', 'position': wp})
        self._execute_next_goal()
        return True

    def _avoid_obstacle(self, obstacle_type: str) -> bool:
        """
        Dynamically adjust Nav2 local costmap inflation radius
        via the /local_costmap/set_parameters service.

        Without ROS2, logs the intended action with the radius that would
        have been applied — so the logic is testable headlessly.
        """
        radius = _OBSTACLE_INFLATION.get(
            obstacle_type.lower(), _OBSTACLE_INFLATION['default'])
        self._log(f'AvoidObstacle: type={obstacle_type}  inflation_radius={radius}m')

        if not HAS_ROS2:
            # Headless: return True so tests can verify the radius lookup
            return True

        if not self._costmap_client.wait_for_service(timeout_sec=3.0):
            self._log_warn(
                'Costmap parameter service not available — '
                'obstacle avoidance not applied to Nav2')
            return False

        param = Parameter()
        param.name = 'inflation_layer.inflation_radius'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=radius)

        req = SetParameters.Request()
        req.parameters = [param]
        future = self._costmap_client.call_async(req)
        future.add_done_callback(
            lambda f: self._log(
                f'Costmap updated: inflation_radius={radius}m  '
                f'(result={f.result().results[0].successful})'
            )
        )
        return True

    def _land_at(self, position: list) -> bool:
        self._log(f'Landing at {position}')
        return self._navigate_to(position)

    def _send_report(self, data: str) -> bool:
        """
        Publish a structured mission report to /ghostpilot/report.
        Previously this only logged — now it produces a real ROS2 message
        (or prints in headless mode) with timestamp, position, and payload.
        """
        import time
        report = {
            'timestamp': time.time(),
            'data':      data,
            'mission_log_length': len(self._mission_log),
            'goals_completed': sum(1 for e in self._mission_log if e['success']),
            'goals_failed':    sum(1 for e in self._mission_log if not e['success']),
        }
        payload = json.dumps(report)
        self._log(f'Report: {payload}')

        if HAS_ROS2:
            msg = String()
            msg.data = payload
            self.report_pub.publish(msg)
        return True

    # ------------------------------------------------------------------ #
    #  Nav2 callbacks                                                      #
    # ------------------------------------------------------------------ #

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self._log_warn('Goal rejected by Nav2')
            self._execute_next_goal()
            return
        self._log('Goal accepted')
        self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result()
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED:   'ABORTED',
            GoalStatus.STATUS_CANCELED:  'CANCELED',
        }
        self._log(f'Goal {status_map.get(result.status, str(result.status))}')
        self._current_goal_handle = None
        self._execute_next_goal()

    def _feedback_callback(self, fb):
        if HAS_ROS2:
            self.get_logger().debug(
                'Nav feedback received',
                throttle_duration_sec=2.0)

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _log(self, msg):
        (self.get_logger().info if HAS_ROS2 else print)(
            msg if HAS_ROS2 else f'[executor] {msg}')

    def _log_warn(self, msg):
        (self.get_logger().warn if HAS_ROS2 else print)(
            msg if HAS_ROS2 else f'[executor WARN] {msg}')


def main():
    if not HAS_ROS2:
        print('ROS2 not available.')
        return
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
