#!/usr/bin/env python3
"""Mission parser - converts natural language to executable goals."""

import json
import re

# Conditional ROS2 import with mock fallback
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    HAS_ROS2 = True
except ImportError:
    import sys
    import os
    # Mock for testing without ROS2
    class Node:
        def __init__(self, name):
            self._name = name
            self._publishers = []
            self._subscribers = []
            self._parameters = {}
        
        def declare_parameter(self, name, default):
            self._parameters[name] = default
        
        def get_parameter(self, name):
            class Param:
                value = self._parameters.get(name, default)
            return Param()
        
        def create_subscription(self, msg_type, topic, callback, qos):
            return type('Sub', (), {'topic': topic, 'callback': callback})()
        
        def create_publisher(self, msg_type, topic, qos):
            return type('Pub', (), {'topic': topic, 'published': []})()
        
        def get_logger(self):
            return type('Logger', (), {
                'info': lambda s, m, **kw: None,
                'error': lambda s, m, **kw: None
            })()
        
        def destroy_node(self):
            pass
    
    class String:
        def __init__(self):
            self.data = ''
    
    class PoseStamped:
        def __init__(self):
            self.header = type('Header', (), {'frame_id': '', 'stamp': None})()
            self.pose = type('Pose', (), {
                'position': type('Point', (), {'x': 0, 'y': 0, 'z': 0})(),
                'orientation': type('Quaternion', (), {'x': 0, 'y': 0, 'z': 0, 'w': 1})()
            })()
    
    HAS_ROS2 = False


class MissionParser(Node if HAS_ROS2 else object):
    """LLM-based mission command parser."""

    def __init__(self):
        if HAS_ROS2:
            super().__init__('mission_parser')
        else:
            Node.__init__(self, 'mission_parser') if 'Node' in dir() else None
        
        self._parameters = {
            'llm_provider': 'regex',
            'llm_model': 'llama3',
            'llm_endpoint': 'http://localhost:11434'
        }
        
        if HAS_ROS2:
            self.declare_parameter('llm_provider', 'ollama')
            self.declare_parameter('llm_model', 'llama3')
            self.declare_parameter('llm_endpoint', 'http://localhost:11434')
            
            self._parameters = {
                'llm_provider': self.get_parameter('llm_provider').value,
                'llm_model': self.get_parameter('llm_model').value,
                'llm_endpoint': self.get_parameter('llm_endpoint').value
            }
            
            self.mission_sub = self.create_subscription(
                String, '/ghostpilot/mission', self._mission_callback, 10
            )
            self.goals_pub = self.create_publisher(String, '/ghostpilot/goals', 10)
            self.nav_pub = self.create_publisher(PoseStamped, '/ghostpilot/goal_pose', 10)
        
        if HAS_ROS2:
            self.get_logger().info('Mission parser initialized')
        self._register_prompts()

    def _register_prompts(self):
        """Define mission parsing prompts."""
        self.system_prompt = """You are a drone mission planner for GhostPilot.
Parse natural language commands into structured goals.
Output JSON with this structure:
{
  "goals": [
    {"type": "NavigateTo", "target": "waypoint_name", "position": [x, y, z]},
    {"type": "InspectArea", "area": "room_name"},
    {"type": "AvoidObstacle", "obstacle_type": "personnel"},
    {"type": "LandAt", "position": [x, y, z]},
    {"type": "Report", "data": "damage"}
  ]
}
Only output valid JSON, no markdown formatting."""

    def _mission_callback(self, msg):
        """Parse incoming mission command."""
        command = msg.data
        if HAS_ROS2:
            self.get_logger().info(f'Parsing mission: {command}')
        
        goals = self._parse_command(command)
        if goals:
            self._publish_goals(goals)

    def _parse_command(self, command: str) -> dict:
        """Parse natural language to structured goals using LLM."""
        llm_provider = self._parameters.get('llm_provider', 'regex')
        llm_model = self._parameters.get('llm_model', 'llama3')
        llm_endpoint = self._parameters.get('llm_endpoint', 'http://localhost:11434')
        
        if llm_provider == 'ollama':
            return self._parse_with_ollama(command, llm_model, llm_endpoint)
        else:
            return self._parse_with_regex(command)

    def _parse_with_ollama(self, command: str, model: str, endpoint: str) -> dict:
        """Call Ollama LLM for parsing."""
        import requests
        
        payload = {
            'model': model,
            'messages': [
                {'role': 'system', 'content': self.system_prompt},
                {'role': 'user', 'content': command}
            ],
            'stream': False
        }
        
        try:
            resp = requests.post(f'{endpoint}/api/chat', json=payload, timeout=30)
            resp.raise_for_status()
            result = resp.json()
            content = result.get('message', {}).get('content', '{}')
            
            content = re.sub(r'```json\s*', '', content)
            content = re.sub(r'```\s*$', '', content)
            return json.loads(content)
        except Exception as e:
            if HAS_ROS2:
                self.get_logger().error(f'LLM parsing failed: {e}')
            return self._parse_with_regex(command)

    # Ordinal word → floor number
    _WORD_TO_FLOOR = {
        'first': 1, 'second': 2, 'third': 3, 'fourth': 4, 'fifth': 5,
        'sixth': 6, 'seventh': 7, 'eighth': 8, 'ninth': 9, 'tenth': 10,
    }

    def _parse_with_regex(self, command: str) -> dict:
        """Fallback regex-based parsing — handles ordinal words, numeric ordinals, and 'floor N'."""
        goals = []

        # Floor parsing: ordinal words | "N[st/nd/rd/th] floor" | "floor N"
        floor_match = re.search(
            r'(?:(first|second|third|fourth|fifth|sixth|seventh|eighth|ninth|tenth)'
            r'|\b(\d+)(?:st|nd|rd|th)?)\s+floor'
            r'|floor\s+(\d+)',
            command, re.IGNORECASE,
        )
        if floor_match:
            if floor_match.group(1):
                floor_num = self._WORD_TO_FLOOR[floor_match.group(1).lower()]
            elif floor_match.group(2):
                floor_num = int(floor_match.group(2))
            else:
                floor_num = int(floor_match.group(3))
            goals.append({'type': 'NavigateToFloor', 'floor': floor_num})
        
        inspect_match = re.search(r'inspect|check|scan', command, re.I)
        if inspect_match:
            goals.append({'type': 'InspectArea', 'area': 'current'})
        
        avoid_match = re.search(r'avoid\s+(\w+)', command, re.I)
        if avoid_match:
            goals.append({'type': 'AvoidObstacle', 'obstacle_type': avoid_match.group(1)})
        
        land_match = re.search(r'land\s+(?:at|on)', command, re.I)
        if land_match:
            goals.append({'type': 'LandAt', 'position': [0.0, 0.0, 0.0]})
        
        report_match = re.search(r'report\s+(\w+)', command, re.I)
        if report_match:
            goals.append({'type': 'Report', 'data': report_match.group(1)})
        
        if not goals:
            goals.append({'type': 'NavigateTo', 'target': 'unknown', 'position': [0.0, 0.0, 1.0]})
        
        return {'goals': goals}

    def _publish_goals(self, goals: dict):
        """Publish parsed goals."""
        if HAS_ROS2:
            msg = String()
            msg.data = json.dumps(goals)
            self.goals_pub.publish(msg)
            self.get_logger().info(f'Published {len(goals["goals"])} goals')


def main():
    if not HAS_ROS2:
        print("ROS2 not available. Run with ROS2 environment sourced.")
        return
    
    rclpy.init()
    node = MissionParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()