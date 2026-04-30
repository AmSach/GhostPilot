#!/usr/bin/env python3
"""Mission parser - converts natural language to executable goals."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import re


class MissionParser(Node):
    """LLM-based mission command parser."""

    def __init__(self):
        super().__init__('mission_parser')
        
        self.declare_parameter('llm_provider', 'ollama')
        self.declare_parameter('llm_model', 'llama3')
        self.declare_parameter('llm_endpoint', 'http://localhost:11434')
        
        self.mission_sub = self.create_subscription(
            String, '/ghostpilot/mission', self._mission_callback, 10
        )
        self.goals_pub = self.create_publisher(String, '/ghostpilot/goals', 10)
        self.nav_pub = self.create_publisher(PoseStamped, '/ghostpilot/goal_pose', 10)
        
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

    def _mission_callback(self, msg: String):
        """Parse incoming mission command."""
        command = msg.data
        self.get_logger().info(f'Parsing mission: {command}')
        
        goals = self._parse_command(command)
        if goals:
            self._publish_goals(goals)

    def _parse_command(self, command: str) -> dict:
        """Parse natural language to structured goals using LLM."""
        llm_provider = self.get_parameter('llm_provider').value
        llm_model = self.get_parameter('llm_model').value
        llm_endpoint = self.get_parameter('llm_endpoint').value
        
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
            self.get_logger().error(f'LLM parsing failed: {e}')
            return self._parse_with_regex(command)

    def _parse_with_regex(self, command: str) -> dict:
        """Fallback regex-based parsing."""
        goals = []
        
        floor_match = re.search(r'third floor|3rd floor|floor (\d+)', command, re.I)
        if floor_match:
            floor_num = int(re.search(r'\d+', floor_match.group()).group()) if floor_match.group().startswith(('third', '3rd')) else int(floor_match.group(1))
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
        msg = String()
        msg.data = json.dumps(goals)
        self.goals_pub.publish(msg)
        self.get_logger().info(f'Published {len(goals["goals"])} goals')


def main():
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