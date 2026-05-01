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
    # Mock for testing without ROS2
    class Node:
        def __init__(self, name):
            self._name = name
            self._parameters = {}

        def declare_parameter(self, name, default):
            self._parameters[name] = default

        def get_parameter(self, name):
            val = self._parameters.get(name)
            return type('Param', (), {'value': val})()

        def create_subscription(self, msg_type, topic, callback, qos):
            return type('Sub', (), {'topic': topic})()

        def create_publisher(self, msg_type, topic, qos):
            return type('Pub', (), {'topic': topic, 'published': []})()

        def get_logger(self):
            return type('Logger', (), {
                'info': lambda s, m, **kw: None,
                'error': lambda s, m, **kw: None,
                'warn': lambda s, m, **kw: None,
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

# FIX: use a relative import that works whether or not the package is installed
try:
    from ghostpilot_agent.prompts.mission_prompts import (
        SYSTEM_PROMPT, MISSION_EXAMPLES, get_mission_prompt,
    )
except ImportError:
    import os as _os, sys as _sys
    _sys.path.insert(0, _os.path.join(_os.path.dirname(__file__), '..', 'prompts'))
    from mission_prompts import SYSTEM_PROMPT, MISSION_EXAMPLES, get_mission_prompt


# Ordinal word → int mapping for floor parsing
_WORD_TO_FLOOR = {
    'first': 1, 'second': 2, 'third': 3, 'fourth': 4, 'fifth': 5,
    'sixth': 6, 'seventh': 7, 'eighth': 8, 'ninth': 9, 'tenth': 10,
}

# Goal data class (so callers can do `from agent.mission_parser import Goal`)
class Goal(dict):
    pass


class MissionParser(Node if HAS_ROS2 else object):
    """LLM-based mission command parser using mission_prompts module."""

    def __init__(self):
        if HAS_ROS2:
            super().__init__('mission_parser')

        self._parameters = {
            'llm_provider': 'regex',
            'llm_model': 'llama3',
            'llm_endpoint': 'http://localhost:11434',
            'use_examples': True,
        }

        if HAS_ROS2:
            self.declare_parameter('llm_provider', 'ollama')
            self.declare_parameter('llm_model', 'llama3')
            self.declare_parameter('llm_endpoint', 'http://localhost:11434')
            self.declare_parameter('use_examples', True)

            self._parameters = {
                'llm_provider': self.get_parameter('llm_provider').value,
                'llm_model': self.get_parameter('llm_model').value,
                'llm_endpoint': self.get_parameter('llm_endpoint').value,
                'use_examples': self.get_parameter('use_examples').value,
            }

            self.mission_sub = self.create_subscription(
                String, '/ghostpilot/mission', self._mission_callback, 10
            )
            self.goals_pub = self.create_publisher(String, '/ghostpilot/goals', 10)
            self.nav_pub = self.create_publisher(PoseStamped, '/ghostpilot/goal_pose', 10)

            self.get_logger().info(
                f'Mission parser initialized '
                f'(provider={self._parameters["llm_provider"]}, '
                f'model={self._parameters["llm_model"]})'
            )

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def parse(self, command: str) -> dict:
        """Public entry point: parse a natural-language command → goals dict."""
        return self._parse_command(command)

    # ------------------------------------------------------------------ #
    #  Internal                                                            #
    # ------------------------------------------------------------------ #

    def _mission_callback(self, msg: String):
        command = msg.data
        if HAS_ROS2:
            self.get_logger().info(f'Parsing mission: {command}')
        goals = self._parse_command(command)
        if goals:
            self._publish_goals(goals)

    def _parse_command(self, command: str) -> dict:
        provider = self._parameters.get('llm_provider', 'regex')
        if provider == 'ollama':
            return self._parse_with_ollama(command)
        elif provider == 'openai':
            return self._parse_with_openai(command)
        else:
            return self._parse_with_regex(command)

    def _parse_with_ollama(self, command: str) -> dict:
        import requests
        model = self._parameters['llm_model']
        endpoint = self._parameters['llm_endpoint']
        messages = get_mission_prompt(command)

        if self._parameters.get('use_examples') and MISSION_EXAMPLES:
            examples_text = '\n\nExamples:\n'
            for ex in MISSION_EXAMPLES[:2]:
                examples_text += f"Input: {ex['input']}\n"
                examples_text += f"Output: {json.dumps({'goals': ex['goals']})}\n\n"
            messages[0]['content'] += examples_text

        payload = {'model': model, 'messages': messages, 'stream': False}
        try:
            resp = requests.post(f'{endpoint}/api/chat', json=payload, timeout=30)
            resp.raise_for_status()
            content = resp.json().get('message', {}).get('content', '{}')
            content = re.sub(r'```json\s*', '', content)
            content = re.sub(r'```\s*$', '', content).strip()
            return json.loads(content)
        except Exception as e:
            if HAS_ROS2:
                self.get_logger().error(f'Ollama parsing failed: {e}')
            return self._parse_with_regex(command)

    def _parse_with_openai(self, command: str) -> dict:
        import os, requests
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            if HAS_ROS2:
                self.get_logger().error('OPENAI_API_KEY not set, falling back to regex')
            return self._parse_with_regex(command)

        messages = get_mission_prompt(command)
        try:
            resp = requests.post(
                'https://api.openai.com/v1/chat/completions',
                headers={'Authorization': f'Bearer {api_key}'},
                json={'model': self._parameters.get('llm_model', 'gpt-4o-mini'),
                      'messages': messages, 'temperature': 0.1},
                timeout=30,
            )
            resp.raise_for_status()
            content = resp.json()['choices'][0]['message']['content']
            content = re.sub(r'```json\s*', '', content)
            content = re.sub(r'```\s*$', '', content).strip()
            return json.loads(content)
        except Exception as e:
            if HAS_ROS2:
                self.get_logger().error(f'OpenAI parsing failed: {e}')
            return self._parse_with_regex(command)

    def _parse_with_regex(self, command: str) -> dict:
        """Regex fallback — handles ordinal words, numeric floors, and all goal types."""
        goals = []

        # Floor parsing: ordinal words OR "N[st|nd|rd|th] floor" OR "floor N"
        floor_match = re.search(
            r'(?:(first|second|third|fourth|fifth|sixth|seventh|eighth|ninth|tenth)'
            r'|\b(\d+)(?:st|nd|rd|th)?)\s+floor'
            r'|floor\s+(\d+)',
            command, re.IGNORECASE,
        )
        if floor_match:
            if floor_match.group(1):
                floor_num = _WORD_TO_FLOOR[floor_match.group(1).lower()]
            elif floor_match.group(2):
                floor_num = int(floor_match.group(2))
            else:
                floor_num = int(floor_match.group(3))
            goals.append({'type': 'NavigateToFloor', 'floor': floor_num})

        if re.search(r'\b(?:inspect|check|scan)\b', command, re.I):
            goals.append({'type': 'InspectArea', 'area': 'current'})

        m = re.search(r'\bavoid\s+(\w+)', command, re.I)
        if m:
            goals.append({'type': 'AvoidObstacle', 'obstacle_type': m.group(1)})

        if re.search(r'\bland\s+(?:at|on)\b', command, re.I):
            goals.append({'type': 'LandAt', 'position': [0.0, 0.0, 0.0]})

        m = re.search(r'\breport\s+(\w+)', command, re.I)
        if m:
            goals.append({'type': 'Report', 'data': m.group(1)})

        if not goals:
            goals.append({'type': 'NavigateTo', 'target': 'unknown', 'position': [0.0, 0.0, 1.0]})

        return {'goals': goals}

    def _publish_goals(self, goals: dict):
        if HAS_ROS2:
            msg = String()
            msg.data = json.dumps(goals)
            self.goals_pub.publish(msg)
            self.get_logger().info(f'Published {len(goals.get("goals", []))} goals')


def main():
    if not HAS_ROS2:
        print('ROS2 not available. Run with ROS2 environment sourced.')
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
