#!/usr/bin/env python3
"""Mission parser - converts natural language to executable goals."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import re

# Import prompts from mission_prompts module (same directory)
from .mission_prompts import (
    SYSTEM_PROMPT,
    MISSION_EXAMPLES,
    get_mission_prompt,
)


class MissionParser(Node):
    """LLM-based mission command parser using mission_prompts module."""

    def __init__(self):
        super().__init__('mission_parser')
        
        self.declare_parameter('llm_provider', 'ollama')
        self.declare_parameter('llm_model', 'llama3')
        self.declare_parameter('llm_endpoint', 'http://localhost:11434')
        self.declare_parameter('use_examples', True)
        
        self.mission_sub = self.create_subscription(
            String, '/ghostpilot/mission', self._mission_callback, 10
        )
        self.goals_pub = self.create_publisher(String, '/ghostpilot/goals', 10)
        self.nav_pub = self.create_publisher(PoseStamped, '/ghostpilot/goal_pose', 10)
        
        # Store configuration
        self._llm_provider = self.get_parameter('llm_provider').value
        self._llm_model = self.get_parameter('llm_model').value
        self._llm_endpoint = self.get_parameter('llm_endpoint').value
        self._use_examples = self.get_parameter('use_examples').value
        
        self.get_logger().info(
            f'Mission parser initialized (provider={self._llm_provider}, model={self._llm_model})'
        )

    def _mission_callback(self, msg: String):
        """Parse incoming mission command."""
        command = msg.data
        self.get_logger().info(f'Parsing mission: {command}')
        
        goals = self._parse_command(command)
        if goals:
            self._publish_goals(goals)

    def _parse_command(self, command: str) -> dict:
        """Parse natural language to structured goals using LLM."""
        if self._llm_provider == 'ollama':
            return self._parse_with_ollama(command)
        elif self._llm_provider == 'openai':
            return self._parse_with_openai(command)
        else:
            return self._parse_with_regex(command)

    def _parse_with_ollama(self, command: str) -> dict:
        """Call Ollama LLM for parsing using mission_prompts module."""
        import requests
        
        # Use get_mission_prompt from mission_prompts.py
        messages = get_mission_prompt(command)
        
        # Optionally add examples as few-shot context
        if self._use_examples and MISSION_EXAMPLES:
            examples_text = "\n\nExamples:\n"
            for ex in MISSION_EXAMPLES[:2]:  # Use first 2 examples
                examples_text += f"Input: {ex['input']}\n"
                examples_text += f"Output: {json.dumps({'goals': ex['goals']})}\n\n"
            
            messages[0]['content'] += examples_text
        
        payload = {
            'model': self._llm_model,
            'messages': messages,
            'stream': False
        }
        
        try:
            resp = requests.post(
                f'{self._llm_endpoint}/api/chat',
                json=payload,
                timeout=30
            )
            resp.raise_for_status()
            result = resp.json()
            content = result.get('message', {}).get('content', '{}')
            
            # Clean up markdown formatting
            content = re.sub(r'```json\s*', '', content)
            content = re.sub(r'```\s*$', '', content)
            content = content.strip()
            
            return json.loads(content)
            
        except requests.RequestException as e:
            self.get_logger().error(f'Ollama request failed: {e}')
            return self._parse_with_regex(command)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'LLM output not valid JSON: {e}')
            return self._parse_with_regex(command)

    def _parse_with_openai(self, command: str) -> dict:
        """Call OpenAI API for parsing (requires OPENAI_API_KEY env var)."""
        import os
        import requests
        
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY not set, falling back to regex')
            return self._parse_with_regex(command)
        
        messages = get_mission_prompt(command)
        
        try:
            resp = requests.post(
                'https://api.openai.com/v1/chat/completions',
                headers={'Authorization': f'Bearer {api_key}'},
                json={
                    'model': self._llm_model or 'gpt-4o-mini',
                    'messages': messages,
                    'temperature': 0.1,
                },
                timeout=30
            )
            resp.raise_for_status()
            result = resp.json()
            content = result['choices'][0]['message']['content']
            
            content = re.sub(r'```json\s*', '', content)
            content = re.sub(r'```\s*$', '', content)
            
            return json.loads(content)
            
        except Exception as e:
            self.get_logger().error(f'OpenAI request failed: {e}')
            return self._parse_with_regex(command)

    def _parse_with_regex(self, command: str) -> dict:
        """Fallback regex-based parsing with fixed ordinal handling."""
        goals = []
        
        # Fixed ordinal number parsing
        word_to_num = {
            'first': 1, 'second': 2, 'third': 3, 'fourth': 4, 'fifth': 5,
            '1st': 1, '2nd': 2, '3rd': 3, '4th': 4, '5th': 5,
        }
        
        floor_match = re.search(
            r'(first|second|third|fourth|fifth|\d+(?:st|nd|rd|th)?)\s+floor|floor\s+(\d+)',
            command,
            re.IGNORECASE
        )
        
        if floor_match:
            if floor_match.group(1):
                word = floor_match.group(1).lower()
                floor_num = word_to_num.get(word)
                if floor_num is None:
                    # Extract digit from "3rd", "4th", etc.
                    digit_match = re.search(r'\d+', word)
                    floor_num = int(digit_match.group()) if digit_match else 1
            elif floor_match.group(2):
                floor_num = int(floor_match.group(2))
            else:
                floor_num = 1
            
            goals.append({'type': 'NavigateToFloor', 'floor': floor_num})
        
        # Inspect area
        inspect_match = re.search(r'inspect|check|scan', command, re.I)
        if inspect_match:
            goals.append({'type': 'InspectArea', 'area': 'current'})
        
        # Avoid obstacle
        avoid_match = re.search(r'avoid\s+(\w+)', command, re.I)
        if avoid_match:
            goals.append({'type': 'AvoidObstacle', 'obstacle_type': avoid_match.group(1)})
        
        # Land command
        land_match = re.search(r'land\s+(?:at|on)', command, re.I)
        if land_match:
            goals.append({'type': 'LandAt', 'position': [0.0, 0.0, 0.0]})
        
        # Report command
        report_match = re.search(r'report\s+(\w+)', command, re.I)
        if report_match:
            goals.append({'type': 'Report', 'data': report_match.group(1)})
        
        # Default fallback
        if not goals:
            goals.append({'type': 'NavigateTo', 'target': 'unknown', 'position': [0.0, 0.0, 1.0]})
        
        return {'goals': goals}

    def _publish_goals(self, goals: dict):
        """Publish parsed goals."""
        msg = String()
        msg.data = json.dumps(goals)
        self.goals_pub.publish(msg)
        self.get_logger().info(f'Published {len(goals.get("goals", []))} goals')


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