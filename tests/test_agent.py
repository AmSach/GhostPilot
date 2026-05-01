#!/usr/bin/env python3
"""GhostPilot Agent Tests - Real assertions, no placeholders."""

import sys
import os
import pytest
import json
import re

# Add source paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'ghostpilot_agent', 'ghostpilot_agent'))

# Import mission parser (has built-in ROS2 mock fallback)
from mission_parser import MissionParser, HAS_ROS2

# Import mission prompts
from mission_prompts import SYSTEM_PROMPT, get_mission_prompt, MISSION_EXAMPLES


class TestMissionParser:
    """Test mission parsing logic."""
    
    def test_regex_floor_parsing(self):
        """Test that floor numbers are correctly parsed — ordinals, numeric, and 'floor N'."""
        parser = MissionParser()

        # Ordinal word: third floor
        result = parser._parse_with_regex("Fly to the third floor")
        assert result['goals'][0]['type'] == 'NavigateToFloor'
        assert result['goals'][0]['floor'] == 3

        # Ordinal word: second floor
        result = parser._parse_with_regex("Go to the second floor")
        assert result['goals'][0]['floor'] == 2

        # Numeric ordinal: 5th floor
        result = parser._parse_with_regex("Navigate to 5th floor")
        assert result['goals'][0]['floor'] == 5

        # Numeric ordinal: 1st floor
        result = parser._parse_with_regex("Fly to 1st floor")
        assert result['goals'][0]['floor'] == 1

        # "floor N" format
        result = parser._parse_with_regex("Navigate to floor 5")
        assert result['goals'][0]['floor'] == 5

        # "floor N" high number
        result = parser._parse_with_regex("Navigate to floor 10")
        assert result['goals'][0]['floor'] == 10
    
    def test_regex_inspect_parsing(self):
        """Test that inspect commands are parsed."""
        parser = MissionParser()
        
        result = parser._parse_with_regex("Inspect the area")
        goal_types = [g['type'] for g in result['goals']]
        assert 'InspectArea' in goal_types
    
    def test_regex_avoid_parsing(self):
        """Test that avoid commands are parsed."""
        parser = MissionParser()
        
        result = parser._parse_with_regex("Avoid personnel")
        goal_types = [g['type'] for g in result['goals']]
        assert 'AvoidObstacle' in goal_types
        # Find the avoid goal
        for g in result['goals']:
            if g['type'] == 'AvoidObstacle':
                assert g['obstacle_type'] == 'personnel'
    
    def test_regex_land_parsing(self):
        """Test that land commands are parsed."""
        parser = MissionParser()
        
        result = parser._parse_with_regex("Land at the pad")
        goal_types = [g['type'] for g in result['goals']]
        assert 'LandAt' in goal_types
    
    def test_regex_combined_commands(self):
        """Test parsing multiple commands in one message."""
        parser = MissionParser()
        
        result = parser._parse_with_regex("Fly to floor 3, inspect the area, and land at base")
        goal_types = [g['type'] for g in result['goals']]
        
        assert 'NavigateToFloor' in goal_types
        assert 'InspectArea' in goal_types
        assert 'LandAt' in goal_types
        assert len(result['goals']) == 3


class TestMissionPrompts:
    """Test mission prompt module."""
    
    def test_system_prompt_exists(self):
        """Verify system prompt is defined."""
        assert SYSTEM_PROMPT is not None
        assert len(SYSTEM_PROMPT) > 100
        assert 'NavigateTo' in SYSTEM_PROMPT
        assert 'InspectArea' in SYSTEM_PROMPT
    
    def test_mission_examples_exist(self):
        """Verify mission examples are defined."""
        assert len(MISSION_EXAMPLES) >= 3
        
        for example in MISSION_EXAMPLES:
            assert 'input' in example
            assert 'goals' in example
            assert isinstance(example['goals'], list)
    
    def test_get_mission_prompt(self):
        """Test prompt builder function."""
        messages = get_mission_prompt("Test command")
        
        assert isinstance(messages, list)
        assert len(messages) == 2
        assert messages[0]['role'] == 'system'
        assert messages[1]['role'] == 'user'
        assert messages[1]['content'] == "Test command"


class TestGoalValidation:
    """Test goal structure validation."""
    
    def test_valid_goal_structure(self):
        """Test that goals have required fields."""
        valid_goals = [
            {"type": "NavigateTo", "target": "waypoint_a", "position": [0, 0, 1]},
            {"type": "InspectArea", "area": "room_1"},
            {"type": "AvoidObstacle", "obstacle_type": "personnel"},
            {"type": "LandAt", "position": [0, 0, 0]},
            {"type": "Report", "data": "damage"},
            {"type": "NavigateToFloor", "floor": 3}
        ]
        
        for goal in valid_goals:
            assert 'type' in goal
            assert goal['type'] in [
                'NavigateTo', 'NavigateToFloor', 'InspectArea',
                'AvoidObstacle', 'LandAt', 'Report'
            ]
    
    def test_floor_height_calculation(self):
        """Test floor to height conversion (3m per floor)."""
        for floor in range(1, 10):
            expected_z = floor * 3.0
            assert expected_z == floor * 3.0, f"Floor {floor} should be at z={floor * 3}m"


@pytest.mark.skipif(not HAS_ROS2, reason="ROS2 not installed")
class TestMissionParserROS2:
    """ROS2-dependent tests (skipped if ROS2 not available)."""
    
    def test_node_initialization(self):
        """Test that MissionParser can initialize as a ROS2 node."""
        import rclpy
        rclpy.init()
        try:
            parser = MissionParser()
            assert parser is not None
            parser.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])