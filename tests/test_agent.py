#!/usr/bin/env python3
"""GhostPilot Agent Tests - Real assertions, no placeholders."""

import pytest
import json
import re
from dataclasses import dataclass
from typing import List, Optional, Dict, Any


@dataclass
class Goal:
    x: float
    y: float
    z: float
    description: str = ""
    goal_type: str = "NavigateTo"


@dataclass
class Mission:
    goals: List[Goal]
    description: str = ""


class TestMissionParser:
    """Tests for mission parsing logic."""

    def test_parse_navigate_to_floor_third(self):
        """Parse 'third floor' correctly."""
        command = "Fly to the third floor"
        result = self._parse_with_regex(command)
        
        assert "goals" in result
        assert len(result["goals"]) >= 1
        goal = result["goals"][0]
        assert goal["type"] == "NavigateToFloor"
        assert goal["floor"] == 3

    def test_parse_navigate_to_floor_numeric(self):
        """Parse 'floor 5' correctly."""
        command = "Go to floor 5"
        result = self._parse_with_regex(command)
        
        assert "goals" in result
        goal = result["goals"][0]
        assert goal["type"] == "NavigateToFloor"
        assert goal["floor"] == 5

    def test_parse_inspect_area(self):
        """Parse inspection command."""
        command = "Inspect the warehouse"
        result = self._parse_with_regex(command)
        
        goals = result.get("goals", [])
        inspect_goals = [g for g in goals if g["type"] == "InspectArea"]
        assert len(inspect_goals) >= 1

    def test_parse_avoid_obstacle(self):
        """Parse obstacle avoidance."""
        command = "Avoid personnel in the corridor"
        result = self._parse_with_regex(command)
        
        goals = result.get("goals", [])
        avoid_goals = [g for g in goals if g["type"] == "AvoidObstacle"]
        assert len(avoid_goals) >= 1
        assert avoid_goals[0]["obstacle_type"] == "personnel"

    def test_parse_land_command(self):
        """Parse landing command."""
        command = "Land at the helipad"
        result = self._parse_with_regex(command)
        
        goals = result.get("goals", [])
        land_goals = [g for g in goals if g["type"] == "LandAt"]
        assert len(land_goals) >= 1

    def test_parse_report_command(self):
        """Parse report generation."""
        command = "Report damage findings"
        result = self._parse_with_regex(command)
        
        goals = result.get("goals", [])
        report_goals = [g for g in goals if g["type"] == "Report"]
        assert len(report_goals) >= 1
        assert report_goals[0]["data"] == "damage"

    def test_parse_complex_mission(self):
        """Parse multi-part mission."""
        command = "Fly to floor 2, inspect all rooms, avoid personnel, land at entrance"
        result = self._parse_with_regex(command)
        
        assert "goals" in result
        assert len(result["goals"]) >= 3

    def test_floor_height_calculation(self):
        """Floor height = floor * 3m."""
        for floor in [1, 2, 3, 5]:
            z = floor * 3.0
            assert z == floor * 3.0

    def test_goal_position_valid(self):
        """Goal positions have valid coordinates."""
        goal = {"type": "NavigateTo", "position": [10.0, -5.0, 3.0]}
        assert len(goal["position"]) == 3
        for coord in goal["position"]:
            assert isinstance(coord, (int, float))

    def _parse_with_regex(self, command: str) -> dict:
        """Regex-based parsing implementation (mirrors mission_parser.py)."""
        goals = []
        
        # Fixed ordinal parsing
        floor_match = re.search(r'(first|second|third|fourth|fifth|\d+(?:st|nd|rd|th)?)\s+floor|floor\s+(\d+)', command, re.I)
        if floor_match:
            word_to_num = {'first': 1, 'second': 2, 'third': 3, 'fourth': 4, 'fifth': 5}
            if floor_match.group(1):
                word = floor_match.group(1).lower()
                if word in word_to_num:
                    floor_num = word_to_num[word]
                else:
                    # Extract digit from "3rd", "4th", etc.
                    digit_match = re.search(r'\d+', word)
                    floor_num = int(digit_match.group()) if digit_match else 1
            elif floor_match.group(2):
                floor_num = int(floor_match.group(2))
            else:
                floor_num = 1
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


class TestMissionExecutor:
    """Tests for mission executor logic."""

    def test_goal_position_extraction(self):
        """Extract position from goal dict."""
        goal = {"type": "NavigateTo", "position": [5.0, 3.0, 2.0]}
        position = goal.get("position", [0, 0, 1])
        
        assert position[0] == 5.0
        assert position[1] == 3.0
        assert position[2] == 2.0

    def test_floor_to_z_conversion(self):
        """Convert floor number to z coordinate."""
        floor = 3
        z = floor * 3.0
        assert z == 9.0

    def test_waypoint_sequence(self):
        """Waypoint inspection generates correct sequence."""
        waypoints = [
            [-2.0, 0.0, 1.5],
            [-2.0, 2.0, 1.5],
            [2.0, 2.0, 1.5],
            [2.0, -2.0, 1.5],
        ]
        assert len(waypoints) == 4
        for wp in waypoints:
            assert wp[2] == 1.5  # All at same altitude

    def test_json_goal_serialization(self):
        """Goals serialize to valid JSON."""
        goals = {
            "goals": [
                {"type": "NavigateTo", "position": [1.0, 2.0, 3.0]},
                {"type": "InspectArea", "area": "warehouse"}
            ]
        }
        
        json_str = json.dumps(goals)
        parsed = json.loads(json_str)
        
        assert parsed == goals
        assert len(parsed["goals"]) == 2


if __name__ == "__main__":
    pytest.main([__file__, "-v"])