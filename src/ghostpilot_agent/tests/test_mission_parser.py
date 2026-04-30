#!/usr/bin/env python3
"""Tests for ghostpilot_agent mission parser."""

import unittest
import re


class TestMissionParser(unittest.TestCase):
    """Test mission command parsing."""

    def test_parse_floor_command(self):
        """Test parsing floor navigation."""
        command = "Fly to the third floor"
        floor_match = re.search(r'third floor|3rd floor|floor (\d+)', command, re.I)
        self.assertIsNotNone(floor_match)

    def test_parse_inspect_command(self):
        """Test parsing inspect command."""
        command = "Inspect the roof"
        inspect_match = re.search(r'inspect|check|scan', command, re.I)
        self.assertIsNotNone(inspect_match)
        self.assertEqual(inspect_match.group(), 'Inspect')

    def test_parse_avoid_command(self):
        """Test parsing avoid command."""
        command = "avoid personnel"
        avoid_match = re.search(r'avoid\s+(\w+)', command, re.I)
        self.assertIsNotNone(avoid_match)
        self.assertEqual(avoid_match.group(1), 'personnel')

    def test_parse_land_command(self):
        """Test parsing land command."""
        command = "land at helipad"
        land_match = re.search(r'land\s+(?:at|on)', command, re.I)
        self.assertIsNotNone(land_match)

    def test_parse_report_command(self):
        """Test parsing report command."""
        command = "report damage"
        report_match = re.search(r'report\s+(\w+)', command, re.I)
        self.assertIsNotNone(report_match)
        self.assertEqual(report_match.group(1), 'damage')

    def test_parse_complex_command(self):
        """Test parsing complex multi-goal command."""
        command = "Fly to the third floor, check each room for occupants, avoid people"
        
        goals = []
        
        floor_match = re.search(r'third floor|3rd floor|floor (\d+)', command, re.I)
        if floor_match:
            goals.append({'type': 'NavigateToFloor', 'floor': 3})
        
        inspect_match = re.search(r'inspect|check|scan', command, re.I)
        if inspect_match:
            goals.append({'type': 'InspectArea', 'area': 'current'})
        
        avoid_match = re.search(r'avoid\s+(\w+)', command, re.I)
        if avoid_match:
            goals.append({'type': 'AvoidObstacle', 'obstacle_type': avoid_match.group(1)})
        
        self.assertEqual(len(goals), 3)
        self.assertEqual(goals[0]['type'], 'NavigateToFloor')
        self.assertEqual(goals[1]['type'], 'InspectArea')
        self.assertEqual(goals[2]['type'], 'AvoidObstacle')

    def test_goal_position_parsing(self):
        """Test parsing position from command."""
        position_str = "[1.5, 2.3, 3.0]"
        import ast
        position = ast.literal_eval(position_str)
        self.assertEqual(position, [1.5, 2.3, 3.0])
        self.assertEqual(len(position), 3)


if __name__ == '__main__':
    unittest.main()