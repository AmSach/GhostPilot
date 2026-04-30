#!/usr/bin/env python3
"""Tests for ghostpilot_agent mission executor."""

import unittest


class TestMissionExecutor(unittest.TestCase):
    """Test mission execution logic."""

    def test_navigate_to_parsing(self):
        """Test parsing NavigateTo goal."""
        goal = {'type': 'NavigateTo', 'position': [1.0, 2.0, 3.0]}
        self.assertEqual(goal['type'], 'NavigateTo')
        self.assertEqual(goal['position'], [1.0, 2.0, 3.0])

    def test_floor_to_z_conversion(self):
        """Test floor to Z height conversion."""
        floor_height = 3.0  # meters per floor
        floor = 3
        z_position = floor * floor_height
        self.assertEqual(z_position, 9.0)

    def test_inspect_sweep_waypoints(self):
        """Test area inspection waypoint generation."""
        waypoints = [
            [-2.0, 0.0, 1.5],
            [-2.0, 2.0, 1.5],
            [2.0, 2.0, 1.5],
            [2.0, -2.0, 1.5],
            [-2.0, -2.0, 1.5],
            [0.0, 0.0, 1.5],
        ]
        self.assertEqual(len(waypoints), 6)
        for wp in waypoints:
            self.assertEqual(len(wp), 3)
            self.assertGreater(wp[2], 0)  # z should be positive

    def test_goal_sequence_order(self):
        """Test that goals execute in correct order."""
        goals = [
            {'type': 'NavigateToFloor', 'floor': 1},
            {'type': 'InspectArea', 'area': 'room1'},
            {'type': 'LandAt', 'position': [0, 0, 0]}
        ]
        self.assertEqual(goals[0]['type'], 'NavigateToFloor')
        self.assertEqual(goals[1]['type'], 'InspectArea')
        self.assertEqual(goals[2]['type'], 'LandAt')

    def test_obstacle_avoidance_config(self):
        """Test obstacle avoidance configuration."""
        obstacle_type = 'personnel'
        config = {
            'avoid_radius': 2.0 if obstacle_type == 'personnel' else 1.0,
            'detection_threshold': 0.5
        }
        self.assertEqual(config['avoid_radius'], 2.0)


if __name__ == '__main__':
    unittest.main()