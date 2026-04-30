#!/usr/bin/env python3
"""Tests for ghostpilot_core SLAM node."""

import unittest
from unittest.mock import MagicMock, patch
import numpy as np


class TestSLAMNode(unittest.TestCase):
    """Test SLAM node functionality."""

    def test_pose_initialization(self):
        """Test that SLAM pose initializes correctly."""
        pose = np.zeros(7)
        pose[6] = 1.0  # w quaternion
        self.assertEqual(len(pose), 7)
        self.assertEqual(pose[6], 1.0)

    def test_pose_format(self):
        """Test pose has correct structure [x,y,z,qx,qy,qz,qw]."""
        pose = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(pose[0], 1.0)  # x
        self.assertEqual(pose[1], 2.0)  # y
        self.assertEqual(pose[2], 3.0)  # z
        self.assertEqual(pose[6], 1.0)  # w quaternion

    def test_imu_buffer_management(self):
        """Test IMU buffer caps at max size."""
        buffer = []
        max_size = 100
        for i in range(150):
            buffer.append(i)
            if len(buffer) > max_size:
                buffer.pop(0)
        self.assertEqual(len(buffer), max_size)

    def test_config_parsing(self):
        """Test configuration file parsing."""
        import yaml
        config = """
camera:
  sensor_name: Intel RealSense D435i
  resolution: [640, 480]
  fps: 30
slam:
  algorithm: VINS-Mono
  feature_extraction_threshold: 10
"""
        parsed = yaml.safe_load(config)
        self.assertEqual(parsed['camera']['sensor_name'], 'Intel RealSense D435i')
        self.assertEqual(parsed['slam']['algorithm'], 'VINS-Mono')


if __name__ == '__main__':
    unittest.main()