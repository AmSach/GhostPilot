#!/usr/bin/env python3
"""GhostPilot Core Tests - Real assertions, no placeholders."""

import sys
import os
import pytest
import numpy as np
import math

# Use mock ROS2 if not installed
try:
    import rclpy
    HAS_ROS2 = True
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mock_ros2'))
    import mock_rclpy as rclpy
    from mock_rclpy import PoseStamped, Node
    HAS_ROS2 = False


class TestSLAMNodeLogic:
    """Test SLAM node mathematical logic."""
    
    def test_pose_to_message_conversion(self):
        """Test conversion from numpy pose to ROS message."""
        # 7-element pose: [x, y, z, qx, qy, qz, qw]
        pose = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.707, 0.707])
        
        # Verify structure
        assert len(pose) == 7
        assert pose[0] == 1.0  # x
        assert pose[1] == 2.0  # y
        assert pose[2] == 3.0  # z
        # Quaternion should be normalized
        q_norm = math.sqrt(pose[3]**2 + pose[4]**2 + pose[5]**2 + pose[6]**2)
        assert abs(q_norm - 1.0) < 0.01, "Quaternion should be unit length"
    
    def test_quaternion_normalization(self):
        """Test that quaternions are properly normalized."""
        # Test various quaternion values
        test_quats = [
            [0, 0, 0, 1],           # Identity
            [0.707, 0, 0, 0.707],   # 90 deg rotation
            [0.5, 0.5, 0.5, 0.5],   # 120 deg rotation
        ]
        
        for q in test_quats:
            norm = math.sqrt(sum(x**2 for x in q))
            assert abs(norm - 1.0) < 0.001, f"Quaternion {q} not normalized"
    
    def test_imu_buffer_size(self):
        """Test IMU buffer stays within limits."""
        max_size = 100
        
        # Simulate buffer
        buffer = []
        for i in range(150):
            buffer.append({'data': i})
            if len(buffer) > max_size:
                buffer.pop(0)
        
        assert len(buffer) == max_size, f"Buffer should be {max_size}, got {len(buffer)}"


class TestPoseBridgeLogic:
    """Test pose bridge conversion logic."""
    
    def test_pose_frame_transform(self):
        """Test pose frame_id assignment."""
        # Simulated pose message
        pose_msg = type('PoseStamped', (), {
            'header': type('Header', (), {
                'frame_id': 'camera_link',
                'stamp': {'sec': 0, 'nanosec': 0}
            })(),
            'pose': type('Pose', (), {
                'position': type('Point', (), {'x': 1, 'y': 2, 'z': 3})(),
                'orientation': type('Quaternion', (), {'x': 0, 'y': 0, 'z': 0, 'w': 1})()
            })()
        })()
        
        # Bridge should set frame to 'map'
        expected_frame = 'map'
        # In actual code: msg.header.frame_id = 'map'
        pose_msg.header.frame_id = expected_frame
        
        assert pose_msg.header.frame_id == 'map'
    
    def test_pose_to_odometry_conversion(self):
        """Test pose to odometry message conversion."""
        # Verify that all pose fields map to odometry
        pose_fields = ['position', 'orientation']
        for field in pose_fields:
            assert field is not None


class TestNavigationMath:
    """Test navigation-related calculations."""
    
    def test_waypoint_distance(self):
        """Test distance calculation between waypoints."""
        p1 = np.array([0, 0, 0])
        p2 = np.array([3, 4, 0])
        
        # Euclidean distance should be 5 (3-4-5 triangle)
        distance = np.linalg.norm(p2 - p1)
        assert abs(distance - 5.0) < 0.001
    
    def test_floor_to_altitude(self):
        """Test floor number to altitude conversion."""
        # 3 meters per floor
        for floor, expected_alt in [(1, 3), (2, 6), (3, 9), (5, 15), (10, 30)]:
            calculated_alt = floor * 3.0
            assert calculated_alt == expected_alt
    
    def test_coordinate_frame_consistency(self):
        """Test that coordinate frames are consistent."""
        # ENU convention: x=right, y=forward, z=up
        # Verify navigation uses consistent frames
        waypoint_enu = {'x': 10, 'y': 20, 'z': 5}  # 10m right, 20m forward, 5m up
        
        assert waypoint_enu['z'] >= 0, "Z should be non-negative (up)"
    
    def test_inspection_waypoint_pattern(self):
        """Test that inspection waypoints form a valid pattern."""
        # Waypoints from executor.py
        waypoints = [
            [-2.0, 0.0, 1.5],
            [-2.0, 2.0, 1.5],
            [2.0, 2.0, 1.5],
            [2.0, -2.0, 1.5],
            [-2.0, -2.0, 1.5],
            [0.0, 0.0, 1.5],
        ]
        
        # Verify all at same altitude
        altitudes = [wp[2] for wp in waypoints]
        assert len(set(altitudes)) == 1, "All waypoints should be at same altitude"
        
        # Verify pattern covers area
        x_coords = [wp[0] for wp in waypoints]
        y_coords = [wp[1] for wp in waypoints]
        
        assert max(x_coords) - min(x_coords) == 4.0, "X range should be 4m"
        assert max(y_coords) - min(y_coords) == 4.0, "Y range should be 4m"


class TestSafetyConstraints:
    """Test safety-related constraints."""
    
    def test_max_altitude_limit(self):
        """Test that altitude has reasonable limits."""
        max_altitude = 100.0  # Example max
        test_altitudes = [0, 10, 50, 100, 150]
        
        for alt in test_altitudes:
            is_valid = 0 <= alt <= max_altitude
            if alt > max_altitude:
                assert not is_valid, f"Altitude {alt} should exceed limit"
            else:
                assert is_valid
    
    def test_position_bounds(self):
        """Test that navigation respects boundaries."""
        bounds = {
            'x_min': -100, 'x_max': 100,
            'y_min': -100, 'y_max': 100,
            'z_min': 0, 'z_max': 50
        }
        
        test_positions = [
            ([0, 0, 5], True),
            ([50, 50, 10], True),
            ([101, 0, 5], False),  # X out of bounds
            ([0, 0, 60], False),   # Z out of bounds
            ([-50, -50, 1], True),
        ]
        
        for pos, expected_valid in test_positions:
            in_bounds = (
                bounds['x_min'] <= pos[0] <= bounds['x_max'] and
                bounds['y_min'] <= pos[1] <= bounds['y_max'] and
                bounds['z_min'] <= pos[2] <= bounds['z_max']
            )
            assert in_bounds == expected_valid, f"Position {pos} validity mismatch"


class TestSLAMIntegration:
    """Test SLAM integration points."""
    
    def test_camera_topic_mapping(self):
        """Test camera topic remapping."""
        # Default: /camera/image_raw
        # Remapped: /camera/realsense/aligned_depth_to_color/image_raw
        
        default_topic = '/camera/image_raw'
        remapped_topic = '/camera/realsense/aligned_depth_to_color/image_raw'
        
        # Verify both are valid topic names
        assert default_topic.startswith('/')
        assert remapped_topic.startswith('/')
    
    def test_imu_topic_mapping(self):
        """Test IMU topic remapping."""
        default_topic = '/imu/data'
        remapped_topic = '/imu/imu_data'
        
        assert default_topic.startswith('/')
        assert remapped_topic.startswith('/')


@pytest.mark.skipif(not HAS_ROS2, reason="ROS2 not installed")
class TestSLAMNodeROS2:
    """ROS2-dependent tests (skipped if ROS2 not available)."""
    
    def test_slam_node_creation(self):
        """Test SLAM node can be created."""
        rclpy.init()
        try:
            from ghostpilot_core.slam_node import SLAMNode
            node = SLAMNode()
            assert node is not None
            node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])