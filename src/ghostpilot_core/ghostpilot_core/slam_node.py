#!/usr/bin/env python3
"""VINS-Mono SLAM wrapper node for GhostPilot."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
import numpy as np


class SLAMNode(Node):
    """Visual-Inertial SLAM node wrapping VINS-Mono or ORB-SLAM3."""

    def __init__(self):
        super().__init__('slam_node')
        
        self.declare_parameter('config_file', '')
        self.declare_parameter('slam_pose_topic', '/ghostpilot/pose')
        self.declare_parameter('odometry_topic', '/ghostpilot/odometry')
        
        slam_pose_topic = self.get_parameter('slam_pose_topic').value
        odometry_topic = self.get_parameter('odometry_topic').value
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, 100
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, slam_pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odometry_topic, 10)
        
        self._pose = None
        self._imu_buffer = []
        self._frame_count = 0
        self.get_logger().info('SLAM node initialized')

    def _image_callback(self, msg: Image):
        """Process camera frame through SLAM."""
        self._frame_count += 1
        # FIXED: Use info level for periodic logging (debug won't show by default)
        if self._frame_count % 30 == 0:  # Log every 30 frames
            self.get_logger().info(f'Processing frame {self._frame_count}')

    def _imu_callback(self, msg: Imu):
        """Buffer IMU measurements for SLAM fusion."""
        self._imu_buffer.append(msg)
        if len(self._imu_buffer) > 100:
            self._imu_buffer.pop(0)

    def _publish_pose(self, pose: np.ndarray, stamp):
        """Publish computed SLAM pose."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(pose[0])
        pose_msg.pose.position.y = float(pose[1])
        pose_msg.pose.position.z = float(pose[2])
        pose_msg.pose.orientation.x = float(pose[3])
        pose_msg.pose.orientation.y = float(pose[4])
        pose_msg.pose.orientation.z = float(pose[5])
        pose_msg.pose.orientation.w = float(pose[6])
        self.pose_pub.publish(pose_msg)


def main():
    rclpy.init()
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()