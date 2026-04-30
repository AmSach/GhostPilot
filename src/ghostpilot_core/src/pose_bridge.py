#!/usr/bin/env python3
"""
Pose Bridge - Connects VINS-Mono SLAM to Nav2 Localization

Converts SLAM odometry into map localization that Nav2 can use.
Handles frame transformations and publishes filtered odometry.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf_transformations
import numpy as np


class PoseBridge(Node):
    """Bridge between VINS-Mono SLAM and Nav2 localization."""
    
    def __init__(self):
        super().__init__('ghostpilot_pose_bridge')
        
        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_filtered_odom', True)
        
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_filtered = self.get_parameter('publish_filtered_odom').value
        
        # TF buffers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Quality of service for reliable delivery
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.create_subscription(
            Odometry,
            '/vins/odometry',
            self.slam_odom_callback,
            qos
        )
        
        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos
        )
        
        # Publishers
        self.filtered_odom_pub = self.create_publisher(
            Odometry,
            '/odometry/filtered',
            qos
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # State
        self.last_imu = None
        self.last_slam_odom = None
        
        self.get_logger().info('Pose bridge started - VINS → Nav2')
    
    def slam_odom_callback(self, msg: Odometry):
        """Handle incoming VINS odometry."""
        self.last_slam_odom = msg
        
        if self.publish_filtered:
            # Publish filtered odometry for Nav2
            filtered = Odometry()
            filtered.header = msg.header
            filtered.header.frame_id = self.map_frame
            filtered.child_frame_id = self.base_frame
            filtered.pose = msg.pose
            filtered.twist = msg.twist
            
            self.filtered_odom_pub.publish(filtered)
        
        # Publish map->odom transform (SLAM provides this)
        self.publish_map_odom_transform(msg)
        
        # Estimate base_link transform from odom using IMU
        self.publish_odom_base_transform(msg)
    
    def imu_callback(self, msg: Imu):
        """Handle IMU data for dead reckoning."""
        self.last_imu = msg
    
    def publish_map_odom_transform(self, odom: Odometry):
        """Publish map → odom transform from SLAM."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odom_base_transform(self, odom: Odometry):
        """Publish odom → base_link transform (dead reckoning)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Simple dead reckoning from twist velocity
        if self.last_imu and odom.twist.twist.linear.x > 0.01:
            # Approximate motion based on velocity
            dt = 0.1  # Assume 10Hz
            t.transform.translation.x = odom.twist.twist.linear.x * dt
            t.transform.translation.y = odom.twist.twist.linear.y * dt
            t.transform.translation.z = 0.0
            
            # Use IMU orientation
            t.transform.rotation = self.last_imu.orientation
        else:
            # At rest - zero transform
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pose bridge')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()