#!/usr/bin/env python3
"""
VINS-Mono SLAM Node Wrapper

Wraps VINS-Mono odometry output into GhostPilot navigation stack.
Publishes odometry in Nav2-compatible format.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import math


class SLAMNode(Node):
    """VINS-Mono SLAM node for GhostPilot."""
    
    def __init__(self):
        super().__init__('ghostpilot_slam_node')
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pub_rate', 10.0)
        
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pub_rate = self.get_parameter('pub_rate').value
        
        # State
        self.last_odom = None
        self.odom_counter = 0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vins/odometry', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.create_subscription(
            Odometry,
            '/vins/odometry_raw',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('GhostPilot SLAM node started')
    
    def odom_callback(self, msg: Odometry):
        """Process incoming VINS odometry."""
        self.last_odom = msg
        self.odom_counter += 1
        
        # Republish with correct frame names
        out_odom = Odometry()
        out_odom.header = msg.header
        out_odom.header.frame_id = self.odom_frame
        out_odom.child_frame_id = self.base_frame
        out_odom.pose = msg.pose
        out_odom.twist = msg.twist
        
        self.odom_pub.publish(out_odom)
        
        # Publish TF
        self.publish_transform(msg)
        
        # Log periodically
        if self.odom_counter % 100 == 0:
            pos = msg.pose.pose.position
            self.get_logger().info(
                f'SLAM pose: x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}'
            )
    
    def publish_transform(self, odom: Odometry):
        """Publish map->odom transform."""
        t = TransformStamped()
        t.header = odom.header
        t.header.frame_id = 'map'
        t.child_frame_id = self.odom_frame
        
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down SLAM node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()