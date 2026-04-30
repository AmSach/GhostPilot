#!/usr/bin/env python3
"""Bridge between SLAM pose and Nav2 localization."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
# NOTE: message_filters removed - was unused import


class PoseBridge(Node):
    """Converts SLAM pose output to Nav2 localization input."""

    def __init__(self):
        super().__init__('pose_bridge')
        
        self.declare_parameter('slam_pose_topic', '/ghostpilot/pose')
        self.declare_parameter('nav2_pose_topic', '/localization_pose')
        self.declare_parameter('nav2_odom_topic', '/odometry/localized')
        
        slam_topic = self.get_parameter('slam_pose_topic').value
        nav2_pose_topic = self.get_parameter('nav2_pose_topic').value
        nav2_odom_topic = self.get_parameter('nav2_odom_topic').value
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        self.slam_sub = self.create_subscription(
            PoseStamped, slam_topic, self._slam_callback, qos
        )
        
        self.pose_pub = self.create_publisher(PoseStamped, nav2_pose_topic, qos)
        self.odom_pub = self.create_publisher(Odometry, nav2_odom_topic, qos)
        
        self.get_logger().info('Pose bridge initialized')

    def _slam_callback(self, msg: PoseStamped):
        """Forward pose to Nav2 with appropriate framing."""
        msg.header.frame_id = 'map'
        self.pose_pub.publish(msg)
        
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = msg.pose
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()