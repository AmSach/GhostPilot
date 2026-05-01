#!/usr/bin/env python3
"""
PoseBridge — converts SLAM pose to Nav2 localisation input.

Enhancements over the stub:
  - Velocity estimated from successive poses (finite difference)
  - Diagonal pose covariance forwarded to Nav2
  - Publishes tf2 map→base_link transform so rviz2 / Nav2 can use it
  - Sanity-checks: rejects poses with NaN/Inf, rejects jumps > 5 m/frame
"""

import os, sys, math
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster
    HAS_ROS2 = True
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..','..','..','..','mock_ros2'))
    import mock_rclpy as rclpy
    from mock_rclpy import Node
    HAS_ROS2 = False


class PoseBridge(Node if HAS_ROS2 else object):
    """Converts SLAM PoseStamped → Nav2 localisation topics + TF."""

    MAX_JUMP_M   = 5.0    # reject jumps larger than this (m)
    MAX_JUMP_S   = 1.0    # over this time interval (s)

    def __init__(self):
        if HAS_ROS2:
            super().__init__('pose_bridge')

        self._params = {
            'slam_pose_topic':  '/ghostpilot/pose',
            'nav2_pose_topic':  '/localization_pose',
            'nav2_odom_topic':  '/odometry/localized',
            'base_frame':       'base_link',
            'map_frame':        'map',
        }
        if HAS_ROS2:
            for k, v in self._params.items():
                self.declare_parameter(k, v)
                self._params[k] = self.get_parameter(k).value

            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.slam_sub = self.create_subscription(
                PoseStamped, self._params['slam_pose_topic'],
                self._slam_callback, qos)
            self.pose_pub = self.create_publisher(
                PoseStamped, self._params['nav2_pose_topic'], qos)
            self.odom_pub = self.create_publisher(
                Odometry, self._params['nav2_odom_topic'], qos)
            self._tf_broadcaster = TransformBroadcaster(self)

        # Velocity estimation state
        self._prev_pos  = None
        self._prev_time = None
        self._velocity  = np.zeros(3)

        self._pose_count  = 0
        self._reject_count = 0

        self._log('Pose bridge initialised')

    # ------------------------------------------------------------------ #

    def _slam_callback(self, msg):
        """Validate, estimate velocity, forward to Nav2, broadcast TF."""
        pos = np.array([msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])

        # Sanity: reject NaN / Inf
        if not np.all(np.isfinite(pos)):
            self._reject_count += 1
            self._log_warn(f'Rejected non-finite pose (total rejects: {self._reject_count})')
            return

        # Sanity: reject impossible jumps
        t = self._stamp_to_sec(msg.header.stamp)
        if self._prev_pos is not None and self._prev_time is not None:
            dt = t - self._prev_time
            if 0 < dt <= self.MAX_JUMP_S:
                jump = float(np.linalg.norm(pos - self._prev_pos))
                if jump > self.MAX_JUMP_M:
                    self._reject_count += 1
                    self._log_warn(
                        f'Rejected {jump:.2f}m jump in {dt:.3f}s '
                        f'(total rejects: {self._reject_count})')
                    return
                self._velocity = (pos - self._prev_pos) / dt

        self._prev_pos  = pos.copy()
        self._prev_time = t
        self._pose_count += 1

        if HAS_ROS2:
            self._publish_pose(msg)
            self._publish_odometry(msg)
            self._broadcast_tf(msg)

    def _publish_pose(self, msg):
        out = PoseStamped()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = self._params['map_frame']
        out.pose = msg.pose
        self.pose_pub.publish(out)

    def _publish_odometry(self, msg):
        odom = Odometry()
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = self._params['map_frame']
        odom.child_frame_id  = self._params['base_frame']
        odom.pose.pose = msg.pose

        # Diagonal velocity from finite difference
        odom.twist.twist.linear.x = float(self._velocity[0])
        odom.twist.twist.linear.y = float(self._velocity[1])
        odom.twist.twist.linear.z = float(self._velocity[2])

        # Conservative diagonal covariance (tuneable)
        for i in range(6):
            odom.pose.covariance[i * 7] = 0.1
        for i in range(6):
            odom.twist.covariance[i * 7] = 0.5

        self.odom_pub.publish(odom)

    def _broadcast_tf(self, msg):
        tf = TransformStamped()
        tf.header.stamp    = msg.header.stamp
        tf.header.frame_id = self._params['map_frame']
        tf.child_frame_id  = self._params['base_frame']
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------------------------ #

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        if hasattr(stamp, 'sec'):
            return stamp.sec + stamp.nanosec * 1e-9
        return float(stamp)

    def _log(self, msg):
        (self.get_logger().info if HAS_ROS2 else print)(
            msg if HAS_ROS2 else f'[pose_bridge] {msg}')

    def _log_warn(self, msg):
        (self.get_logger().warn if HAS_ROS2 else print)(
            msg if HAS_ROS2 else f'[pose_bridge WARN] {msg}')


def main():
    if not HAS_ROS2:
        print('ROS2 not available.')
        return
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
