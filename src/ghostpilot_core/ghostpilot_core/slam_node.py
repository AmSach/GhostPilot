#!/usr/bin/env python3
"""
VINS-Mono SLAM wrapper node for GhostPilot.

Subscribes to /camera/image_raw and /imu/data, feeds both into VINSMono,
and publishes pose/odometry/path on every processed camera frame.
"""

import os, sys
import yaml
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image, Imu
    from nav_msgs.msg import Odometry, Path
    import cv2
    HAS_ROS2 = True
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..','..','..','..','mock_ros2'))
    import mock_rclpy as rclpy
    from mock_rclpy import Node
    HAS_ROS2 = False

_HERE = os.path.dirname(__file__)
sys.path.insert(0, _HERE)
from vins_mono import VINSMono


class SLAMNode(Node if HAS_ROS2 else object):
    """Visual-Inertial SLAM node — fully wired VINS-Mono pipeline."""

    def __init__(self):
        if HAS_ROS2:
            super().__init__('slam_node')

        self._params = {
            'config_file':      '',
            'slam_pose_topic':  '/ghostpilot/pose',
            'odometry_topic':   '/ghostpilot/odometry',
            'path_topic':       '/ghostpilot/path',
            'camera_topic':     '/camera/image_raw',
            'imu_topic':        '/imu/data',
            'publish_path':     True,
            'max_path_length':  500,
        }
        if HAS_ROS2:
            for k, v in self._params.items():
                self.declare_parameter(k, v)
                self._params[k] = self.get_parameter(k).value

        config = self._load_config(self._params['config_file'])
        self._vins = VINSMono(config)

        if HAS_ROS2:
            self.image_sub = self.create_subscription(
                Image, self._params['camera_topic'], self._image_callback, 10)
            self.imu_sub = self.create_subscription(
                Imu, self._params['imu_topic'], self._imu_callback, 100)
            self.pose_pub = self.create_publisher(
                PoseStamped, self._params['slam_pose_topic'], 10)
            self.odom_pub = self.create_publisher(
                Odometry, self._params['odometry_topic'], 10)
            self.path_pub = self.create_publisher(
                Path, self._params['path_topic'], 10)

        self._imu_buffer = []
        self._frame_count = 0
        self._path_poses  = []
        self._log('SLAM node initialised — VINSMono pipeline active')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _imu_callback(self, msg):
        """Buffer and forward IMU measurements to VINS pre-integrator."""
        self._imu_buffer.append(msg)
        if len(self._imu_buffer) > 100:
            self._imu_buffer.pop(0)

        acc  = np.array([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])
        self._vins.process_imu(acc, gyro, self._stamp_to_sec(msg.header.stamp))

    def _image_callback(self, msg):
        """Convert image → gray, run VINSMono, publish if initialised."""
        self._frame_count += 1
        gray = self._decode_image(msg)
        if gray is None:
            return

        t = self._stamp_to_sec(msg.header.stamp)
        pose_vec = self._vins.process_image(gray, t)   # [x,y,z,qx,qy,qz,qw] or None

        if pose_vec is None:
            if self._frame_count % 30 == 0:
                self._log(f'Initialising… frame {self._frame_count} '
                          f'(need 2 frames with ≥8 common features)')
            return

        stamp = msg.header.stamp
        self._publish_pose(pose_vec, stamp)
        self._publish_odometry(pose_vec, stamp)
        if self._params['publish_path']:
            self._publish_path(pose_vec, stamp)

        if self._frame_count % 30 == 0:
            p = pose_vec[:3]
            self._log(f'Frame {self._frame_count} | '
                      f'xyz=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}) | '
                      f'keyframes={self._vins._kf_count}')

    # ------------------------------------------------------------------ #
    #  Publishers                                                          #
    # ------------------------------------------------------------------ #

    def _publish_pose(self, v, stamp):
        if not HAS_ROS2:
            return
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x    = float(v[0])
        msg.pose.position.y    = float(v[1])
        msg.pose.position.z    = float(v[2])
        msg.pose.orientation.x = float(v[3])
        msg.pose.orientation.y = float(v[4])
        msg.pose.orientation.z = float(v[5])
        msg.pose.orientation.w = float(v[6])
        self.pose_pub.publish(msg)

    def _publish_odometry(self, v, stamp):
        if not HAS_ROS2:
            return
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x    = float(v[0])
        msg.pose.pose.position.y    = float(v[1])
        msg.pose.pose.position.z    = float(v[2])
        msg.pose.pose.orientation.x = float(v[3])
        msg.pose.pose.orientation.y = float(v[4])
        msg.pose.pose.orientation.z = float(v[5])
        msg.pose.pose.orientation.w = float(v[6])
        vel = self._vins.velocity
        msg.twist.twist.linear.x = float(vel[0])
        msg.twist.twist.linear.y = float(vel[1])
        msg.twist.twist.linear.z = float(vel[2])
        # Covariance from VINS marginalizer
        cov = np.diag(self._vins.marginalizer.H_prior)
        for i in range(min(6, len(cov))):
            msg.pose.covariance[i * 7] = float(1.0 / max(cov[i], 1e-6))
        self.odom_pub.publish(msg)

    def _publish_path(self, v, stamp):
        if not HAS_ROS2:
            return
        ps = PoseStamped()
        ps.header.stamp = stamp; ps.header.frame_id = 'map'
        ps.pose.position.x = float(v[0])
        ps.pose.position.y = float(v[1])
        ps.pose.position.z = float(v[2])
        self._path_poses.append(ps)
        if len(self._path_poses) > self._params['max_path_length']:
            self._path_poses.pop(0)
        path = Path()
        path.header.stamp = stamp; path.header.frame_id = 'map'
        path.poses = self._path_poses
        self.path_pub.publish(path)

    # ------------------------------------------------------------------ #
    #  Utilities                                                           #
    # ------------------------------------------------------------------ #

    def _decode_image(self, msg) -> 'np.ndarray | None':
        try:
            import cv2
            enc = getattr(msg, 'encoding', 'mono8')
            h, w = msg.height, msg.width
            data = bytes(msg.data)
            if enc in ('mono8', '8UC1'):
                return np.frombuffer(data, dtype=np.uint8).reshape(h, w)
            elif enc in ('bgr8', 'rgb8'):
                raw = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
                return cv2.cvtColor(raw, cv2.COLOR_RGB2GRAY if enc == 'rgb8'
                                    else cv2.COLOR_BGR2GRAY)
            elif enc in ('16UC1', 'mono16'):
                raw = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
                return (raw >> 8).astype(np.uint8)
            else:
                raw = np.frombuffer(data, dtype=np.uint8).reshape(h, w, -1)
                return cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self._log_warn(f'Image decode error: {e}')
            return None

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        if hasattr(stamp, 'sec'):
            return stamp.sec + stamp.nanosec * 1e-9
        return float(stamp)

    def _load_config(self, path: str) -> dict:
        if not path:
            path = os.path.join(os.path.dirname(__file__),
                                '..', 'config', 'vins_params.yaml')
        if os.path.isfile(path):
            with open(path) as f:
                cfg = yaml.safe_load(f)
                self._log(f'Loaded VINS config: {path}')
                return cfg or {}
        self._log_warn(f'Config not found at {path}, using defaults')
        return {}

    def _log(self, msg):
        (self.get_logger().info if HAS_ROS2 else print)(
            msg if HAS_ROS2 else f'[slam_node] {msg}')

    def _log_warn(self, msg):
        (self.get_logger().warn if HAS_ROS2 else print)(
            msg if HAS_ROS2 else f'[slam_node WARN] {msg}')


def main():
    if not HAS_ROS2:
        print('ROS2 not available — use simulate.py to test VINS headlessly.')
        return
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
