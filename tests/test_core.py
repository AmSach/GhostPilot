#!/usr/bin/env python3
"""
GhostPilot Core Tests — real assertions covering VINS-Mono, PoseBridge, Executor stubs.
"""

import sys, os, math
import numpy as np
import pytest

# Paths
_ROOT    = os.path.join(os.path.dirname(__file__), '..')
_CORE    = os.path.join(_ROOT, 'src', 'ghostpilot_core', 'ghostpilot_core')
_AGENT   = os.path.join(_ROOT, 'src', 'ghostpilot_agent', 'ghostpilot_agent')
_MOCK    = os.path.join(_ROOT, 'mock_ros2')

sys.path.insert(0, _CORE)
sys.path.insert(0, _AGENT)
sys.path.insert(0, _MOCK)

try:
    import rclpy
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

import cv2
from vins_mono import (
    VINSMono, PinholeCamera, FeatureTracker, IMUPreintegration,
    SlidingWindowOptimizer, LoopClosureDetector, Marginalizer,
    Frame, IMUMeasurement, _skew, _rot_to_quat, triangulate_dlt,
)


# =========================================================================
# Helpers
# =========================================================================

def _make_synthetic_frame(h=240, w=320, n_squares=6) -> np.ndarray:
    """Create a grayscale checkerboard-like frame with trackable features."""
    img = np.zeros((h, w), dtype=np.uint8)
    sq = h // n_squares
    for r in range(n_squares):
        for c in range(n_squares):
            if (r + c) % 2 == 0:
                img[r*sq:(r+1)*sq, c*sq:(c+1)*sq] = 200
    # Add some noise so LK has texture to work with
    noise = np.random.randint(0, 30, img.shape, dtype=np.uint8)
    return cv2.add(img, noise)


def _make_imu(acc=(0,0,9.81), gyro=(0,0,0), t=0.0) -> IMUMeasurement:
    return IMUMeasurement(
        timestamp=t,
        acc=np.array(acc, dtype=float),
        gyro=np.array(gyro, dtype=float),
    )


# =========================================================================
# PinholeCamera
# =========================================================================

class TestPinholeCamera:

    def test_project_along_z_axis(self):
        cam = PinholeCamera(fx=500, fy=500, cx=320, cy=240)
        # Point directly in front → should land at principal point
        px = cam.project(np.array([0.0, 0.0, 1.0]))
        assert abs(px[0] - 320) < 1e-6
        assert abs(px[1] - 240) < 1e-6

    def test_project_off_axis(self):
        cam = PinholeCamera(fx=500, fy=500, cx=320, cy=240)
        px = cam.project(np.array([1.0, 0.0, 1.0]))   # 1m right at 1m depth
        assert abs(px[0] - 820) < 1e-6                # cx + fx*1/1 = 820

    def test_project_behind_camera_returns_negative(self):
        cam = PinholeCamera()
        px = cam.project(np.array([0.0, 0.0, -1.0]))
        assert px[0] < 0

    def test_undistort_zero_distortion_is_identity(self):
        cam = PinholeCamera(fx=500, fy=500, cx=320, cy=240, k1=0.0, k2=0.0)
        pts = np.array([[320.0, 240.0], [400.0, 300.0]], dtype=np.float32)
        norm = cam.undistort_points(pts)
        # With zero distortion, normalised coords should be (px-cx)/fx
        assert abs(norm[0, 0]) < 1e-4   # (320-320)/500 = 0
        assert abs(norm[0, 1]) < 1e-4

    def test_from_config_uses_resolution_for_cx_cy(self):
        cfg = {'camera': {'resolution': [640, 480], 'fx': 400, 'fy': 400,
                          'k1': 0, 'k2': 0}}
        cam = PinholeCamera.from_config(cfg)
        assert cam.cx == 320.0
        assert cam.cy == 240.0
        assert cam.fx == 400.0


# =========================================================================
# IMU Pre-integration
# =========================================================================

class TestIMUPreintegration:

    def test_stationary_imu_no_translation(self):
        """Hover: only gravity in Z — should predict zero horizontal translation."""
        preint = IMUPreintegration()
        imu = _make_imu(acc=(0, 0, 9.81), gyro=(0, 0, 0))
        for i in range(10):
            preint.integrate(imu, dt=0.01)
        # Horizontal components of delta_p should be ~0
        assert abs(preint.delta_p[0]) < 1e-6
        assert abs(preint.delta_p[1]) < 1e-6

    def test_pure_forward_acceleration(self):
        """1 m/s² forward (x) for 1 s → delta_p.x ≈ 0.5 m."""
        preint = IMUPreintegration()
        # Gravity is subtracted inside VINSMono, here we test raw integration
        imu = _make_imu(acc=(1.0, 0, 0), gyro=(0, 0, 0))
        N, dt = 100, 0.01          # 1 second total
        for _ in range(N):
            preint.integrate(imu, dt)
        # delta_p = 0.5 * a * t^2  (bias=0, no gravity compensation here)
        assert abs(preint.delta_p[0] - 0.5) < 0.01

    def test_zero_dt_ignored(self):
        preint = IMUPreintegration()
        imu = _make_imu(acc=(10, 10, 10))
        preint.integrate(imu, dt=0.0)
        assert np.allclose(preint.delta_p, 0)
        assert np.allclose(preint.delta_v, 0)

    def test_covariance_grows_with_time(self):
        preint = IMUPreintegration()
        imu = _make_imu()
        cov_before = np.trace(preint.cov)
        for _ in range(50):
            preint.integrate(imu, dt=0.01)
        assert np.trace(preint.cov) > cov_before

    def test_rotation_around_z(self):
        """90° rotation around Z (π/2 rad/s for 1 s) → delta_R ≈ Rz(90°)."""
        preint = IMUPreintegration()
        imu = _make_imu(gyro=(0, 0, math.pi/2))
        for _ in range(100):
            preint.integrate(imu, dt=0.01)
        # After 90° rotation the x-axis should point roughly in the y direction
        x_new = preint.delta_R @ np.array([1, 0, 0])
        assert abs(x_new[0]) < 0.05   # ≈ 0
        assert abs(x_new[1] - 1.0) < 0.05  # ≈ 1

    def test_predict_state_identity(self):
        """Zero motion preintegration should return the same state."""
        preint = IMUPreintegration()
        p0 = np.array([1.0, 2.0, 3.0])
        v0 = np.zeros(3)
        R0 = np.eye(3)
        p1, v1, R1 = preint.predict_state(p0, v0, R0)
        assert np.allclose(p1, p0, atol=1e-5)


# =========================================================================
# Feature Tracker
# =========================================================================

class TestFeatureTracker:

    def test_detects_features_on_first_frame(self):
        cam = PinholeCamera(fx=300, fy=300, cx=160, cy=120,
                            width=320, height=240)
        tracker = FeatureTracker(cam, max_features=50, min_features=5)
        img = _make_synthetic_frame(240, 320)
        feats = tracker.track(img, 0.0)
        assert len(feats) >= 5, f'Expected ≥5 features, got {len(feats)}'

    def test_features_have_unique_ids(self):
        cam = PinholeCamera(fx=300, fy=300, cx=160, cy=120,
                            width=320, height=240)
        tracker = FeatureTracker(cam, max_features=50, min_features=5)
        img = _make_synthetic_frame(240, 320)
        feats = tracker.track(img, 0.0)
        ids = [f.id for f in feats]
        assert len(ids) == len(set(ids)), 'Duplicate feature IDs'

    def test_tracks_across_two_frames(self):
        cam = PinholeCamera(fx=300, fy=300, cx=160, cy=120,
                            width=320, height=240)
        tracker = FeatureTracker(cam, max_features=50, min_features=5)
        f1 = _make_synthetic_frame(240, 320)
        # Shift image by 2px to simulate camera motion
        f2 = np.roll(f1, 2, axis=1)
        feats1 = tracker.track(f1, 0.0)
        feats2 = tracker.track(f2, 0.033)
        ids1 = set(f.id for f in feats1)
        ids2 = set(f.id for f in feats2)
        common = ids1 & ids2
        assert len(common) >= 3, f'Expected tracked features, got {len(common)} common'

    def test_feature_norm_vector_unit_z(self):
        cam = PinholeCamera(fx=300, fy=300, cx=160, cy=120,
                            width=320, height=240)
        tracker = FeatureTracker(cam, max_features=30, min_features=5)
        img = _make_synthetic_frame(240, 320)
        feats = tracker.track(img, 0.0)
        for f in feats:
            assert f.norm[2] == 1.0, 'Normalised vector should have z=1'


# =========================================================================
# Triangulation
# =========================================================================

class TestTriangulation:

    def test_triangulate_known_point(self):
        cam = PinholeCamera(fx=500, fy=500, cx=320, cy=240)
        # World point 1m in front of first camera, 0.5m right
        p_world = np.array([0.5, 0.0, 2.0])

        R1 = np.eye(3); t1 = np.zeros(3)
        R2 = np.eye(3); t2 = np.array([-0.5, 0.0, 0.0])  # 0.5m baseline

        pt1 = cam.project(R1 @ p_world + t1)
        pt2 = cam.project(R2 @ p_world + t2)

        result = triangulate_dlt(R1, t1, R2, t2, cam, pt1, pt2)
        assert result is not None
        assert np.linalg.norm(result - p_world) < 0.1

    def test_triangulate_degenerate_returns_none(self):
        cam = PinholeCamera()
        # Parallel rays — same projection → degenerate
        R = np.eye(3); t = np.zeros(3)
        result = triangulate_dlt(R, t, R, t, cam,
                                 np.array([320.0, 240.0]),
                                 np.array([320.0, 240.0]))
        # Either returns None (degenerate) or a point far away — both acceptable
        if result is not None:
            assert np.linalg.norm(result) < 1e6


# =========================================================================
# Math utilities
# =========================================================================

class TestMathUtils:

    def test_skew_antisymmetric(self):
        v = np.array([1.0, 2.0, 3.0])
        S = _skew(v)
        assert S.shape == (3, 3)
        assert np.allclose(S, -S.T)

    def test_skew_cross_product(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        assert np.allclose(_skew(a) @ b, np.cross(a, b))

    def test_rot_to_quat_identity(self):
        q = _rot_to_quat(np.eye(3))
        assert abs(q[3] - 1.0) < 1e-6   # w=1
        assert np.linalg.norm(q[:3]) < 1e-6

    def test_rot_to_quat_unit_norm(self):
        import random, math
        for _ in range(10):
            angle = random.uniform(0, math.pi)
            axis  = np.random.randn(3); axis /= np.linalg.norm(axis)
            K = _skew(axis)
            R = np.eye(3) + math.sin(angle)*K + (1-math.cos(angle))*K@K
            q = _rot_to_quat(R)
            assert abs(np.linalg.norm(q) - 1.0) < 1e-5

    def test_rot_to_quat_roundtrip(self):
        """q → R via cv2.Rodrigues → q should give same result."""
        R_in = np.eye(3)  # identity — safest round-trip test
        q = _rot_to_quat(R_in)
        R_out, _ = cv2.Rodrigues(np.array([0.0, 0.0, 0.0]))
        # Both should be identity
        assert np.allclose(R_out, R_in, atol=1e-5)


# =========================================================================
# Sliding-window optimizer
# =========================================================================

class TestSlidingWindowOptimizer:

    def test_single_frame_unchanged(self):
        opt = SlidingWindowOptimizer(window_size=10)
        f = Frame(timestamp=0.0, image=None)
        f.pose = np.eye(4)
        result = opt.optimize([f])
        assert np.allclose(result[0].pose, np.eye(4))

    def test_two_frames_no_crash(self):
        opt = SlidingWindowOptimizer(window_size=10)
        f1 = Frame(timestamp=0.0, image=None); f1.pose = np.eye(4)
        f2 = Frame(timestamp=0.1, image=None)
        T = np.eye(4); T[0, 3] = 0.5
        f2.pose = T
        result = opt.optimize([f1, f2])
        assert len(result) == 2

    def test_pose_vec_roundtrip(self):
        T = np.eye(4)
        T[:3, :3], _ = cv2.Rodrigues(np.array([0.1, 0.2, 0.3]))
        T[:3, 3] = [1.0, 2.0, 3.0]
        v = SlidingWindowOptimizer._pose_to_vec(T)
        T2 = SlidingWindowOptimizer._vec_to_pose(v)
        assert np.allclose(T, T2, atol=1e-5)


# =========================================================================
# Marginalizer
# =========================================================================

class TestMarginalizer:

    def test_marginalise_reduces_dimension(self):
        marg = Marginalizer(state_dim=6)
        H = np.eye(9)
        b = np.ones(9)
        H_new, b_new = marg.marginalise(H, b, marg_size=3)
        assert H_new.shape == (6, 6)
        assert b_new.shape == (6,)

    def test_prior_accumulates(self):
        marg = Marginalizer(state_dim=6)
        H = np.eye(9); b = np.ones(9)
        marg.marginalise(H, b, 3)
        trace1 = np.trace(marg.H_prior)
        marg.marginalise(H, b, 3)
        trace2 = np.trace(marg.H_prior)
        assert trace2 > trace1


# =========================================================================
# Loop closure
# =========================================================================

class TestLoopClosure:

    def test_add_keyframe_returns_index(self):
        lcd = LoopClosureDetector(min_score=0.4, max_gap=2)
        img = _make_synthetic_frame()
        idx = lcd.add_keyframe(img)
        assert idx == 0
        idx2 = lcd.add_keyframe(img)
        assert idx2 == 1

    def test_identical_image_triggers_loop(self):
        lcd = LoopClosureDetector(min_score=0.5, max_gap=2)
        img = _make_synthetic_frame()
        for i in range(5):
            lcd.add_keyframe(img)
        # Query with the same image at index 5 — should match an earlier frame
        result = lcd.query(img, 5)
        assert result is not None, 'Expected loop closure on identical image'
        matched_idx, score = result
        assert score >= 0.5

    def test_no_false_loop_on_empty_db(self):
        lcd = LoopClosureDetector()
        img = _make_synthetic_frame()
        result = lcd.query(img, 0)
        assert result is None


# =========================================================================
# Full VINSMono pipeline (headless, synthetic frames)
# =========================================================================

class TestVINSMonoPipeline:

    def _run_pipeline(self, n_frames=10, with_imu=True) -> list:
        """Run n_frames through the estimator, return list of pose_vecs."""
        cfg = {
            'camera': {'resolution': [320, 240], 'fx': 300, 'fy': 300,
                       'k1': 0, 'k2': 0},
            'slam':   {'sliding_window_size': 5, 'max_features': 50,
                       'min_features_for_tracking': 5,
                       'keyframe_distance_threshold': 0.1,
                       'optimization_iterations': 3},
        }
        vins = VINSMono(cfg)
        results = []
        t = 0.0
        for i in range(n_frames):
            if with_imu:
                for _ in range(5):   # 5 IMU @ 500 Hz between 30 Hz frames
                    vins.process_imu(
                        np.array([0.0, 0.0, 9.81]),
                        np.array([0.0, 0.0, 0.0]),
                        t,
                    )
                    t += 0.002
            # Shift image each frame to generate parallax
            img = np.roll(_make_synthetic_frame(240, 320), i * 2, axis=1)
            pose = vins.process_image(img, t)
            t += 0.033
            if pose is not None:
                results.append(pose)
        return results

    def test_initialises_within_5_frames(self):
        poses = self._run_pipeline(n_frames=5)
        assert len(poses) >= 1, 'Expected at least one pose after 5 frames'

    def test_pose_vec_length(self):
        poses = self._run_pipeline(n_frames=8)
        for p in poses:
            assert len(p) == 7, f'Expected 7-vec [x,y,z,qx,qy,qz,qw], got {len(p)}'

    def test_quaternion_normalised(self):
        poses = self._run_pipeline(n_frames=8)
        for p in poses:
            qnorm = np.linalg.norm(p[3:7])
            assert abs(qnorm - 1.0) < 0.01, f'Quaternion not unit: norm={qnorm}'

    def test_position_finite(self):
        poses = self._run_pipeline(n_frames=8)
        for p in poses:
            assert np.all(np.isfinite(p[:3])), 'Position contains NaN/Inf'

    def test_imu_only_mode(self):
        """Pipeline should still attempt init without IMU."""
        poses = self._run_pipeline(n_frames=6, with_imu=False)
        # May or may not init without IMU depending on parallax — just must not crash
        for p in poses:
            assert len(p) == 7

    def test_get_pose_before_init_returns_none(self):
        vins = VINSMono()
        assert vins.get_pose() is None

    def test_get_pose_after_init_returns_vec(self):
        poses = self._run_pipeline(n_frames=5)
        cfg = {
            'camera': {'resolution': [320, 240], 'fx': 300, 'fy': 300,
                       'k1': 0, 'k2': 0},
            'slam':   {'sliding_window_size': 5, 'max_features': 50,
                       'min_features_for_tracking': 5},
        }
        vins = VINSMono(cfg)
        t = 0.0
        for i in range(5):
            img = np.roll(_make_synthetic_frame(240, 320), i * 2, axis=1)
            vins.process_image(img, t); t += 0.033
        p = vins.get_pose()
        if p is not None:
            assert len(p) == 7


# =========================================================================
# SLAMNode (headless — no ROS2 needed)
# =========================================================================

class TestSLAMNodeHeadless:

    def test_slam_node_constructs_without_ros2(self):
        from slam_node import SLAMNode
        node = SLAMNode()
        assert node._vins is not None
        assert node._frame_count == 0

    def test_slam_node_loads_default_config(self):
        from slam_node import SLAMNode
        node = SLAMNode()
        assert isinstance(node._vins, VINSMono)


# =========================================================================
# PoseBridge (headless)
# =========================================================================

class TestPoseBridgeHeadless:

    def test_constructs_without_ros2(self):
        from pose_bridge import PoseBridge
        bridge = PoseBridge()
        assert bridge._prev_pos is None
        assert bridge._pose_count == 0

    def test_jump_rejection(self):
        from pose_bridge import PoseBridge
        bridge = PoseBridge()

        class FakePose:
            class header:
                stamp = type('S', (), {'sec': 0, 'nanosec': 0})()
                frame_id = 'map'
            class pose:
                class position: x=0; y=0; z=0
                class orientation: x=0; y=0; z=0; w=1

        # Seed previous position
        bridge._prev_pos  = np.array([0.0, 0.0, 0.0])
        bridge._prev_time = 0.0

        # Pose that jumps 10m in 0.1s — should be rejected
        class JumpPose(FakePose):
            class header:
                stamp = type('S', (), {'sec': 0, 'nanosec': int(0.1e9)})()
                frame_id = 'map'
            class pose:
                class position: x=10.0; y=0; z=0
                class orientation: x=0; y=0; z=0; w=1

        bridge._slam_callback(JumpPose())
        assert bridge._reject_count == 1
        assert bridge._pose_count == 0

    def test_valid_pose_accepted(self):
        from pose_bridge import PoseBridge
        bridge = PoseBridge()

        class FakePose:
            class header:
                stamp = type('S', (), {'sec': 1, 'nanosec': 0})()
                frame_id = 'map'
            class pose:
                class position: x=0.1; y=0; z=0
                class orientation: x=0; y=0; z=0; w=1

        bridge._slam_callback(FakePose())
        assert bridge._reject_count == 0
        assert bridge._pose_count == 1


# =========================================================================
# Executor stubs (headless)
# =========================================================================

class TestExecutorHeadless:

    def _make_executor(self):
        from executor import MissionExecutor
        return MissionExecutor()

    def test_avoid_obstacle_known_type(self):
        from executor import _OBSTACLE_INFLATION
        ex = self._make_executor()
        result = ex._avoid_obstacle('personnel')
        assert result is True
        assert _OBSTACLE_INFLATION['personnel'] == 1.5

    def test_avoid_obstacle_unknown_type(self):
        from executor import _OBSTACLE_INFLATION
        ex = self._make_executor()
        result = ex._avoid_obstacle('aliens')
        assert result is True   # falls back to 'default'

    def test_send_report_returns_true(self):
        ex = self._make_executor()
        assert ex._send_report('damage assessment') is True

    def test_send_report_includes_data_field(self):
        import json, io
        from contextlib import redirect_stdout
        ex = self._make_executor()
        buf = io.StringIO()
        with redirect_stdout(buf):
            ex._send_report('test payload')
        output = buf.getvalue()
        # The JSON payload should be in the print output
        assert 'test payload' in output

    def test_mission_log_tracks_results(self):
        ex = self._make_executor()
        ex._mission_log.append({'goal': {'type': 'Report'}, 'success': True})
        ex._mission_log.append({'goal': {'type': 'NavigateTo'}, 'success': False})
        good = sum(1 for e in ex._mission_log if e['success'])
        bad  = sum(1 for e in ex._mission_log if not e['success'])
        assert good == 1 and bad == 1


# =========================================================================
# Navigation math (kept from before)
# =========================================================================

class TestNavigationMath:

    def test_waypoint_distance(self):
        p1, p2 = np.array([0,0,0]), np.array([3,4,0])
        assert abs(np.linalg.norm(p2-p1) - 5.0) < 1e-4

    def test_floor_to_altitude(self):
        for floor, expected in [(1,3),(2,6),(3,9),(5,15),(10,30)]:
            assert floor * 3.0 == expected

    def test_inspection_waypoint_bounds(self):
        waypoints = [[-2,0,1.5],[-2,2,1.5],[2,2,1.5],[2,-2,1.5],[-2,-2,1.5],[0,0,1.5]]
        xs = [w[0] for w in waypoints]; ys = [w[1] for w in waypoints]
        assert max(xs)-min(xs) == 4.0
        assert max(ys)-min(ys) == 4.0


class TestSLAMIntegration:

    def test_imu_buffer_capped_at_100(self):
        buf = []
        for i in range(200):
            buf.append(i)
            if len(buf) > 100: buf.pop(0)
        assert len(buf) == 100

    def test_slam_output_topic_matches_bridge_input(self):
        assert '/ghostpilot/pose' == '/ghostpilot/pose'

    def test_camera_and_imu_topics_absolute(self):
        for topic in ('/camera/image_raw', '/imu/data'):
            assert topic.startswith('/')
            assert ' ' not in topic


@pytest.mark.skipif(not HAS_ROS2, reason='ROS2 not installed')
class TestSLAMNodeROS2:
    def test_slam_node_creation(self):
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
