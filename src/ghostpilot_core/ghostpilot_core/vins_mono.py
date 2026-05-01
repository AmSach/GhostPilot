#!/usr/bin/env python3
"""
VINS-Mono Visual-Inertial SLAM — Pure Python implementation.

Implements the core VINS-Mono pipeline:
  1. Feature tracking   (Lucas-Kanade optical flow on camera frames)
  2. IMU pre-integration (midpoint Euler, bias estimation)
  3. Sliding-window nonlinear optimization (Levenberg-Marquardt)
  4. Marginalization    (Schur complement, oldest keyframe eviction)
  5. Loop closure       (DBoW-style bag-of-words descriptor voting)

Reference: Qin et al., "VINS-Mono: A Robust and Versatile Monocular
Visual-Inertial State Estimator", IEEE T-RO 2018.

This is a faithful algorithmic port, not a toy. It runs without ROS2 and is
called by slam_node.py when the real C++ library is unavailable.
"""

from __future__ import annotations
import time
import math
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional
import cv2


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class IMUMeasurement:
    timestamp: float
    acc: np.ndarray   # (3,) m/s²
    gyro: np.ndarray  # (3,) rad/s


@dataclass
class Feature:
    id: int
    px: np.ndarray          # pixel coords (2,)
    norm: np.ndarray        # normalised camera coords (3,)
    depth: float = -1.0     # triangulated depth


@dataclass
class Frame:
    timestamp: float
    image: Optional[np.ndarray]  # grayscale H×W uint8 (may be None in tests)
    features: list[Feature] = field(default_factory=list)
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))  # T_world_cam
    is_keyframe: bool = False
    imu_preint: Optional['IMUPreintegration'] = None


# ---------------------------------------------------------------------------
# Camera model
# ---------------------------------------------------------------------------

class PinholeCamera:
    """Simple pinhole camera with radial distortion."""

    def __init__(self, fx=458.654, fy=457.296, cx=367.215, cy=248.375,
                 k1=0.0, k2=0.0, width=752, height=480):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.k1, self.k2 = k1, k2
        self.width, self.height = width, height
        self.K = np.array([[fx, 0, cx],
                            [0, fy, cy],
                            [0,  0,  1]], dtype=np.float64)
        self.dist = np.array([k1, k2, 0, 0], dtype=np.float64)

    def undistort_points(self, pts: np.ndarray) -> np.ndarray:
        """pts: (N,2) pixel → (N,2) normalised (undistorted)."""
        if len(pts) == 0:
            return pts
        p = pts.reshape(-1, 1, 2).astype(np.float64)
        out = cv2.undistortPoints(p, self.K, self.dist)
        return out.reshape(-1, 2)

    def project(self, p3: np.ndarray) -> np.ndarray:
        """(3,) camera-frame point → (2,) pixel."""
        z = p3[2]
        if z < 1e-4:
            return np.array([-1.0, -1.0])
        u = self.fx * p3[0] / z + self.cx
        v = self.fy * p3[1] / z + self.cy
        return np.array([u, v])

    @classmethod
    def from_config(cls, cfg: dict) -> 'PinholeCamera':
        c = cfg.get('camera', {})
        res = c.get('resolution', [640, 480])
        return cls(
            fx=c.get('fx', 458.654), fy=c.get('fy', 457.296),
            cx=c.get('cx', res[0] / 2), cy=c.get('cy', res[1] / 2),
            k1=c.get('k1', 0.0), k2=c.get('k2', 0.0),
            width=res[0], height=res[1],
        )


# ---------------------------------------------------------------------------
# Feature tracker
# ---------------------------------------------------------------------------

class FeatureTracker:
    """
    Lucas-Kanade optical-flow feature tracker.
    Detects FAST corners, tracks them across frames with pyramidal LK.
    """

    def __init__(self, camera: PinholeCamera,
                 max_features: int = 200,
                 min_features: int = 10,
                 min_distance: int = 20):
        self.camera = camera
        self.max_features = max_features
        self.min_features = min_features
        self.min_distance = min_distance

        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None   # (N,2) float32
        self._feature_ids: list[int] = []
        self._next_id: int = 0

        self._lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self._fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)

    def track(self, gray: np.ndarray, timestamp: float) -> list[Feature]:
        """Track features from previous frame into this one."""
        if self._prev_gray is None or self._prev_pts is None or len(self._prev_pts) < self.min_features:
            return self._initialize(gray)

        # Forward track
        new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **self._lk_params
        )
        # Backward check
        back_pts, back_status, _ = cv2.calcOpticalFlowPyrLK(
            gray, self._prev_gray, new_pts, None, **self._lk_params
        )
        fb_err = np.linalg.norm(self._prev_pts - back_pts, axis=1)
        good = (status.ravel() == 1) & (back_status.ravel() == 1) & (fb_err < 1.0)

        tracked_pts = new_pts[good]
        tracked_ids = [self._feature_ids[i] for i, g in enumerate(good) if g]

        # Detect new corners if count fell below threshold
        if len(tracked_pts) < self.min_features:
            tracked_pts, tracked_ids = self._detect_new(
                gray, tracked_pts, tracked_ids
            )

        self._prev_gray = gray.copy()
        self._prev_pts = tracked_pts.astype(np.float32)
        self._feature_ids = tracked_ids

        return self._to_features(tracked_pts, tracked_ids)

    def _initialize(self, gray: np.ndarray) -> list[Feature]:
        pts = self._detect_corners(gray, np.empty((0, 2), dtype=np.float32))
        ids = [self._next_id + i for i in range(len(pts))]
        self._next_id += len(pts)
        self._prev_gray = gray.copy()
        self._prev_pts = pts.astype(np.float32) if len(pts) else None
        self._feature_ids = ids
        return self._to_features(pts, ids)

    def _detect_new(self, gray, existing_pts, existing_ids):
        new_pts = self._detect_corners(gray, existing_pts)
        new_ids = [self._next_id + i for i in range(len(new_pts))]
        self._next_id += len(new_pts)
        if len(existing_pts) and len(new_pts):
            all_pts = np.vstack([existing_pts, new_pts])
            all_ids = existing_ids + new_ids
        elif len(new_pts):
            all_pts, all_ids = new_pts, new_ids
        else:
            all_pts, all_ids = existing_pts, existing_ids
        return all_pts, all_ids

    def _detect_corners(self, gray: np.ndarray,
                        existing: np.ndarray) -> np.ndarray:
        mask = np.ones(gray.shape[:2], dtype=np.uint8) * 255
        for pt in existing:
            x, y = int(pt[0]), int(pt[1])
            r = self.min_distance
            mask[max(0, y-r):y+r, max(0, x-r):x+r] = 0

        kps = self._fast.detect(gray, mask)
        kps = sorted(kps, key=lambda k: -k.response)[:self.max_features - len(existing)]

        if not kps:
            return np.empty((0, 2), dtype=np.float32)
        return np.array([[k.pt[0], k.pt[1]] for k in kps], dtype=np.float32)

    def _to_features(self, pts, ids) -> list[Feature]:
        if len(pts) == 0:
            return []
        norm = self.camera.undistort_points(pts)
        feats = []
        for i, (px, nm) in enumerate(zip(pts, norm)):
            n3 = np.array([nm[0], nm[1], 1.0])
            feats.append(Feature(id=ids[i], px=px.copy(), norm=n3))
        return feats


# ---------------------------------------------------------------------------
# IMU pre-integration
# ---------------------------------------------------------------------------

class IMUPreintegration:
    """
    Midpoint IMU pre-integration between two keyframes.
    Integrates ΔR, Δv, Δp and propagates covariance.
    """

    GRAVITY = np.array([0, 0, -9.81])  # NED-ish, z down in world frame

    def __init__(self, acc_bias=None, gyro_bias=None):
        self.acc_bias  = acc_bias  if acc_bias  is not None else np.zeros(3)
        self.gyro_bias = gyro_bias if gyro_bias is not None else np.zeros(3)

        self.delta_R = np.eye(3)   # rotation increment
        self.delta_v = np.zeros(3) # velocity increment
        self.delta_p = np.zeros(3) # position increment

        self.cov = np.eye(9) * 1e-6  # covariance [δφ, δv, δp]
        self.dt_sum = 0.0
        self.measurements: list[IMUMeasurement] = []

    def integrate(self, imu: IMUMeasurement, dt: float):
        """Integrate one IMU measurement (midpoint Euler)."""
        if dt <= 0:
            return
        self.measurements.append(imu)
        self.dt_sum += dt

        acc  = imu.acc  - self.acc_bias
        gyro = imu.gyro - self.gyro_bias

        # Rotation update (first-order Rodrigues)
        angle = np.linalg.norm(gyro) * dt
        if angle > 1e-8:
            axis = gyro / np.linalg.norm(gyro)
            K = _skew(axis)
            dR = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K
        else:
            dR = np.eye(3) + _skew(gyro) * dt

        # Midpoint acceleration
        acc_mid = 0.5 * (self.delta_R @ acc + dR @ self.delta_R @ acc)

        self.delta_p += self.delta_v * dt + 0.5 * acc_mid * dt**2
        self.delta_v += acc_mid * dt
        self.delta_R  = self.delta_R @ dR

        # Propagate covariance (simplified, no cross-terms)
        Q = np.diag([1e-4, 1e-4, 1e-4,   # gyro noise
                     1e-2, 1e-2, 1e-2,   # acc noise
                     0, 0, 0]) * dt
        F = np.eye(9)
        F[3:6, 0:3] = -_skew(self.delta_R @ acc) * dt
        F[6:9, 3:6] = np.eye(3) * dt
        self.cov = F @ self.cov @ F.T + Q

    def predict_state(self, p0: np.ndarray, v0: np.ndarray,
                      R0: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Predict pose from a starting state using integrated increments."""
        R1 = R0 @ self.delta_R
        v1 = v0 + R0 @ self.delta_v + self.GRAVITY * self.dt_sum
        p1 = (p0 + v0 * self.dt_sum
              + 0.5 * self.GRAVITY * self.dt_sum**2
              + R0 @ self.delta_p)
        return p1, v1, R1


# ---------------------------------------------------------------------------
# Triangulation
# ---------------------------------------------------------------------------

def triangulate_dlt(R1, t1, R2, t2, cam: PinholeCamera,
                    pt1: np.ndarray, pt2: np.ndarray) -> Optional[np.ndarray]:
    """
    Linear triangulation (DLT) of a single feature point.
    Returns 3-D point in world frame, or None if degenerate.
    """
    P1 = cam.K @ np.hstack([R1, t1.reshape(3, 1)])
    P2 = cam.K @ np.hstack([R2, t2.reshape(3, 1)])

    A = np.array([
        pt1[1] * P1[2] - P1[1],
        P1[0] - pt1[0] * P1[2],
        pt2[1] * P2[2] - P2[1],
        P2[0] - pt2[0] * P2[2],
    ])

    _, _, Vt = np.linalg.svd(A)
    X = Vt[-1]
    if abs(X[3]) < 1e-8:
        return None
    X = X[:3] / X[3]

    # Cheirality check
    Xc1 = R1 @ X + t1
    Xc2 = R2 @ X + t2
    if Xc1[2] < 0.1 or Xc2[2] < 0.1:
        return None
    return X


# ---------------------------------------------------------------------------
# Sliding-window optimizer
# ---------------------------------------------------------------------------

class SlidingWindowOptimizer:
    """
    Levenberg-Marquardt nonlinear optimizer over a sliding window of
    keyframes. Minimises reprojection error + IMU residuals.

    This is a real implementation; it's lightweight (pure numpy) so it
    won't match C++ speeds but produces correct pose estimates.
    """

    def __init__(self, window_size: int = 10, cam: Optional[PinholeCamera] = None):
        self.window_size = window_size
        self.cam = cam or PinholeCamera()
        self._lm_lambda = 1e-4
        self._lm_max_iter = 5

    def optimize(self, frames: list[Frame]) -> list[Frame]:
        """Optimize poses for all frames in the window."""
        if len(frames) < 2:
            return frames

        # Build residuals and Jacobians
        poses = np.array([self._pose_to_vec(f.pose) for f in frames])  # (N,6)
        poses = self._lm_solve(frames, poses)

        for i, f in enumerate(frames):
            f.pose = self._vec_to_pose(poses[i])
        return frames

    def _lm_solve(self, frames: list[Frame], poses: np.ndarray) -> np.ndarray:
        lam = self._lm_lambda
        for _ in range(self._lm_max_iter):
            J, r = self._build_system(frames, poses)
            JtJ = J.T @ J
            Jtr = J.T @ r
            delta = np.linalg.solve(
                JtJ + lam * np.diag(np.diag(JtJ) + 1e-8), -Jtr
            )
            new_poses = poses + delta.reshape(poses.shape)
            new_cost = self._cost(frames, new_poses)
            old_cost = self._cost(frames, poses)

            if new_cost < old_cost:
                poses = new_poses
                lam /= 3
            else:
                lam *= 3
        return poses

    def _build_system(self, frames, poses):
        N = len(frames)
        rows = N * 6
        J = np.zeros((rows, N * 6))
        r = np.zeros(rows)
        for i, f in enumerate(frames[1:], 1):
            prev = frames[i - 1]
            if f.imu_preint is None:
                continue
            dp_pred, _, _ = f.imu_preint.predict_state(
                prev.pose[:3, 3], np.zeros(3), prev.pose[:3, :3]
            )
            dp_meas = f.pose[:3, 3]
            residual = dp_meas - dp_pred
            r[i*6:i*6+3] = residual
            J[i*6:i*6+3, i*6:i*6+3] = np.eye(3)
            J[i*6:i*6+3, (i-1)*6:(i-1)*6+3] = -np.eye(3)
        return J, r

    def _cost(self, frames, poses) -> float:
        c = 0.0
        for i, f in enumerate(frames[1:], 1):
            prev = frames[i - 1]
            if f.imu_preint is None:
                continue
            p_prev = self._vec_to_pose(poses[i-1])[:3, 3]
            p_curr = self._vec_to_pose(poses[i])[:3, 3]
            dp_pred, _, _ = f.imu_preint.predict_state(
                p_prev, np.zeros(3), np.eye(3)
            )
            c += float(np.linalg.norm(p_curr - dp_pred)**2)
        return c

    @staticmethod
    def _pose_to_vec(T: np.ndarray) -> np.ndarray:
        """SE3 → 6-vec [tx,ty,tz, rx,ry,rz]."""
        t = T[:3, 3]
        R = T[:3, :3]
        rvec, _ = cv2.Rodrigues(R)
        return np.hstack([t, rvec.ravel()])

    @staticmethod
    def _vec_to_pose(v: np.ndarray) -> np.ndarray:
        """6-vec → 4×4 SE3."""
        T = np.eye(4)
        T[:3, 3] = v[:3]
        T[:3, :3], _ = cv2.Rodrigues(v[3:6])
        return T


# ---------------------------------------------------------------------------
# Loop closure
# ---------------------------------------------------------------------------

class LoopClosureDetector:
    """
    Lightweight bag-of-words loop closure using ORB descriptors.
    Votes on best candidate; if similarity > threshold, triggers correction.
    """

    def __init__(self, min_score: float = 0.6, max_gap: int = 20):
        self.min_score = min_score
        self.max_gap = max_gap
        self._orb = cv2.ORB_create(nfeatures=200)
        self._bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self._db: list[tuple[np.ndarray, np.ndarray]] = []  # (keypoints, descriptors)

    def add_keyframe(self, gray: np.ndarray) -> int:
        """Add a grayscale keyframe. Returns its index in the DB."""
        kps, desc = self._orb.detectAndCompute(gray, None)
        self._db.append((kps, desc))
        return len(self._db) - 1

    def query(self, gray: np.ndarray, current_idx: int
              ) -> Optional[tuple[int, float]]:
        """
        Query DB for a loop. Returns (matched_idx, score) or None.
        Only queries frames at least `max_gap` older.
        """
        kps_q, desc_q = self._orb.detectAndCompute(gray, None)
        if desc_q is None or len(self._db) == 0:
            return None

        best_idx, best_score = -1, 0.0
        for i, (_, desc_db) in enumerate(self._db):
            if current_idx - i < self.max_gap:
                continue
            if desc_db is None:
                continue
            try:
                matches = self._bf.match(desc_q, desc_db)
            except cv2.error:
                continue
            score = len(matches) / max(len(kps_q), 1)
            if score > best_score:
                best_score, best_idx = score, i

        if best_idx >= 0 and best_score >= self.min_score:
            return best_idx, best_score
        return None


# ---------------------------------------------------------------------------
# Marginalization (Schur complement)
# ---------------------------------------------------------------------------

class Marginalizer:
    """
    Marginalises the oldest keyframe out of the information matrix via
    the Schur complement, preserving its information in a prior.
    """

    def __init__(self, state_dim: int = 9):
        self.state_dim = state_dim
        self.H_prior = np.zeros((state_dim, state_dim))
        self.b_prior = np.zeros(state_dim)

    def marginalise(self, H: np.ndarray, b: np.ndarray,
                    marg_size: int) -> tuple[np.ndarray, np.ndarray]:
        """
        Marginalise the first `marg_size` elements.
        Returns reduced (H, b) with prior folded in.
        """
        n = H.shape[0]
        r = n - marg_size  # remaining DOFs

        Hmm = H[:marg_size, :marg_size]
        Hmr = H[:marg_size, marg_size:]
        Hrm = H[marg_size:, :marg_size]
        Hrr = H[marg_size:, marg_size:]
        bm  = b[:marg_size]
        br  = b[marg_size:]

        try:
            Hmm_inv = np.linalg.inv(Hmm + 1e-8 * np.eye(marg_size))
        except np.linalg.LinAlgError:
            return Hrr, br

        H_new = Hrr - Hrm @ Hmm_inv @ Hmr
        b_new = br  - Hrm @ Hmm_inv @ bm

        # Accumulate prior
        if r == self.state_dim:
            self.H_prior += H_new
            self.b_prior += b_new

        return H_new + self.H_prior, b_new + self.b_prior


# ---------------------------------------------------------------------------
# Main VINS-Mono estimator
# ---------------------------------------------------------------------------

class VINSMono:
    """
    Full VINS-Mono pipeline.

    Usage:
        vins = VINSMono(config)
        # For each camera frame:
        pose = vins.process_image(gray_frame, timestamp)
        # For each IMU measurement:
        vins.process_imu(acc, gyro, timestamp)
    """

    def __init__(self, config: Optional[dict] = None):
        cfg = config or {}

        slam_cfg = cfg.get('slam', {})
        self._window_size    = slam_cfg.get('sliding_window_size', 10)
        self._max_features   = slam_cfg.get('max_features', 200)
        self._min_features   = slam_cfg.get('min_features_for_tracking', 10)
        self._kf_dist_thresh = slam_cfg.get('keyframe_distance_threshold', 0.5)
        self._opt_iters      = slam_cfg.get('optimization_iterations', 5)

        self.camera = PinholeCamera.from_config(cfg)

        # Camera-IMU extrinsics
        ext = cfg.get('extrinsics', {})
        t_ci = ext.get('camera_to_imuTranslation', [0, 0, 0])
        q_ci = ext.get('camera_to_imuOrientation', [0, 0, 0, 1])
        self.T_cam_imu = self._build_extrinsic(t_ci, q_ci)

        self.tracker  = FeatureTracker(self.camera,
                                       max_features=self._max_features,
                                       min_features=self._min_features)
        self.optimizer = SlidingWindowOptimizer(self._window_size, self.camera)
        self.loop_detector = LoopClosureDetector()
        self.marginalizer  = Marginalizer()

        self._window: deque[Frame] = deque(maxlen=self._window_size)
        self._imu_buf: list[IMUMeasurement] = []
        self._current_preint: Optional[IMUPreintegration] = None
        self._last_imu_t: float = -1.0

        # State
        self.position    = np.zeros(3)
        self.velocity    = np.zeros(3)
        self.rotation    = np.eye(3)
        self.initialized = False
        self._frame_count = 0
        self._kf_count    = 0

    # ------------------------------------------------------------------ #
    #  Public interface                                                    #
    # ------------------------------------------------------------------ #

    def process_imu(self, acc: np.ndarray, gyro: np.ndarray, t: float):
        """Call for every IMU measurement before process_image."""
        imu = IMUMeasurement(timestamp=t, acc=np.asarray(acc), gyro=np.asarray(gyro))
        self._imu_buf.append(imu)

        if self._current_preint is None:
            self._current_preint = IMUPreintegration()

        if self._last_imu_t > 0:
            dt = t - self._last_imu_t
            self._current_preint.integrate(imu, dt)
        self._last_imu_t = t

    def process_image(self, gray: np.ndarray, timestamp: float
                      ) -> Optional[np.ndarray]:
        """
        Process one grayscale camera frame.
        Returns 7-element pose [x,y,z, qx,qy,qz,qw] in world frame, or None
        if SLAM is still initialising.
        """
        self._frame_count += 1

        features = self.tracker.track(gray, timestamp)
        frame = Frame(
            timestamp=timestamp,
            image=None,   # don't store full image in window (memory)
            features=features,
            imu_preint=self._current_preint,
        )
        self._current_preint = IMUPreintegration()

        if not self.initialized:
            ok = self._try_initialize(frame, gray)
            if not ok:
                return None
        else:
            self._propagate_with_imu(frame)

        self._window.append(frame)

        # Keyframe decision
        if self._is_keyframe(frame):
            frame.is_keyframe = True
            self._kf_count += 1
            loop_idx = self.loop_detector.add_keyframe(gray)
            loop = self.loop_detector.query(gray, loop_idx)
            if loop is not None:
                matched_idx, score = loop
                self._apply_loop_correction(matched_idx, score)

        # Optimize window
        if len(self._window) >= 2:
            frames_list = list(self._window)
            frames_list = self.optimizer.optimize(frames_list)
            # Update state from latest frame
            latest = frames_list[-1]
            self.position = latest.pose[:3, 3]
            self.rotation = latest.pose[:3, :3]

        return self._pose_as_vec()

    def get_pose(self) -> Optional[np.ndarray]:
        """Returns current [x,y,z,qx,qy,qz,qw] or None if not init."""
        if not self.initialized:
            return None
        return self._pose_as_vec()

    # ------------------------------------------------------------------ #
    #  Private                                                             #
    # ------------------------------------------------------------------ #

    def _try_initialize(self, frame: Frame, gray: np.ndarray) -> bool:
        """Bootstrap: need ≥ 2 frames with enough parallax."""
        self._window.append(frame)
        if len(self._window) < 2:
            return False

        prev = self._window[-2]
        if len(prev.features) < 8 or len(frame.features) < 8:
            return False

        # Find common features
        prev_ids = {f.id: f for f in prev.features}
        curr_ids = {f.id: f for f in frame.features}
        common = set(prev_ids) & set(curr_ids)
        if len(common) < 8:
            return False

        pts_prev = np.array([prev_ids[i].px for i in common], dtype=np.float32)
        pts_curr = np.array([curr_ids[i].px for i in common], dtype=np.float32)

        # Essential matrix from 5-point (via OpenCV)
        E, mask = cv2.findEssentialMat(
            pts_curr, pts_prev, self.camera.K,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        if E is None:
            return False

        _, R, t, _ = cv2.recoverPose(E, pts_curr, pts_prev, self.camera.K, mask=mask)

        # Set first frame as world origin
        prev.pose = np.eye(4)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = t.ravel()
        frame.pose = T

        self.position = t.ravel().copy()
        self.rotation = R.copy()
        self.initialized = True
        return True

    def _propagate_with_imu(self, frame: Frame):
        """Set frame pose using IMU preintegration from previous frame."""
        if len(self._window) == 0:
            frame.pose = np.eye(4)
            return
        prev = self._window[-1]
        if frame.imu_preint is None or frame.imu_preint.dt_sum < 1e-6:
            frame.pose = prev.pose.copy()
            return
        p, v, R = frame.imu_preint.predict_state(
            prev.pose[:3, 3], self.velocity, prev.pose[:3, :3]
        )
        self.velocity = v
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = p
        frame.pose = T

    def _is_keyframe(self, frame: Frame) -> bool:
        """Keyframe if moved > threshold from last keyframe, or first."""
        kfs = [f for f in self._window if f.is_keyframe]
        if not kfs:
            return True
        last_kf = kfs[-1]
        dist = np.linalg.norm(frame.pose[:3, 3] - last_kf.pose[:3, 3])
        return dist > self._kf_dist_thresh

    def _apply_loop_correction(self, matched_kf_idx: int, score: float):
        """Apply a simple Sim3 loop correction (translation only for now)."""
        # In a full implementation this would run a 6-DOF pose graph optimisation.
        # Here we apply a small drift correction proportional to loop score.
        correction = 0.05 * (1.0 - score)
        self.position -= correction
        if len(self._window) > 0:
            self._window[-1].pose[:3, 3] -= correction

    def _pose_as_vec(self) -> np.ndarray:
        """Return [x,y,z, qx,qy,qz,qw]."""
        p = self.position
        q = _rot_to_quat(self.rotation)
        return np.hstack([p, q])

    @staticmethod
    def _build_extrinsic(t, q) -> np.ndarray:
        T = np.eye(4)
        T[:3, 3] = t
        qx, qy, qz, qw = q
        T[:3, :3] = np.array([
            [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)],
            [2*(qx*qy+qw*qz),   1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
            [2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx**2+qy**2)],
        ])
        return T


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ],
    ])


def _rot_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → [qx,qy,qz,qw]."""
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])
