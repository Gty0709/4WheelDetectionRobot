"""Odom ring buffer interpolation/extrapolation for map→camera TF composition."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from detection_pkg.backprojection import make_transform


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class OdomSample:
    stamp_ns: int
    position: np.ndarray  # (3,)
    rotation: np.ndarray  # 3x3
    vx: float
    vy: float
    vz: float
    omega_z: float

    def to_matrix(self) -> np.ndarray:
        return make_transform(self.rotation, self.position)


def compose_map_camera_tf(
    t_map_odom: np.ndarray,
    t_odom_base: np.ndarray,
    t_base_camera: np.ndarray,
) -> np.ndarray:
    """Compose map→camera from map→odom, odom→base, base→camera."""
    return t_map_odom @ t_odom_base @ t_base_camera


def _yaw_from_rotation(rot: np.ndarray) -> float:
    return float(Rotation.from_matrix(rot).as_euler('zyx')[0])


def _rotation_from_yaw(yaw: float) -> np.ndarray:
    return Rotation.from_euler('z', yaw).as_matrix()


def extrapolate_sample(sample: OdomSample, dt_sec: float) -> OdomSample:
    """Short forward extrapolation using body-frame twist (diff-drive friendly)."""
    yaw = _yaw_from_rotation(sample.rotation)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    dx = (cos_y * sample.vx - sin_y * sample.vy) * dt_sec
    dy = (sin_y * sample.vx + cos_y * sample.vy) * dt_sec
    dz = sample.vz * dt_sec
    yaw_new = yaw + sample.omega_z * dt_sec
    return OdomSample(
        stamp_ns=sample.stamp_ns + int(dt_sec * 1_000_000_000),
        position=sample.position + np.array([dx, dy, dz], dtype=np.float64),
        rotation=_rotation_from_yaw(yaw_new),
        vx=sample.vx,
        vy=sample.vy,
        vz=sample.vz,
        omega_z=sample.omega_z,
    )


def _interpolate_samples(s0: OdomSample, s1: OdomSample, stamp_ns: int) -> OdomSample:
    if s1.stamp_ns == s0.stamp_ns:
        return s0
    alpha = (stamp_ns - s0.stamp_ns) / (s1.stamp_ns - s0.stamp_ns)
    alpha = min(max(alpha, 0.0), 1.0)
    rots = Rotation.from_matrix(np.stack([s0.rotation, s1.rotation]))
    rot = Slerp([0.0, 1.0], rots)(alpha).as_matrix()
    pos = (1.0 - alpha) * s0.position + alpha * s1.position
    return OdomSample(
        stamp_ns=stamp_ns,
        position=pos,
        rotation=rot,
        vx=(1.0 - alpha) * s0.vx + alpha * s1.vx,
        vy=(1.0 - alpha) * s0.vy + alpha * s1.vy,
        vz=(1.0 - alpha) * s0.vz + alpha * s1.vz,
        omega_z=(1.0 - alpha) * s0.omega_z + alpha * s1.omega_z,
    )


class OdomRingBuffer:
    """Rolling /odom buffer with interpolate + bounded forward extrapolation."""

    def __init__(
        self,
        duration_sec: float = 2.0,
        interp_max_gap_ns: int = 80_000_000,
        max_extrapolate_ns: int = 120_000_000,
    ) -> None:
        self._duration_ns = max(int(duration_sec * 1_000_000_000), 1)
        self._interp_max_gap_ns = max(interp_max_gap_ns, 1)
        self._max_extrapolate_ns = max(max_extrapolate_ns, 0)
        self._samples: Deque[OdomSample] = deque()
        self.last_extrapolate_ms: int = 0

    def __len__(self) -> int:
        return len(self._samples)

    def add_from_odometry(self, msg) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist
        sample = OdomSample(
            stamp_ns=stamp_to_ns(msg.header.stamp),
            position=np.array([pos.x, pos.y, pos.z], dtype=np.float64),
            rotation=Rotation.from_quat([ori.x, ori.y, ori.z, ori.w]).as_matrix(),
            vx=float(twist.linear.x),
            vy=float(twist.linear.y),
            vz=float(twist.linear.z),
            omega_z=float(twist.angular.z),
        )
        self._samples.append(sample)
        self._trim(sample.stamp_ns)

    def _trim(self, newest_ns: int) -> None:
        cutoff = newest_ns - self._duration_ns
        while self._samples and self._samples[0].stamp_ns < cutoff:
            self._samples.popleft()

    def pose_at(self, stamp_ns: int) -> Optional[np.ndarray]:
        """Return T_odom_base at stamp_ns, or None if unavailable."""
        self.last_extrapolate_ms = 0
        if not self._samples:
            return None

        if stamp_ns < self._samples[0].stamp_ns:
            gap = self._samples[0].stamp_ns - stamp_ns
            if gap > self._max_extrapolate_ns:
                return None
            dt_sec = -gap / 1e9
            sample = extrapolate_sample(self._samples[0], dt_sec)
            self.last_extrapolate_ms = gap // 1_000_000
            return sample.to_matrix()

        if stamp_ns > self._samples[-1].stamp_ns:
            gap = stamp_ns - self._samples[-1].stamp_ns
            if gap > self._max_extrapolate_ns:
                return None
            dt_sec = gap / 1e9
            sample = extrapolate_sample(self._samples[-1], dt_sec)
            self.last_extrapolate_ms = gap // 1_000_000
            return sample.to_matrix()

        stamps = [s.stamp_ns for s in self._samples]
        idx = int(np.searchsorted(stamps, stamp_ns))
        if idx == 0:
            return self._samples[0].to_matrix()
        if idx >= len(self._samples):
            return self._samples[-1].to_matrix()

        s0, s1 = self._samples[idx - 1], self._samples[idx]
        if s1.stamp_ns - s0.stamp_ns > self._interp_max_gap_ns:
            return None
        return _interpolate_samples(s0, s1, stamp_ns).to_matrix()
