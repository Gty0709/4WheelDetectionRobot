"""Odom-dead-reckoned trajectory in map frame (immune to AMCL pose jumps)."""

from __future__ import annotations

import math
from typing import Optional, Tuple

Pose3 = Tuple[float, float, float]


def _normalize_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def _delta_se2(
    x0: float, y0: float, yaw0: float,
    x1: float, y1: float, yaw1: float,
) -> Tuple[float, float, float]:
    dx = x1 - x0
    dy = y1 - y0
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return c * dx - s * dy, s * dx + c * dy, _normalize_yaw(yaw1 - yaw0)


def _compose_se2(
    x0: float, y0: float, yaw0: float,
    dx: float, dy: float, dyaw: float,
) -> Pose3:
    c, s = math.cos(yaw0), math.sin(yaw0)
    return (
        x0 + c * dx - s * dy,
        y0 + s * dx + c * dy,
        _normalize_yaw(yaw0 + dyaw),
    )


class OdomTrajectoryTracker:
    """Integrate /odom deltas, anchored once to map pose at patrol start."""

    def __init__(
        self,
        max_odom_step_m: float = 0.35,
        max_odom_yaw_step: float = 0.9,
    ):
        self._max_odom_step_m = max_odom_step_m
        self._max_odom_yaw_step = max_odom_yaw_step
        self._last_odom: Optional[Pose3] = None
        self._pose: Optional[Pose3] = None
        self._anchored = False

    @property
    def anchored(self) -> bool:
        return self._anchored

    def reset_anchor(self, map_pose: Pose3, odom_pose: Pose3) -> Pose3:
        self._pose = map_pose
        self._last_odom = odom_pose
        self._anchored = True
        return self._pose

    def update_odom(self, odom_pose: Pose3) -> Optional[Pose3]:
        if not self._anchored or self._pose is None or self._last_odom is None:
            return self._pose
        dx, dy, dyaw = _delta_se2(*self._last_odom, *odom_pose)
        if math.hypot(dx, dy) > self._max_odom_step_m or abs(dyaw) > self._max_odom_yaw_step:
            self._last_odom = odom_pose
            return self._pose
        self._pose = _compose_se2(*self._pose, dx, dy, dyaw)
        self._last_odom = odom_pose
        return self._pose


def filter_trajectory_outliers(
    samples: list,
    max_step_m: float = 0.45,
) -> list:
    """Remove AMCL jump spikes from a pose list with keys x,y."""
    if not samples:
        return []
    out = [samples[0]]
    for s in samples[1:]:
        prev = out[-1]
        if math.hypot(float(s['x']) - float(prev['x']), float(s['y']) - float(prev['y'])) <= max_step_m:
            out.append(s)
    return out
