"""Per-waypoint track: image anchor (u,v) + robust map-position fusion."""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional

import numpy as np

from detection_pkg.backprojection import MapWaypoint, bbox_center_to_map


@dataclass
class Observation:
    u: float
    v: float
    range_m: float
    map_x: float
    map_y: float
    confidence: float
    source: str


def _spatial_median(values: List[float]) -> float:
    if not values:
        return 0.0
    if len(values) == 1:
        return values[0]
    return float(statistics.median(values))


def _spatial_inliers(
    observations: List[Observation],
    sigma_floor_m: float,
) -> List[Observation]:
    """Keep observations whose map XY is within 1σ of the spatial median."""
    if len(observations) <= 1:
        return list(observations)

    mx = _spatial_median([o.map_x for o in observations])
    my = _spatial_median([o.map_y for o in observations])
    dists = [math.hypot(o.map_x - mx, o.map_y - my) for o in observations]
    mu = sum(dists) / len(dists)
    if len(dists) > 1:
        variance = sum((d - mu) ** 2 for d in dists) / len(dists)
        sigma = math.sqrt(variance)
    else:
        sigma = 0.0
    threshold = max(sigma, sigma_floor_m)
    inliers = [o for o, d in zip(observations, dists) if d <= threshold]
    return inliers if inliers else list(observations)


@dataclass
class WaypointTrack:
    """Rolling buffer; merges map XY with spatial-median inlier filter."""

    buffer_size: int = 30
    uv_ema: float = 0.15
    map_ema: float = 1.0
    observations: Deque[Observation] = field(default_factory=deque)
    anchor_u: Optional[float] = None
    anchor_v: Optional[float] = None

    def __post_init__(self) -> None:
        if not isinstance(self.observations, deque):
            self.observations = deque(self.observations, maxlen=self.buffer_size)
        elif self.observations.maxlen != self.buffer_size:
            self.observations = deque(self.observations, maxlen=self.buffer_size)

    def matches_pixel(self, u: float, v: float, tol_u: float, tol_v: float) -> bool:
        if self.anchor_u is None or self.anchor_v is None:
            return False
        return abs(u - self.anchor_u) <= tol_u and abs(v - self.anchor_v) <= tol_v

    def _update_anchor(self, obs: Observation) -> None:
        if self.anchor_u is None or self.anchor_v is None:
            self.anchor_u = obs.u
            self.anchor_v = obs.v
            return
        alpha = self.uv_ema
        self.anchor_u = (1.0 - alpha) * self.anchor_u + alpha * obs.u
        self.anchor_v = (1.0 - alpha) * self.anchor_v + alpha * obs.v

    def fuse_map_1sigma(
        self,
        obs: Observation,
        sigma_floor_m: float,
        prev: Optional[MapWaypoint] = None,
    ) -> MapWaypoint:
        """Spatial-median inlier filter on map XY, then fuse inliers."""
        self.observations.append(obs)
        self._update_anchor(obs)

        inliers = _spatial_inliers(list(self.observations), sigma_floor_m)

        mx = _spatial_median([o.map_x for o in inliers])
        my = _spatial_median([o.map_y for o in inliers])
        conf = max(o.confidence for o in inliers)
        source = inliers[-1].source

        if prev is not None and self.map_ema < 1.0:
            alpha = self.map_ema
            mx = (1.0 - alpha) * prev.x + alpha * mx
            my = (1.0 - alpha) * prev.y + alpha * my

        return MapWaypoint(x=mx, y=my, confidence=conf, source=source)


def observation_from_detection(
    u: float,
    v: float,
    range_m: float,
    confidence: float,
    source: str,
    camera_matrix: np.ndarray,
    t_map_camera: np.ndarray,
) -> Optional[Observation]:
    mapped = bbox_center_to_map(u, v, camera_matrix, t_map_camera, depth_m=range_m)
    if mapped is None:
        return None
    return Observation(
        u=u, v=v, range_m=range_m,
        map_x=mapped[0], map_y=mapped[1],
        confidence=confidence, source=source,
    )
