"""Footprint collision checks against occupancy grid costmaps."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

# Default footprint from nav2_params.yaml (base_footprint polygon)
DEFAULT_FOOTPRINT: List[Tuple[float, float]] = [
    (0.12, 0.10),
    (0.12, -0.10),
    (-0.12, -0.10),
    (-0.12, 0.10),
]

LETHAL_COST = 253
INSCRIBED_COST = 252


def rotate_footprint(
    footprint: Sequence[Tuple[float, float]],
    x: float,
    y: float,
    yaw: float,
) -> List[Tuple[float, float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [(x + fx * c - fy * s, y + fx * s + fy * c) for fx, fy in footprint]


def world_to_map(
    wx: float,
    wy: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[int, int]:
    mx = int((wx - origin_x) / resolution)
    my = int((wy - origin_y) / resolution)
    return mx, my


def cost_at(
    mx: int,
    my: int,
    width: int,
    height: int,
    data: Sequence[int],
) -> Optional[int]:
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return LETHAL_COST
    return int(data[my * width + mx])


def footprint_free(
    x: float,
    y: float,
    yaw: float,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    data: Sequence[int],
    footprint: Sequence[Tuple[float, float]] = DEFAULT_FOOTPRINT,
    max_cost: int = INSCRIBED_COST,
) -> bool:
    """Return True if all footprint corners are below max_cost."""
    return footprint_max_cost(
        x, y, yaw, width, height, resolution, origin_x, origin_y, data, footprint
    ) < max_cost


def footprint_max_cost(
    x: float,
    y: float,
    yaw: float,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    data: Sequence[int],
    footprint: Sequence[Tuple[float, float]] = DEFAULT_FOOTPRINT,
) -> int:
    """Maximum occupancy cost under footprint corners."""
    worst = 0
    for wx, wy in rotate_footprint(footprint, x, y, yaw):
        mx, my = world_to_map(wx, wy, origin_x, origin_y, resolution)
        c = cost_at(mx, my, width, height, data)
        if c is None:
            return LETHAL_COST
        worst = max(worst, c)
    return worst
