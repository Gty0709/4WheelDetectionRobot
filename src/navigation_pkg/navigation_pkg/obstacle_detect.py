"""Detect dynamic obstacles from Nav2 costmaps vs saved SLAM static map."""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy.time
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from tf2_ros import Buffer

# Nav2 costmap_2d lethal / inscribed thresholds
INSCRIBED_INFLATED_OBSTACLE = 253


def _parse_origin(origin_field) -> Tuple[float, float]:
    if isinstance(origin_field, list) and len(origin_field) >= 2:
        return float(origin_field[0]), float(origin_field[1])
    if isinstance(origin_field, dict):
        return float(origin_field.get('x', 0)), float(origin_field.get('y', 0))
    return 0.0, 0.0


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _transform_xy(
    tf_buffer: Buffer,
    from_frame: str,
    to_frame: str,
    x: float,
    y: float,
) -> Optional[Tuple[float, float]]:
    if from_frame == to_frame:
        return x, y
    try:
        t = tf_buffer.lookup_transform(
            to_frame, from_frame, rclpy.time.Time(),
            timeout=Duration(seconds=0.15),
        )
    except Exception:
        return None
    tx = t.transform.translation.x
    ty = t.transform.translation.y
    yaw = _quat_to_yaw(t.transform.rotation)
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y + tx, s * x + c * y + ty


class StaticMapIndex:
    """Occupancy lookup for the saved SLAM map (map frame)."""

    def __init__(self, session_dir: Path):
        yaml_path = session_dir / 'slam_map.yaml'
        with open(yaml_path, encoding='utf-8') as f:
            meta = yaml.safe_load(f) or {}
        self.resolution = float(meta.get('resolution', 0.05))
        self.origin_x, self.origin_y = _parse_origin(meta.get('origin', [0, 0, 0]))
        self.occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        self.negate = int(meta.get('negate', 0))
        image_name = str(meta.get('image', 'slam_map.pgm'))
        img_path = session_dir / image_name
        if not img_path.is_file():
            img_path = session_dir / 'slam_map.pgm'
        gray = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise FileNotFoundError(f'cannot load static map image: {img_path}')
        self._gray = gray
        self.height, self.width = gray.shape[:2]

    def is_occupied(self, wx: float, wy: float) -> bool:
        px = int((wx - self.origin_x) / self.resolution)
        py = int(self.height - 1 - (wy - self.origin_y) / self.resolution)
        if px < 0 or py < 0 or px >= self.width or py >= self.height:
            return True
        val = int(self._gray[py, px])
        if self.negate:
            occ_prob = val / 255.0
        else:
            occ_prob = (255 - val) / 255.0
        return occ_prob >= self.occupied_thresh


def extract_dynamic_obstacles(
    costmap: OccupancyGrid,
    static_map: StaticMapIndex,
    tf_buffer: Optional[Buffer] = None,
    target_frame: str = 'map',
    min_cost: int = INSCRIBED_INFLATED_OBSTACLE,
    cell_stride: int = 2,
) -> List[Tuple[float, float]]:
    """Return map-frame points where costmap marks obstacle but static SLAM map is free."""
    info = costmap.info
    data: Sequence[int] = costmap.data
    w, h = info.width, info.height
    if w == 0 or h == 0 or not data:
        return []

    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    src_frame = costmap.header.frame_id or 'map'
    out: List[Tuple[float, float]] = []

    for my in range(0, h, cell_stride):
        row = my * w
        for mx in range(0, w, cell_stride):
            cost = data[row + mx]
            if cost < min_cost:
                continue
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            if tf_buffer is not None and src_frame != target_frame:
                mapped = _transform_xy(tf_buffer, src_frame, target_frame, wx, wy)
                if mapped is None:
                    continue
                wx, wy = mapped
            elif src_frame != target_frame:
                continue
            if not static_map.is_occupied(wx, wy):
                out.append((wx, wy))
    return out


# Gazebo 仿真新增障碍（不在 SLAM 静态地图中），用于 PNG 补全显示
GAZEBO_DYNAMIC_OBSTACLE_XY: List[Tuple[float, float]] = [
    (-3.0, 2.5),
    (3.0, 2.5),
    (-3.0, -2.5),
    (3.0, -2.5),
]


def obstacles_for_png(
    recorded: Sequence[dict],
    static_map: StaticMapIndex,
    merge_grid_m: float = 0.25,
) -> List[dict]:
    """Merge local-costmap detections with known dynamic obstacles for PNG overlay."""
    merged: List[dict] = [dict(o) for o in recorded]
    seen = {
        (int(round(float(o['x']) / merge_grid_m)), int(round(float(o['y']) / merge_grid_m)))
        for o in merged
    }
    for x, y in GAZEBO_DYNAMIC_OBSTACLE_XY:
        if static_map.is_occupied(x, y):
            continue
        key = (int(round(x / merge_grid_m)), int(round(y / merge_grid_m)))
        if key in seen:
            continue
        merged.append({'x': x, 'y': y, 'source': 'local_costmap_or_gazebo'})
        seen.add(key)
    return merged
