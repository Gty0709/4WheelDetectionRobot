"""Render patrol trajectory onto the SLAM map as PNG."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from navigation_pkg.trajectory_tracker import filter_trajectory_outliers

MISSION_TRAJECTORY_PNG = 'mission_trajectory.png'


def _parse_origin(origin_field) -> Tuple[float, float]:
    if isinstance(origin_field, list) and len(origin_field) >= 2:
        return float(origin_field[0]), float(origin_field[1])
    if isinstance(origin_field, dict):
        return float(origin_field.get('x', 0)), float(origin_field.get('y', 0))
    return 0.0, 0.0


def _world_to_pixel(
    x: float,
    y: float,
    origin_xy: Tuple[float, float],
    resolution: float,
    height: int,
) -> Tuple[int, int]:
    px = int(round((x - origin_xy[0]) / resolution))
    py = int(round(height - 1 - (y - origin_xy[1]) / resolution))
    return px, py


def _load_map_bgr(session_dir: Path) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    yaml_path = session_dir / 'slam_map.yaml'
    with open(yaml_path, encoding='utf-8') as f:
        meta = yaml.safe_load(f) or {}
    resolution = float(meta.get('resolution', 0.05))
    origin_xy = _parse_origin(meta.get('origin', [0, 0, 0]))
    image_name = str(meta.get('image', 'slam_map.pgm'))
    for candidate in (session_dir / image_name, session_dir / 'slam_map.pgm', session_dir / 'slam_map.png'):
        if not candidate.is_file():
            continue
        if candidate.suffix.lower() == '.pgm':
            gray = cv2.imread(str(candidate), cv2.IMREAD_GRAYSCALE)
            if gray is not None:
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), resolution, origin_xy
        img = cv2.imread(str(candidate), cv2.IMREAD_COLOR)
        if img is not None:
            return img, resolution, origin_xy
    raise FileNotFoundError(f'no map image under {session_dir}')


def render_mission_trajectory_png(
    map_session_dir: Path,
    out_path: Path,
    trajectory: Sequence[dict],
    waypoints: Sequence[dict],
    visited_ids: Optional[Iterable[int]] = None,
) -> Path:
    """Draw trajectory line + waypoint spheres on the SLAM map."""
    visited = set(visited_ids or [])
    img, resolution, origin_xy = _load_map_bgr(map_session_dir)
    h, w = img.shape[:2]

    trajectory = filter_trajectory_outliers(list(trajectory))
    pts: List[Tuple[int, int]] = []
    for sample in trajectory:
        px, py = _world_to_pixel(
            float(sample['x']), float(sample['y']),
            origin_xy, resolution, h,
        )
        if 0 <= px < w and 0 <= py < h:
            pts.append((px, py))

    if len(pts) >= 2:
        cv2.polylines(img, [np.array(pts, dtype=np.int32)], False, (255, 180, 40), 2, cv2.LINE_AA)

    for wp in waypoints:
        wp_id = int(wp['id'])
        px, py = _world_to_pixel(float(wp['x']), float(wp['y']), origin_xy, resolution, h)
        if not (0 <= px < w and 0 <= py < h):
            continue
        if wp_id in visited:
            color, edge = (40, 220, 40), (20, 140, 20)
        else:
            color, edge = (40, 40, 220), (20, 20, 140)
        cv2.circle(img, (px, py), 5, color, -1, lineType=cv2.LINE_AA)
        cv2.circle(img, (px, py), 5, edge, 1, lineType=cv2.LINE_AA)

    if trajectory:
        sx, sy = _world_to_pixel(
            float(trajectory[0]['x']), float(trajectory[0]['y']),
            origin_xy, resolution, h,
        )
        ex, ey = _world_to_pixel(
            float(trajectory[-1]['x']), float(trajectory[-1]['y']),
            origin_xy, resolution, h,
        )
        if 0 <= sx < w and 0 <= sy < h:
            cv2.circle(img, (sx, sy), 7, (40, 200, 200), 2, lineType=cv2.LINE_AA)
        if len(trajectory) > 1 and 0 <= ex < w and 0 <= ey < h:
            yaw = float(trajectory[-1].get('yaw', 0.0))
            dx = int(16 * math.cos(yaw))
            dy = int(-16 * math.sin(yaw))
            cv2.arrowedLine(
                img, (ex, ey), (ex + dx, ey + dy),
                (200, 80, 40), 2, tipLength=0.35, line_type=cv2.LINE_AA,
            )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)
    return out_path
