"""Draw waypoints (and optional final pose) onto a SLAM map PNG."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

WAYPOINTS_PNG = 'slam_map_waypoints.png'
BASE_STEM = 'slam_map'
GT_CONFIG_NAME = 'paperclips_small_house.yaml'


def _parse_origin(origin_field) -> Tuple[float, float]:
    if isinstance(origin_field, list) and len(origin_field) >= 2:
        return float(origin_field[0]), float(origin_field[1])
    if isinstance(origin_field, dict):
        return float(origin_field.get('x', 0)), float(origin_field.get('y', 0))
    return 0.0, 0.0


def _resolve_gt_config_path() -> Optional[Path]:
    candidates: List[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('perception_pkg'))
        candidates.append(share / 'config' / GT_CONFIG_NAME)
        ws_root = share.parents[3]
        candidates.append(ws_root / 'src' / 'perception_pkg' / 'config' / GT_CONFIG_NAME)
    except Exception:
        pass
    here = Path(__file__).resolve()
    candidates.append(here.parents[1].parent / 'perception_pkg' / 'config' / GT_CONFIG_NAME)
    for path in candidates:
        if path.is_file():
            return path
    return None


def load_map_meta(session_dir: Path) -> Dict[str, Any]:
    yaml_path = session_dir / f'{BASE_STEM}.yaml'
    if not yaml_path.is_file():
        raise FileNotFoundError(f'map yaml not found: {yaml_path}')
    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'invalid map yaml: {yaml_path}')
    return data


def load_waypoints(session_dir: Path) -> List[Dict[str, Any]]:
    wp_path = session_dir / 'waypoints.yaml'
    if not wp_path.is_file():
        return []
    data = yaml.safe_load(wp_path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        return []
    wps = data.get('waypoints', [])
    return wps if isinstance(wps, list) else []


def load_initial_pose(session_dir: Path) -> Optional[Dict[str, float]]:
    pose_path = session_dir / 'initial_pose.yaml'
    if not pose_path.is_file():
        return None
    data = yaml.safe_load(pose_path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        return None
    if 'x' not in data or 'y' not in data:
        return None
    return {
        'x': float(data['x']),
        'y': float(data['y']),
        'yaw': float(data.get('yaw', 0.0)),
    }


def load_map_odom_offset(session_dir: Path) -> Tuple[float, float, float]:
    """Return (x, y, yaw) of odom origin expressed in map frame."""
    path = session_dir / 'map_odom_offset.yaml'
    if not path.is_file():
        return 0.0, 0.0, 0.0
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return (
        float(data.get('x', 0.0)),
        float(data.get('y', 0.0)),
        float(data.get('yaw', 0.0)),
    )


def map_odom_offset_present(session_dir: Path) -> bool:
    return (session_dir / 'map_odom_offset.yaml').is_file()


def load_paperclip_gt(gt_path: Optional[Path] = None) -> List[Dict[str, Any]]:
    path = gt_path or _resolve_gt_config_path()
    if path is None or not path.is_file():
        return []
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    clips = data.get('clips', [])
    return clips if isinstance(clips, list) else []


def odom_xy_to_map(
    x_odom: float,
    y_odom: float,
    offset_x: float,
    offset_y: float,
    offset_yaw: float,
) -> Tuple[float, float]:
    c = math.cos(offset_yaw)
    s = math.sin(offset_yaw)
    x_map = offset_x + c * x_odom - s * y_odom
    y_map = offset_y + s * x_odom + c * y_odom
    return x_map, y_map


def world_to_pixel(
    x: float,
    y: float,
    origin_xy: Tuple[float, float],
    resolution: float,
    height: int,
) -> Tuple[int, int]:
    px = int(round((x - origin_xy[0]) / resolution))
    py = int(round(height - 1 - (y - origin_xy[1]) / resolution))
    return px, py


def _load_base_bgr(session_dir: Path, meta: Dict[str, Any]) -> np.ndarray:
    """Load occupancy image referenced by slam_map.yaml (prefer authoritative PGM)."""
    image_name = str(meta.get('image', f'{BASE_STEM}.pgm'))
    primary = session_dir / image_name
    if primary.is_file():
        if primary.suffix.lower() == '.pgm':
            gray = cv2.imread(str(primary), cv2.IMREAD_GRAYSCALE)
            if gray is not None:
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            img = cv2.imread(str(primary), cv2.IMREAD_COLOR)
            if img is not None:
                return img
    png_path = session_dir / f'{BASE_STEM}.png'
    if png_path.is_file():
        img = cv2.imread(str(png_path), cv2.IMREAD_COLOR)
        if img is not None:
            return img
    raise FileNotFoundError(f'no map image for {session_dir} (expected {image_name})')


def render_waypoints_overlay(
    session_dir: Path,
    *,
    out_name: str = WAYPOINTS_PNG,
    draw_initial_pose: bool = True,
    draw_gt: bool = True,
    gt_config: Optional[Path] = None,
) -> Optional[Path]:
    """Write ``slam_map_waypoints.png``: green detections, red GT on top, blue pose."""
    session_dir = session_dir.expanduser().resolve()
    meta = load_map_meta(session_dir)
    waypoints = load_waypoints(session_dir)
    has_gt = draw_gt and bool(load_paperclip_gt(gt_config))
    if not waypoints and not has_gt:
        return None

    resolution = float(meta.get('resolution', 0.05))
    origin_xy = _parse_origin(meta.get('origin', [0, 0, 0]))
    img = _load_base_bgr(session_dir, meta)
    h, w = img.shape[:2]

    for wp in waypoints:
        px, py = world_to_pixel(float(wp['x']), float(wp['y']), origin_xy, resolution, h)
        if 0 <= px < w and 0 <= py < h:
            cv2.circle(img, (px, py), 3, (30, 220, 30), -1, lineType=cv2.LINE_AA)
            cv2.circle(img, (px, py), 3, (10, 120, 10), 1, lineType=cv2.LINE_AA)

    if draw_gt:
        if not map_odom_offset_present(session_dir):
            print(
                f'[map_overlay] WARNING: {session_dir / "map_odom_offset.yaml"} missing; '
                'GT crosses drawn with zero offset (misaligned). '
                'Save map with SLAM running to write map→odom offset.',
                flush=True,
            )
        ox, oy, oyaw = load_map_odom_offset(session_dir)
        for clip in load_paperclip_gt(gt_config):
            x_map, y_map = odom_xy_to_map(float(clip['x']), float(clip['y']), ox, oy, oyaw)
            px, py = world_to_pixel(x_map, y_map, origin_xy, resolution, h)
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(img, (px, py), 1, (40, 40, 240), -1, lineType=cv2.LINE_AA)

    if draw_initial_pose:
        pose = load_initial_pose(session_dir)
        if pose is not None:
            px, py = world_to_pixel(pose['x'], pose['y'], origin_xy, resolution, h)
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(img, (px, py), 8, (240, 120, 30), 2, lineType=cv2.LINE_AA)
                dx = int(14 * np.cos(pose['yaw']))
                dy = int(-14 * np.sin(pose['yaw']))
                cv2.arrowedLine(
                    img, (px, py), (px + dx, py + dy),
                    (240, 120, 30), 2, tipLength=0.35, line_type=cv2.LINE_AA,
                )

    out_path = session_dir / out_name
    cv2.imwrite(str(out_path), img)

    meta_path = session_dir / 'session_meta.json'
    if meta_path.is_file():
        try:
            session_meta = json.loads(meta_path.read_text(encoding='utf-8'))
        except json.JSONDecodeError:
            session_meta = {}
        session_meta['map_waypoints_png'] = out_name
        session_meta['waypoints_yaml'] = 'waypoints.yaml'
        session_meta['initial_pose_yaml'] = 'initial_pose.yaml'
        meta_path.write_text(
            json.dumps(session_meta, indent=2, ensure_ascii=False), encoding='utf-8')

    return out_path
