"""Stereo back-projection: image bbox center -> map-frame waypoint.

Geometry follows docs/SIM_DATASET_PROJECTION_FIX.md (REP-103 post-multiply).
Independent from scripts/generate_sim_dataset.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# REP-103: camera_link -> camera_optical_frame (Gazebo Classic + OpenCV).
CAMERA_OPTICAL_FIX = Rotation.from_euler('xyz', [-np.pi / 2, 0, -np.pi / 2]).as_matrix()


@dataclass
class DetectionBox:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int = 0

    @property
    def center_u(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def center_v(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def area(self) -> float:
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


@dataclass
class MapWaypoint:
    x: float
    y: float
    confidence: float
    source: str  # stereo | ground | ground_fallback


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def camera_matrix_from_info(k: Sequence[float]) -> np.ndarray:
    return np.array([
        [k[0], k[1], k[2]],
        [k[3], k[4], k[5]],
        [k[6], k[7], k[8]],
    ], dtype=np.float64)


def pixel_to_ray_optical(u: float, v: float, camera_matrix: np.ndarray) -> np.ndarray:
    """Unit ray direction in optical frame (Z forward, X right, Y down)."""
    k_inv = np.linalg.inv(camera_matrix)
    ray = k_inv @ np.array([u, v, 1.0], dtype=np.float64)
    ray /= np.linalg.norm(ray)
    return ray


def ray_ground_intersection(
    origin: np.ndarray,
    direction: np.ndarray,
    z_ground: float,
) -> Optional[np.ndarray]:
    if abs(direction[2]) < 1e-9:
        return None
    scale = (z_ground - origin[2]) / direction[2]
    if scale <= 0.0:
        return None
    return origin + scale * direction


def camera_optical_origin(t_map_camera_link: np.ndarray) -> np.ndarray:
    """Optical-frame origin expressed in map coordinates."""
    t_map_optical = t_map_camera_link @ make_transform(CAMERA_OPTICAL_FIX, np.zeros(3))
    return t_map_optical[:3, 3].astype(np.float64)


def camera_range_in_map(point_map: np.ndarray, t_map_camera_link: np.ndarray) -> float:
    """Euclidean distance from camera optical center to map-frame 3D point."""
    origin = camera_optical_origin(t_map_camera_link)
    diff = point_map[:3] - origin
    return float(np.linalg.norm(diff))


def disparity_from_stereo(u_l: float, u_r: float) -> float:
    return float(u_l - u_r)


def disparity_at_depth(fx: float, baseline_m: float, depth_m: float) -> float:
    if depth_m <= 1e-6:
        return 0.0
    return fx * baseline_m / depth_m


def max_disparity_px(fx: float, baseline_m: float, min_depth_m: float) -> float:
    return disparity_at_depth(fx, baseline_m, min_depth_m)


def min_disparity_for_max_depth(fx: float, baseline_m: float, max_depth_m: float) -> float:
    return disparity_at_depth(fx, baseline_m, max_depth_m)


def validate_stereo_point(
    point_map: np.ndarray,
    u_l: float,
    u_r: float,
    fx: float,
    baseline_m: float,
    t_map_camera_link: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
    max_ground_z_error: float,
    min_disparity_px: float,
    ground_z: float = 0.002,
    max_map_radius_m: float = 15.0,
) -> Optional[float]:
    """Return camera range (m) if triangulation passes annular / sanity gates."""
    if not np.all(np.isfinite(point_map[:3])):
        return None

    disparity = disparity_from_stereo(u_l, u_r)
    disp_max = max_disparity_px(fx, baseline_m, min_depth_m)
    disp_min = min_disparity_for_max_depth(fx, baseline_m, max_depth_m)
    if disparity < min_disparity_px or disparity > disp_max:
        return None
    if disparity < disp_min:
        return None

    range_m = camera_range_in_map(point_map, t_map_camera_link)
    if range_m < min_depth_m or range_m > max_depth_m:
        return None

    if abs(float(point_map[2]) - ground_z) > max_ground_z_error:
        return None

    map_radius = math.hypot(float(point_map[0]), float(point_map[1]))
    if map_radius > max_map_radius_m:
        return None

    return range_m


def validate_map_point(
    point_map: np.ndarray,
    t_map_camera_link: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
    max_ground_z_error: float,
    ground_z: float = 0.002,
    max_map_radius_m: float = 15.0,
) -> Optional[float]:
    """Range gate for ground-plane or other non-stereo map points."""
    if not np.all(np.isfinite(point_map[:3])):
        return None
    range_m = camera_range_in_map(point_map, t_map_camera_link)
    if range_m < min_depth_m or range_m > max_depth_m:
        return None
    if abs(float(point_map[2]) - ground_z) > max_ground_z_error:
        return None
    map_radius = math.hypot(float(point_map[0]), float(point_map[1]))
    if map_radius > max_map_radius_m:
        return None
    return range_m


def triangulate_stereo(
    u_l: float,
    v_l: float,
    u_r: float,
    v_r: float,
    k_left: np.ndarray,
    k_right: np.ndarray,
    t_left_map: np.ndarray,
    t_right_map: np.ndarray,
) -> Optional[np.ndarray]:
    """Triangulate 3D point in map frame; P matrices use REP-103 optical frame."""

    def projection_matrix(t_map_camlink: np.ndarray, k: np.ndarray) -> np.ndarray:
        t_map_optical = t_map_camlink @ make_transform(CAMERA_OPTICAL_FIX, np.zeros(3))
        t_optical_map = np.linalg.inv(t_map_optical)
        r = t_optical_map[:3, :3]
        t = t_optical_map[:3, 3:4]
        return k @ np.hstack([r, t])

    p_left = projection_matrix(t_left_map, k_left)
    p_right = projection_matrix(t_right_map, k_right)
    pts = cv2.triangulatePoints(
        p_left, p_right,
        np.array([[u_l], [v_l]], dtype=np.float64),
        np.array([[u_r], [v_r]], dtype=np.float64),
    )
    if abs(float(pts[3, 0])) < 1e-9:
        return None
    point = (pts[:3, 0] / pts[3, 0]).astype(np.float64)
    if not np.all(np.isfinite(point)):
        return None
    return point


def optical_point_to_map(
    point_optical: np.ndarray,
    t_map_camera_link: np.ndarray,
) -> np.ndarray:
    t_map_optical = t_map_camera_link @ make_transform(CAMERA_OPTICAL_FIX, np.zeros(3))
    p_h = np.array([point_optical[0], point_optical[1], point_optical[2], 1.0])
    return (t_map_optical @ p_h)[:3]


def bbox_center_to_map(
    u: float,
    v: float,
    camera_matrix: np.ndarray,
    t_map_camera_link: np.ndarray,
    depth_m: Optional[float] = None,
    ground_z: float = 0.002,
) -> Optional[Tuple[float, float, str]]:
    """Back-project pixel to map XY. depth_m set => stereo; else ground plane."""
    ray_optical = pixel_to_ray_optical(u, v, camera_matrix)
    t_map_optical = t_map_camera_link @ make_transform(CAMERA_OPTICAL_FIX, np.zeros(3))
    origin = t_map_optical[:3, 3]
    direction = t_map_optical[:3, :3] @ ray_optical

    if depth_m is not None and depth_m > 0.05:
        point_optical = ray_optical * depth_m
        point_map = optical_point_to_map(point_optical, t_map_camera_link)
        return float(point_map[0]), float(point_map[1]), 'stereo'

    hit = ray_ground_intersection(origin, direction, ground_z)
    if hit is None:
        return None
    return float(hit[0]), float(hit[1]), 'ground'


def match_stereo_detections(
    left_boxes: Sequence[DetectionBox],
    right_boxes: Sequence[DetectionBox],
    row_tolerance_px: float = 8.0,
    area_ratio_min: float = 0.5,
    area_ratio_max: float = 2.0,
    min_disparity_px: float = 2.0,
    max_disparity_px: float = 1e6,
) -> List[Tuple[DetectionBox, DetectionBox]]:
    """Greedy stereo pairing: left u > right u, similar row, area, and disparity."""
    pairs: List[Tuple[DetectionBox, DetectionBox]] = []
    used_right: set[int] = set()

    for left in sorted(left_boxes, key=lambda b: -b.confidence):
        best_idx = -1
        best_score = -1.0
        for idx, right in enumerate(right_boxes):
            if idx in used_right:
                continue
            if abs(left.center_v - right.center_v) > row_tolerance_px:
                continue
            if left.center_u <= right.center_u:
                continue
            disparity = disparity_from_stereo(left.center_u, right.center_u)
            if disparity < min_disparity_px or disparity > max_disparity_px:
                continue
            area_ratio = left.area / max(right.area, 1e-6)
            if area_ratio < area_ratio_min or area_ratio > area_ratio_max:
                continue
            row_penalty = abs(left.center_v - right.center_v) / max(row_tolerance_px, 1.0)
            score = left.confidence + right.confidence - 0.05 * row_penalty
            if score > best_score:
                best_score = score
                best_idx = idx
        if best_idx >= 0:
            used_right.add(best_idx)
            pairs.append((left, right_boxes[best_idx]))
    return pairs


def stereo_depth_from_disparity(
    u_l: float,
    u_r: float,
    fx: float,
    baseline_m: float,
) -> Optional[float]:
    disparity = u_l - u_r
    if disparity <= 0.5:
        return None
    return fx * baseline_m / disparity


def deduplicate_waypoints(
    waypoints: Sequence[MapWaypoint],
    radius_m: float = 0.12,
) -> List[MapWaypoint]:
    """Cluster by distance; keep highest-confidence per cluster."""
    sorted_wps = sorted(waypoints, key=lambda w: -w.confidence)
    kept: List[MapWaypoint] = []
    for wp in sorted_wps:
        if all(math.hypot(wp.x - k.x, wp.y - k.y) >= radius_m for k in kept):
            kept.append(wp)
    return kept


def nms_detection_boxes(
    boxes: Sequence[DetectionBox],
    iou_threshold: float = 0.45,
) -> List[DetectionBox]:
    """Greedy NMS on image boxes (same frame duplicates)."""
    sorted_boxes = sorted(boxes, key=lambda b: -b.confidence)
    kept: List[DetectionBox] = []

    def iou(a: DetectionBox, b: DetectionBox) -> float:
        x1 = max(a.x1, b.x1)
        y1 = max(a.y1, b.y1)
        x2 = min(a.x2, b.x2)
        y2 = min(a.y2, b.y2)
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if inter <= 0.0:
            return 0.0
        area_a = a.area
        area_b = b.area
        return inter / max(area_a + area_b - inter, 1e-6)

    for box in sorted_boxes:
        if all(iou(box, k) < iou_threshold for k in kept):
            kept.append(box)
    return kept
