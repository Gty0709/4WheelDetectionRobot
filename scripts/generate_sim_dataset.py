#!/usr/bin/env python3
"""Generate YOLO26 training dataset from rosbag images + known paperclip world poses."""

from __future__ import annotations

import argparse
import json
import random
import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]

TOPIC_LEFT_IMAGE = '/camera/left/image_raw'
TOPIC_RIGHT_IMAGE = '/camera/right/image_raw'
TOPIC_LEFT_INFO = '/camera/left/camera_info'
TOPIC_RIGHT_INFO = '/camera/right/camera_info'
TOPIC_ODOM = '/odom'
TOPIC_TF_STATIC = '/tf_static'

CLASS_ID = 0
CLASS_NAME = 'clip'

# REP-103 camera_link -> camera_optical_frame for Gazebo Classic + OpenCV projection.
CAMERA_OPTICAL_FIX = Rotation.from_euler('xyz', [-np.pi / 2, 0, -np.pi / 2]).as_matrix()

# 2D occluders from small_house.world (ground-plane ray cast, meters).
SMALL_HOUSE_INNER_WALL_X = -3.0
SMALL_HOUSE_INNER_WALL_HALF_W = 0.15 / 2.0
SMALL_HOUSE_INNER_WALL_HALF_LEN = 4.15 / 2.0
SMALL_HOUSE_BOUNDS = (-5.0, 5.0, -4.0, 4.0)  # xmin, xmax, ymin, ymax
SMALL_HOUSE_PILLARS = (
    (-3.0, 2.5, 0.5, 0.5),
    (3.0, 2.5, 0.5, 0.5),
    (-3.0, -2.5, 0.5, 0.5),
    (3.0, -2.5, 0.5, 0.5),
)


@dataclass
class ClipGT:
    clip_id: int
    name: str
    center: np.ndarray
    corners: np.ndarray  # (4, 3)


@dataclass
class OdomSample:
    stamp_ns: int
    position: np.ndarray
    rotation: np.ndarray  # 3x3


@dataclass
class FrameRecord:
    stamp_ns: int
    camera: str
    image: np.ndarray
    labels: List[Tuple[float, float, float, float]]


@dataclass
class Stats:
    total_images_seen: int = 0
    sampled_frames: int = 0
    skipped_no_odom: int = 0
    skipped_empty_labels: int = 0
    boxes_written: int = 0
    filter_reasons: Dict[str, int] = field(default_factory=dict)

    def bump_filter(self, reason: str) -> None:
        self.filter_reasons[reason] = self.filter_reasons.get(reason, 0) + 1


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def slerp_rotation(r0: np.ndarray, r1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = Rotation.from_matrix(r0).as_quat()
    q1 = Rotation.from_matrix(r1).as_quat()
    return Rotation.from_quat(_slerp_quat(q0, q1, alpha)).as_matrix()


def _slerp_quat(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        out = q0 + alpha * (q1 - q0)
        return out / np.linalg.norm(out)
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta_0 * alpha
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    height, width = msg.height, msg.width
    encoding = msg.encoding.lower()
    if encoding == 'rgb8':
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if encoding == 'bgr8':
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3).copy()
    if encoding == 'rgba8':
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    if encoding == 'mono8':
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'Unsupported image encoding: {msg.encoding}')


def load_clips_config(path: Path) -> Tuple[List[ClipGT], Tuple[float, float]]:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    half_x, half_y = 0.5 * data['footprint'][0], 0.5 * data['footprint'][1]
    local_corners = np.array([
        [-half_x, -half_y, 0.0],
        [half_x, -half_y, 0.0],
        [half_x, half_y, 0.0],
        [-half_x, half_y, 0.0],
    ], dtype=np.float64)

    clips: List[ClipGT] = []
    for item in data['clips']:
        yaw = float(item['yaw'])
        rot = Rotation.from_euler('z', yaw).as_matrix()
        center = np.array([item['x'], item['y'], item['z']], dtype=np.float64)
        corners = (rot @ local_corners.T).T + center
        clips.append(ClipGT(
            clip_id=int(item['id']),
            name=str(item['name']),
            center=center,
            corners=corners,
        ))
    return clips, tuple(data['footprint'])


def parse_world_clips(world_path: Path) -> None:
    text = world_path.read_text(encoding='utf-8')
    pattern = re.compile(
        r'<model name="(paperclip_\d+)">\s*<static>1</static>\s*'
        r'<pose>([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+)</pose>'
    )
    clips = []
    for idx, match in enumerate(pattern.finditer(text), start=1):
        clips.append({
            'id': idx,
            'name': match.group(1),
            'x': float(match.group(2)),
            'y': float(match.group(3)),
            'z': float(match.group(4)),
            'yaw': float(match.group(6)),
        })
    out = {'frame': 'odom', 'footprint': [0.14, 0.14], 'clips': clips}
    print(yaml.dump(out, allow_unicode=True, sort_keys=False))


class OdomBuffer:
    def __init__(self, tolerance_ns: int) -> None:
        self.tolerance_ns = tolerance_ns
        self.samples: List[OdomSample] = []

    def add(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.samples.append(OdomSample(
            stamp_ns=stamp_to_ns(msg.header.stamp),
            position=np.array([position.x, position.y, position.z], dtype=np.float64),
            rotation=Rotation.from_quat([
                orientation.x, orientation.y, orientation.z, orientation.w,
            ]).as_matrix(),
        ))

    def finalize(self) -> None:
        self.samples.sort(key=lambda sample: sample.stamp_ns)

    def lookup(self, stamp_ns: int) -> Optional[OdomSample]:
        if not self.samples:
            return None
        if stamp_ns < self.samples[0].stamp_ns - self.tolerance_ns:
            return None
        if stamp_ns > self.samples[-1].stamp_ns + self.tolerance_ns:
            return None

        stamps = [sample.stamp_ns for sample in self.samples]
        idx = int(np.searchsorted(stamps, stamp_ns))
        if idx == 0:
            return self.samples[0] if abs(stamps[0] - stamp_ns) <= self.tolerance_ns else None
        if idx >= len(self.samples):
            last = self.samples[-1]
            return last if abs(last.stamp_ns - stamp_ns) <= self.tolerance_ns else None

        s0, s1 = self.samples[idx - 1], self.samples[idx]
        if stamp_ns < s0.stamp_ns - self.tolerance_ns or stamp_ns > s1.stamp_ns + self.tolerance_ns:
            return None
        if s1.stamp_ns == s0.stamp_ns:
            return s0
        alpha = (stamp_ns - s0.stamp_ns) / (s1.stamp_ns - s0.stamp_ns)
        return OdomSample(
            stamp_ns=stamp_ns,
            position=(1.0 - alpha) * s0.position + alpha * s1.position,
            rotation=slerp_rotation(s0.rotation, s1.rotation, alpha),
        )


def load_static_camera_extrinsics(bag_path: Path) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    static: Dict[Tuple[str, str], Tuple[np.ndarray, np.ndarray]] = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != TOPIC_TF_STATIC:
            continue
        message = deserialize_message(data, TFMessage)
        for transform in message.transforms:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            static[(transform.header.frame_id, transform.child_frame_id)] = (
                np.array([translation.x, translation.y, translation.z], dtype=np.float64),
                Rotation.from_quat([
                    rotation.x, rotation.y, rotation.z, rotation.w,
                ]).as_matrix(),
            )
        break

    missing = []
    for camera in ('left', 'right'):
        key = ('base_link', f'camera_{camera}_link')
        if key not in static:
            missing.append(key)
    if missing:
        raise RuntimeError(f'Missing static TF in bag: {missing}')
    return {
        camera: static[('base_link', f'camera_{camera}_link')]
        for camera in ('left', 'right')
    }


def build_rvec_tvec(
    odom: OdomSample,
    camera_extrinsic: Tuple[np.ndarray, np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    translation, rotation = camera_extrinsic
    transform_odom_camera = (
        make_transform(odom.rotation, odom.position)
        @ make_transform(rotation, translation)
    )
    transform_world_camera = transform_odom_camera @ make_transform(
        CAMERA_OPTICAL_FIX, np.zeros(3),
    )
    rotation_cam_world = transform_world_camera[:3, :3].T
    translation_cam_world = -rotation_cam_world @ transform_world_camera[:3, 3]
    rvec, _ = cv2.Rodrigues(rotation_cam_world)
    return rvec, translation_cam_world.reshape(3, 1)


def camera_position_world(
    odom: OdomSample,
    camera_extrinsic: Tuple[np.ndarray, np.ndarray],
) -> np.ndarray:
    translation, _ = camera_extrinsic
    return odom.position + odom.rotation @ translation


def _segment_y_at_x(
    cam_xy: np.ndarray,
    target_xy: np.ndarray,
    wall_x: float,
) -> Optional[float]:
    cx, cy = cam_xy
    tx, ty = target_xy
    if abs(tx - cx) < 1e-9:
        return None
    return cy + (ty - cy) * (wall_x - cx) / (tx - cx)


def _segment_x_at_y(
    cam_xy: np.ndarray,
    target_xy: np.ndarray,
    wall_y: float,
) -> Optional[float]:
    cx, cy = cam_xy
    tx, ty = target_xy
    if abs(ty - cy) < 1e-9:
        return None
    return cx + (tx - cx) * (wall_y - cy) / (ty - cy)


def _segment_intersects_aabb(
    cam_xy: np.ndarray,
    target_xy: np.ndarray,
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
) -> bool:
    xmin = center_x - size_x / 2.0
    xmax = center_x + size_x / 2.0
    ymin = center_y - size_y / 2.0
    ymax = center_y + size_y / 2.0
    dx = target_xy[0] - cam_xy[0]
    dy = target_xy[1] - cam_xy[1]
    t0, t1 = 0.0, 1.0
    for p, q, lo, hi in (
        (-dx, cam_xy[0] - xmin, 0.0, dx),
        (dx, xmax - cam_xy[0], 0.0, dx),
        (-dy, cam_xy[1] - ymin, 0.0, dy),
        (dy, ymax - cam_xy[1], 0.0, dy),
    ):
        if abs(p) < 1e-9:
            if q < 0.0:
                return False
            continue
        t_enter = q / p
        t_exit = (q - hi) / p
        if p < 0.0:
            t_enter, t_exit = t_exit, t_enter
        t0 = max(t0, t_enter)
        t1 = min(t1, t_exit)
        if t0 > t1:
            return False
    return t0 < 1.0


def clip_occluded_in_small_house(cam_xy: np.ndarray, target_xy: np.ndarray) -> bool:
    """Return True when walls/pillars block the camera-to-clip ground ray."""
    xmin, xmax, ymin, ymax = SMALL_HOUSE_BOUNDS
    cx, tx = cam_xy[0], target_xy[0]
    cy, ty = cam_xy[1], target_xy[1]

    wall_west = SMALL_HOUSE_INNER_WALL_X - SMALL_HOUSE_INNER_WALL_HALF_W
    wall_east = SMALL_HOUSE_INNER_WALL_X + SMALL_HOUSE_INNER_WALL_HALF_W

    def inner_room_side(x: float) -> int:
        if x < wall_west:
            return -1
        if x > wall_east:
            return 1
        return 0

    cam_side = inner_room_side(cx)
    tgt_side = inner_room_side(tx)
    if cam_side != 0 and tgt_side != 0 and cam_side != tgt_side:
        y_at = _segment_y_at_x(cam_xy, target_xy, SMALL_HOUSE_INNER_WALL_X)
        if y_at is not None and abs(y_at) <= SMALL_HOUSE_INNER_WALL_HALF_LEN:
            return True
        cross_dist = float(np.linalg.norm(target_xy - cam_xy))
        if (
            abs(cy) > SMALL_HOUSE_INNER_WALL_HALF_LEN
            and abs(ty) > SMALL_HOUSE_INNER_WALL_HALF_LEN
            and cross_dist > 5.0
        ):
            return True

    if (cx > xmin and tx < xmin) or (cx < xmin and tx > xmin):
        y_at = _segment_y_at_x(cam_xy, target_xy, xmin)
        if y_at is not None and ymin <= y_at <= ymax:
            return True
    if (cx < xmax and tx > xmax) or (cx > xmax and tx < xmax):
        y_at = _segment_y_at_x(cam_xy, target_xy, xmax)
        if y_at is not None and ymin <= y_at <= ymax:
            return True
    if (cy > ymin and ty < ymin) or (cy < ymin and ty > ymin):
        x_at = _segment_x_at_y(cam_xy, target_xy, ymin)
        if x_at is not None and xmin <= x_at <= xmax:
            return True
    if (cy < ymax and ty > ymax) or (cy > ymax and ty < ymax):
        x_at = _segment_x_at_y(cam_xy, target_xy, ymax)
        if x_at is not None and xmin <= x_at <= xmax:
            return True

    for pillar in SMALL_HOUSE_PILLARS:
        if _segment_intersects_aabb(cam_xy, target_xy, *pillar):
            return True
    return False


def clip_has_floor_texture(
    image: np.ndarray,
    u_min: float,
    v_min: float,
    u_max: float,
    v_max: float,
    min_box_height: float,
    min_texture_width: float,
    max_dark_fraction: float,
) -> bool:
    """Reject empty floor or uniformly dark wall patches; keep light-gray clips."""
    height, width = image.shape[:2]
    center_u = 0.5 * (u_min + u_max)
    center_v = 0.5 * (v_min + v_max)
    half_w = max(min_texture_width / 2.0, 0.5 * (u_max - u_min))
    half_h = max(min_box_height / 2.0, 0.5 * (v_max - v_min))
    x1 = max(0, int(center_u - half_w))
    y1 = max(0, int(center_v - half_h))
    x2 = min(width, int(np.ceil(center_u + half_w)))
    y2 = min(height, int(np.ceil(center_v + half_h)))
    if x2 <= x1 or y2 <= y1:
        return False

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    patch = gray[y1:y2, x1:x2]
    if patch.size == 0:
        return False

    dark_fraction = float(np.mean(patch < 120))
    if max_dark_fraction > 0.0 and dark_fraction > max_dark_fraction:
        return False

    floor = gray
    bg = floor[max(0, y1 - 25):y1, x1:x2]
    if bg.size == 0:
        bg = floor[y2:min(height, y2 + 25), x1:x2]
    contrast = abs(float(patch.mean()) - float(bg.mean())) if bg.size else 0.0
    texture_energy = float(cv2.Laplacian(patch, cv2.CV_64F).var())
    variation = float(np.std(patch))
    return variation >= 4.0 and (contrast >= 10.0 or texture_energy >= 80.0)


def camera_matrix_from_info(msg: CameraInfo) -> Tuple[np.ndarray, int, int]:
    camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    return camera_matrix, int(msg.width), int(msg.height)


def default_camera_matrix(width: int = 640, height: int = 480, hfov: float = 1.047) -> np.ndarray:
    focal = width / (2.0 * np.tan(hfov / 2.0))
    return np.array([
        [focal, 0.0, width / 2.0],
        [0.0, focal, height / 2.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def clip_bbox_to_image(
    u_min: float,
    v_min: float,
    u_max: float,
    v_max: float,
    width: int,
    height: int,
    min_box_height: float,
) -> Optional[Tuple[float, float, float, float]]:
    if u_max <= u_min:
        return None
    box_height = v_max - v_min
    if box_height < min_box_height:
        center_v = 0.5 * (v_min + v_max)
        half_height = 0.5 * min_box_height
        v_min = center_v - half_height
        v_max = center_v + half_height
    u_min = max(0.0, min(float(width), u_min))
    u_max = max(0.0, min(float(width), u_max))
    v_min = max(0.0, min(float(height), v_min))
    v_max = max(0.0, min(float(height), v_max))
    if u_max <= u_min or v_max <= v_min:
        return None
    return u_min, v_min, u_max, v_max


def bbox_to_yolo(
    u_min: float,
    v_min: float,
    u_max: float,
    v_max: float,
    width: int,
    height: int,
) -> Tuple[float, float, float, float]:
    cx = (u_min + u_max) * 0.5 / width
    cy = (v_min + v_max) * 0.5 / height
    bw = (u_max - u_min) / width
    bh = (v_max - v_min) / height
    return cx, cy, bw, bh


def points_in_front_of_camera(
    points_world: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    min_depth: float = 0.1,
) -> bool:
    rotation, _ = cv2.Rodrigues(rvec)
    points_cam = (rotation @ points_world.T + tvec.reshape(3, 1)).T
    return bool(np.all(points_cam[:, 2] > min_depth))


def project_clips(
    clips: Sequence[ClipGT],
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    width: int,
    height: int,
    min_box_area: float,
    min_norm_size: float,
    min_box_height: float,
    max_distance: float,
    robot_xy: np.ndarray,
    camera_xy: np.ndarray,
    image: Optional[np.ndarray],
    min_texture_width: float,
    max_texture_fraction: float,
    stats: Stats,
) -> List[Tuple[float, float, float, float]]:
    labels: List[Tuple[float, float, float, float]] = []
    distortion = np.zeros(5, dtype=np.float64)

    for clip in clips:
        distance = float(np.linalg.norm(clip.center[:2] - camera_xy))
        if distance > max_distance:
            stats.bump_filter('too_far')
            continue

        if clip_occluded_in_small_house(camera_xy, clip.center[:2]):
            stats.bump_filter('occluded')
            continue

        if not points_in_front_of_camera(clip.corners, rvec, tvec):
            stats.bump_filter('behind_camera')
            continue

        image_points, _ = cv2.projectPoints(
            clip.corners.astype(np.float64),
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        pixels = image_points.reshape(-1, 2)
        if np.any(~np.isfinite(pixels)):
            stats.bump_filter('invalid_projection')
            continue

        u_min, v_min = pixels.min(axis=0)
        u_max, v_max = pixels.max(axis=0)
        raw_width = max(0.0, u_max - u_min)
        raw_height = max(0.0, v_max - v_min)
        raw_area = raw_width * raw_height
        if u_max < 0 or v_max < 0 or u_min >= width or v_min >= height:
            stats.bump_filter('outside_image')
            continue

        if image is not None:
            if not clip_has_floor_texture(
                image,
                u_min,
                v_min,
                u_max,
                v_max,
                min_box_height,
                min_texture_width,
                max_texture_fraction,
            ):
                stats.bump_filter('no_texture')
                continue

        clipped = clip_bbox_to_image(
            u_min, v_min, u_max, v_max, width, height, min_box_height,
        )
        if clipped is None:
            stats.bump_filter('outside_image')
            continue
        u_min, v_min, u_max, v_max = clipped
        center_v = 0.5 * (v_min + v_max)
        if center_v < height * 0.32:
            stats.bump_filter('above_horizon')
            continue
        visible_area = (u_max - u_min) * (v_max - v_min)
        if raw_area > 0 and visible_area / raw_area < 0.15:
            stats.bump_filter('mostly_outside')
            continue
        if visible_area < min_box_area:
            stats.bump_filter('small_area')
            continue

        yolo = bbox_to_yolo(u_min, v_min, u_max, v_max, width, height)
        if yolo[2] < min_norm_size or yolo[3] < min_norm_size:
            stats.bump_filter('small_norm')
            continue
        labels.append(yolo)

    return labels


def open_bag_reader(bag_path: Path) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    return reader


def resolve_bag_path(bag_arg: Path) -> Path:
    bag_arg = bag_arg.resolve()
    if bag_arg.is_dir():
        return bag_arg
    if bag_arg.is_file() and bag_arg.suffix == '.db3':
        return bag_arg.parent
    raise FileNotFoundError(f'Bag path not found: {bag_arg}')


def preload_bag(
    bag_path: Path,
    cameras: Sequence[str],
) -> Tuple[OdomBuffer, Dict[str, np.ndarray], Dict[str, Tuple[int, int]], List[Tuple[str, int, bytes]]]:
    reader = open_bag_reader(bag_path)
    odom_buffer = OdomBuffer(tolerance_ns=50_000_000)
    camera_k: Dict[str, np.ndarray] = {}
    camera_size: Dict[str, Tuple[int, int]] = {}
    image_messages: List[Tuple[str, int, bytes]] = []

    image_topics = []
    if 'left' in cameras:
        image_topics.append(('left', TOPIC_LEFT_IMAGE, TOPIC_LEFT_INFO))
    if 'right' in cameras:
        image_topics.append(('right', TOPIC_RIGHT_IMAGE, TOPIC_RIGHT_INFO))
    image_topic_names = {topic[1] for topic in image_topics}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == TOPIC_ODOM:
            odom_buffer.add(deserialize_message(data, Odometry))
        elif topic in (TOPIC_LEFT_INFO, TOPIC_RIGHT_INFO):
            camera = 'left' if 'left' in topic else 'right'
            if camera not in camera_k:
                info = deserialize_message(data, CameraInfo)
                camera_k[camera], width, height = camera_matrix_from_info(info)
                camera_size[camera] = (width, height)
        elif topic in image_topic_names:
            message = deserialize_message(data, Image)
            image_messages.append((topic, stamp_to_ns(message.header.stamp), data))

    odom_buffer.finalize()

    for camera, _, _ in image_topics:
        if camera not in camera_k:
            camera_k[camera] = default_camera_matrix()
            camera_size[camera] = (640, 480)

    return odom_buffer, camera_k, camera_size, image_messages


def process_images(
    image_messages: List[Tuple[str, int, bytes]],
    odom_buffer: OdomBuffer,
    camera_k: Dict[str, np.ndarray],
    camera_size: Dict[str, Tuple[int, int]],
    camera_extrinsics: Dict[str, Tuple[np.ndarray, np.ndarray]],
    clips: Sequence[ClipGT],
    cameras: Sequence[str],
    sample_every: int,
    min_box_area: float,
    min_norm_size: float,
    min_box_height: float,
    max_distance: float,
    min_texture_width: float,
    max_texture_fraction: float,
    keep_empty: bool,
) -> Tuple[List[FrameRecord], Stats]:
    stats = Stats()
    frame_counters = {'left': 0, 'right': 0}
    topic_to_cam = {
        TOPIC_LEFT_IMAGE: 'left',
        TOPIC_RIGHT_IMAGE: 'right',
    }
    records: List[FrameRecord] = []

    for topic, stamp_ns, data in image_messages:
        camera = topic_to_cam.get(topic)
        if camera is None or camera not in cameras:
            continue
        stats.total_images_seen += 1
        frame_counters[camera] += 1
        if frame_counters[camera] % sample_every != 0:
            continue

        odom = odom_buffer.lookup(stamp_ns)
        if odom is None:
            stats.skipped_no_odom += 1
            continue

        message = deserialize_message(data, Image)
        image = image_msg_to_bgr(message)
        width, height = camera_size[camera]
        rvec, tvec = build_rvec_tvec(odom, camera_extrinsics[camera])
        cam_xy = camera_position_world(odom, camera_extrinsics[camera])[:2]
        labels = project_clips(
            clips=clips,
            rvec=rvec,
            tvec=tvec,
            camera_matrix=camera_k[camera],
            width=width,
            height=height,
            min_box_area=min_box_area,
            min_norm_size=min_norm_size,
            min_box_height=min_box_height,
            max_distance=max_distance,
            robot_xy=odom.position[:2],
            camera_xy=cam_xy,
            image=image,
            min_texture_width=min_texture_width,
            max_texture_fraction=max_texture_fraction,
            stats=stats,
        )

        if not labels and not keep_empty:
            stats.skipped_empty_labels += 1
            continue

        stats.sampled_frames += 1
        stats.boxes_written += len(labels)
        records.append(FrameRecord(
            stamp_ns=stamp_ns,
            camera=camera,
            image=image,
            labels=labels,
        ))

    records.sort(key=lambda record: (record.stamp_ns, record.camera))
    return records, stats


def split_records(
    records: Sequence[FrameRecord],
    train_ratio: float = 0.8,
    val_ratio: float = 0.1,
) -> Dict[str, List[FrameRecord]]:
    count = len(records)
    train_count = int(count * train_ratio)
    val_count = int(count * val_ratio)
    return {
        'train': list(records[:train_count]),
        'valid': list(records[train_count:train_count + val_count]),
        'test': list(records[train_count + val_count:]),
    }


def write_dataset(
    output_dir: Path,
    splits: Dict[str, List[FrameRecord]],
    preview_count: int,
    seed: int,
) -> None:
    if output_dir.exists():
        shutil.rmtree(output_dir)
    for split in ('train', 'valid', 'test'):
        (output_dir / split / 'images').mkdir(parents=True, exist_ok=True)
        (output_dir / split / 'labels').mkdir(parents=True, exist_ok=True)
    preview_dir = output_dir / 'preview'
    preview_dir.mkdir(parents=True, exist_ok=True)

    all_written: List[Tuple[Path, FrameRecord]] = []
    global_idx = 0
    for split_name, split_records in splits.items():
        for record in split_records:
            global_idx += 1
            stem = f'sim_{global_idx:06d}_{record.camera}'
            image_path = output_dir / split_name / 'images' / f'{stem}.jpg'
            label_path = output_dir / split_name / 'labels' / f'{stem}.txt'
            cv2.imwrite(str(image_path), record.image)
            lines = [
                f'{CLASS_ID} {cx:.6f} {cy:.6f} {box_w:.6f} {box_h:.6f}'
                for cx, cy, box_w, box_h in record.labels
            ]
            label_path.write_text('\n'.join(lines) + ('\n' if lines else ''), encoding='utf-8')
            all_written.append((image_path, record))

    rng = random.Random(seed)
    preview_samples = rng.sample(all_written, min(preview_count, len(all_written)))
    for image_path, record in preview_samples:
        vis = record.image.copy()
        img_h, img_w = vis.shape[:2]
        for cx, cy, box_w, box_h in record.labels:
            x1 = int((cx - box_w / 2) * img_w)
            y1 = int((cy - box_h / 2) * img_h)
            x2 = int((cx + box_w / 2) * img_w)
            y2 = int((cy + box_h / 2) * img_h)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        preview_path = preview_dir / f'{image_path.stem}_preview.jpg'
        cv2.imwrite(str(preview_path), vis)

    data_yaml = {
        'path': str(output_dir.resolve()),
        'train': 'train/images',
        'val': 'valid/images',
        'test': 'test/images',
        'nc': 1,
        'names': [CLASS_NAME],
    }
    (output_dir / 'data.yaml').write_text(
        yaml.dump(data_yaml, sort_keys=False, allow_unicode=True),
        encoding='utf-8',
    )


def write_stats(output_dir: Path, stats: Stats, splits: Dict[str, List[FrameRecord]]) -> None:
    split_counts = {name: len(records) for name, records in splits.items()}
    split_boxes = {
        name: sum(len(record.labels) for record in records)
        for name, records in splits.items()
    }
    non_empty = sum(1 for records in splits.values() for record in records if record.labels)
    total = sum(split_counts.values())
    payload = {
        'total_images_seen': stats.total_images_seen,
        'sampled_frames': stats.sampled_frames,
        'skipped_no_odom': stats.skipped_no_odom,
        'skipped_empty_labels': stats.skipped_empty_labels,
        'boxes_written': stats.boxes_written,
        'filter_reasons': stats.filter_reasons,
        'splits': split_counts,
        'boxes_per_split': split_boxes,
        'non_empty_label_frames': non_empty,
        'non_empty_ratio': round(non_empty / total, 4) if total else 0.0,
    }
    (output_dir / 'stats.json').write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Generate YOLO26 sim dataset from rosbag + paperclip GT')
    parser.add_argument(
        '--bag',
        type=Path,
        default=ROOT / 'src/perception_pkg/maps/bags/slam_20260613_161043',
    )
    parser.add_argument(
        '--clips-config',
        type=Path,
        default=ROOT / 'src/perception_pkg/config/paperclips_small_house.yaml',
    )
    parser.add_argument('--output', type=Path, default=ROOT / 'sim_dataset')
    parser.add_argument('--sample-every', type=int, default=5)
    parser.add_argument('--min-box-area', type=float, default=30.0)
    parser.add_argument('--min-norm-size', type=float, default=0.005)
    parser.add_argument('--min-box-height', type=float, default=12.0)
    parser.add_argument('--max-distance', type=float, default=7.5)
    parser.add_argument('--min-texture-width', type=float, default=40.0,
                        help='Min patch width in px for texture validation')
    parser.add_argument('--max-texture-fraction', type=float, default=0.88,
                        help='Reject uniformly dark wall-like patches above this fraction')
    parser.add_argument('--sync-tolerance-ms', type=float, default=50.0)
    parser.add_argument('--preview', type=int, default=50)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--cameras', choices=['both', 'left', 'right'], default='both')
    parser.add_argument('--keep-empty', action='store_true', help='Keep frames with zero visible clips')
    parser.add_argument('--parse-world', type=Path, default=None, help='Print YAML from world file and exit')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.parse_world:
        parse_world_clips(args.parse_world)
        return

    cameras = ['left', 'right'] if args.cameras == 'both' else [args.cameras]
    bag_path = resolve_bag_path(args.bag)
    clips, _ = load_clips_config(args.clips_config.resolve())
    camera_extrinsics = load_static_camera_extrinsics(bag_path)

    print(f'[generate_sim_dataset] bag={bag_path}')
    print(f'[generate_sim_dataset] clips={len(clips)} cameras={cameras}')

    odom_buffer, camera_k, camera_size, image_messages = preload_bag(bag_path, cameras)
    odom_buffer.tolerance_ns = int(args.sync_tolerance_ms * 1_000_000)
    print(
        f'[generate_sim_dataset] odom samples={len(odom_buffer.samples)} '
        f'images={len(image_messages)}'
    )

    records, stats = process_images(
        image_messages=image_messages,
        odom_buffer=odom_buffer,
        camera_k=camera_k,
        camera_size=camera_size,
        camera_extrinsics=camera_extrinsics,
        clips=clips,
        cameras=cameras,
        sample_every=max(1, args.sample_every),
        min_box_area=args.min_box_area,
        min_norm_size=args.min_norm_size,
        min_box_height=args.min_box_height,
        max_distance=args.max_distance,
        min_texture_width=args.min_texture_width,
        max_texture_fraction=args.max_texture_fraction,
        keep_empty=args.keep_empty,
    )
    print(f'[generate_sim_dataset] sampled={len(records)} boxes={stats.boxes_written}')

    splits = split_records(records)
    write_dataset(args.output.resolve(), splits, preview_count=args.preview, seed=args.seed)
    write_stats(args.output.resolve(), stats, splits)
    print(f'[generate_sim_dataset] done -> {args.output.resolve()}')


if __name__ == '__main__':
    main()
