#!/usr/bin/env python3
"""Validate stereo back-projection against paperclip ground truth from a rosbag.

Independent from scripts/generate_sim_dataset.py.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from detection_pkg.backprojection import (  # noqa: E402
    CAMERA_OPTICAL_FIX,
    bbox_center_to_map,
    make_transform,
    pixel_to_ray_optical,
    ray_ground_intersection,
)

TOPIC_ODOM = '/odom'
TOPIC_TF_STATIC = '/tf_static'
GT_PATH = ROOT / 'src/perception_pkg/config/paperclips_small_house.yaml'


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def load_gt(path: Path) -> list[tuple[str, np.ndarray]]:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    clips = []
    for item in data['clips']:
        center = np.array([item['x'], item['y'], item['z']], dtype=np.float64)
        clips.append((item['name'], center))
    return clips


def read_bag_static_tf(bag_path: Path) -> dict:
    from tf2_msgs.msg import TFMessage

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    static = {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != TOPIC_TF_STATIC:
            continue
        msg = deserialize_message(raw, TFMessage)
        for tf in msg.transforms:
            key = (tf.header.frame_id, tf.child_frame_id)
            t = tf.transform.translation
            q = tf.transform.rotation
            static[key] = (
                np.array([t.x, t.y, t.z]),
                Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
            )
    return static


def project_point(
    point_odom: np.ndarray,
    odom_pos: np.ndarray,
    odom_rot: np.ndarray,
    cam_trans: np.ndarray,
    cam_rot: np.ndarray,
    k: np.ndarray,
) -> tuple[float, float] | None:
    t_odom_cam = make_transform(odom_rot, odom_pos) @ make_transform(cam_rot, cam_trans)
    t_odom_opt = t_odom_cam @ make_transform(CAMERA_OPTICAL_FIX, np.zeros(3))
    p_h = np.array([point_odom[0], point_odom[1], point_odom[2], 1.0])
    p_cam = np.linalg.inv(t_odom_opt) @ p_h
    if p_cam[2] <= 0.05:
        return None
    uv = k @ (p_cam[:3] / p_cam[2])
    return float(uv[0]), float(uv[1])


def main() -> None:
    parser = argparse.ArgumentParser(description='Validate back-projection vs paperclip GT')
    parser.add_argument('--bag', required=True, help='Path to rosbag directory')
    parser.add_argument('--camera', default='left', choices=['left', 'right'])
    parser.add_argument('--ground-z', type=float, default=0.002)
    parser.add_argument('--fx', type=float, default=554.25)
    parser.add_argument('--fy', type=float, default=554.25)
    parser.add_argument('--cx', type=float, default=320.0)
    parser.add_argument('--cy', type=float, default=240.0)
    args = parser.parse_args()

    bag_path = Path(args.bag).resolve()
    static = read_bag_static_tf(bag_path)
    cam_key = ('base_link', f'camera_{args.camera}_link')
    if cam_key not in static:
        raise SystemExit(f'Missing TF {cam_key} in bag')
    cam_trans, cam_rot = static[cam_key]

    k = np.array([
        [args.fx, 0, args.cx],
        [0, args.fy, args.cy],
        [0, 0, 1],
    ], dtype=np.float64)

    # Use origin odom sample from first /odom in bag
    from nav_msgs.msg import Odometry
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    odom_pos = odom_rot = None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != TOPIC_ODOM:
            continue
        msg = deserialize_message(raw, Odometry)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        odom_pos = np.array([p.x, p.y, p.z])
        odom_rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        break
    if odom_pos is None:
        raise SystemExit('No /odom in bag')

    # map ~= odom at mapping start for sim
    t_map_cam = make_transform(odom_rot, odom_pos) @ make_transform(cam_rot, cam_trans)

    clips = load_gt(GT_PATH)
    errors = []
    for name, center in clips:
        uv = project_point(center, odom_pos, odom_rot, cam_trans, cam_rot, k)
        if uv is None:
            continue
        u, v = uv
        mapped = bbox_center_to_map(u, v, k, t_map_cam, ground_z=args.ground_z)
        if mapped is None:
            continue
        err = float(np.hypot(mapped[0] - center[0], mapped[1] - center[1]))
        errors.append((name, err))

    if not errors:
        raise SystemExit('No validation samples')

    dists = [e for _, e in errors]
    print(f'Samples: {len(errors)}')
    print(f'Median error (m): {np.median(dists):.4f}')
    print(f'Max error (m): {max(dists):.4f}')
    worst = sorted(errors, key=lambda x: -x[1])[:5]
    print('Worst 5:')
    for name, err in worst:
        print(f'  {name}: {err:.4f} m')


if __name__ == '__main__':
    main()
