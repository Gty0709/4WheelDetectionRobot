#!/usr/bin/env python3
"""Diagnose mapping bag: cmd_vel vs odom yaw, map->odom correction."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def analyze(bag_dir: Path) -> int:
    rclpy.init()
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        ConverterOptions('', ''),
    )
    cmd: list[tuple[float, float, float]] = []
    odom: list[tuple[float, float, float, float]] = []
    map_odom: list[tuple[float, float, float, float]] = []

    wheel_odom: list[tuple[float, float]] = []
    cam_imu_n = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t * 1e-9
        if topic == '/cmd_vel':
            m = deserialize_message(data, Twist)
            cmd.append((ts, m.linear.x, m.angular.z))
        elif topic == '/odom':
            m = deserialize_message(data, Odometry)
            odom.append((ts, _yaw(m.pose.pose.orientation), m.twist.twist.angular.z))
        elif topic == '/wheel/odom':
            m = deserialize_message(data, Odometry)
            wheel_odom.append((ts, m.twist.twist.angular.z))
        elif topic == '/camera/imu/data':
            cam_imu_n += 1
        elif topic == '/tf':
            m = deserialize_message(data, TFMessage)
            for tr in m.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    map_odom.append((
                        ts,
                        tr.transform.translation.x,
                        tr.transform.translation.y,
                        _yaw(tr.transform.rotation),
                    ))

    rclpy.shutdown()

    if not cmd or not odom:
        print('[analyze] bag 缺少 /cmd_vel 或 /odom', file=sys.stderr)
        return 1

    if not wheel_odom:
        print('⚠ bag 无 /wheel/odom → diff_drive 与 EKF 可能仍在争用 /odom；请 kill_sim + rebuild')
    else:
        print(f'/wheel/odom 样本: {len(wheel_odom)}（EKF 轮速输入 OK）')
    if cam_imu_n == 0:
        print('⚠ bag 无 /camera/imu/data（相机 IMU 未发布）')
    else:
        print(f'/camera/imu/data 样本: {cam_imu_n}')

    nonzero_w = sum(1 for c in cmd if abs(c[2]) > 0.01)

    # 检测 /odom 是否在 (0,0) 与积分位姿间交替（双发布者典型症状）
    alt_zero = 0
    for i in range(1, min(len(odom), 500)):
        r0 = math.hypot(odom[i - 1][1], odom[i - 1][2])
        r1 = math.hypot(odom[i][1], odom[i][2])
        if (r0 < 0.05 and r1 > 0.15) or (r1 < 0.05 and r0 > 0.15):
            alt_zero += 1
    if alt_zero > 20:
        print(f'⚠ /odom 原点跳变 {alt_zero} 次/500 样本 → diff_drive 与 EKF 争用 /odom，需 /wheel/odom 隔离')

    print(f'cmd_vel 样本: {len(cmd)}, |ω|>0.01: {nonzero_w} ({100*nonzero_w/len(cmd):.1f}%)')

    straight = [c for c in cmd if abs(c[1]) > 0.08 and abs(c[2]) < 0.005]
    if len(straight) >= 20:
        t0, t1 = straight[0][0], straight[-1][0]
        seg = [o for o in odom if t0 <= o[0] <= t1]
        if len(seg) >= 2:
            dyaw = math.degrees(seg[-1][1] - seg[0][1])
            w_rms = math.sqrt(sum(o[2] * o[2] for o in seg) / len(seg))
            print(f'直行段 (|v|>0.08, ω≈0): {t1-t0:.1f}s')
            print(f'  odom 积分偏航: {dyaw:+.1f}°  (理想应接近 0°)')
            print(f'  odom twist.ω RMS: {w_rms:.3f} rad/s')

    if map_odom:
        yaw_range = math.degrees(max(p[3] for p in map_odom) - min(p[3] for p in map_odom))
        tx_range = max(p[1] for p in map_odom) - min(p[1] for p in map_odom)
        ty_range = max(p[2] for p in map_odom) - min(p[2] for p in map_odom)
        print(f'map→odom 样本: {len(map_odom)}')
        print(f'  平移变化: Δtx={tx_range:.3f}m Δty={ty_range:.3f}m')
        print(f'  偏航变化: {yaw_range:.2f}°  (长期≈0 表示 SLAM 未修正里程计)')

    if len(straight) >= 20 and abs(dyaw) > 15:
        print('\n结论: 滑条未发角速度但 odom 大幅偏航 → 仿真轮地摩擦/里程计问题（非检测）。')
        print('      六轮布局应为前后四驱+中轮从动；请 rebuild 后重跑 analyze 对比。')
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description='分析建图 bag 漂移')
    parser.add_argument('session_dir', type=Path, help='maps/map_<时间戳>/ 或 bag 目录')
    args = parser.parse_args()
    bag = args.session_dir
    if bag.is_dir() and (bag / 'bag').is_dir():
        bag = bag / 'bag'
    if not (bag / 'metadata.yaml').is_file():
        print(f'[analyze] 无 bag: {bag}', file=sys.stderr)
        sys.exit(1)
    sys.exit(analyze(bag))


if __name__ == '__main__':
    main()
