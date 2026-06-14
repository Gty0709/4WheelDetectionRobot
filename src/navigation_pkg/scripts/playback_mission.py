#!/usr/bin/env python3
"""Playback saved patrol mission trajectory at configurable rate in RViz."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from navigation_pkg.waypoint_io import resolve_path_latest


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MissionPlayback(Node):
    def __init__(self, mission_dir: Path, rate: float, loop: bool):
        super().__init__('mission_playback')
        self._rate = max(rate, 0.1)
        self._loop = loop

        mission_file = mission_dir / 'mission.yaml'
        if not mission_file.is_file():
            raise FileNotFoundError(f'Missing {mission_file}')
        with open(mission_file, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        self._frame_id = str(data.get('frame_id', 'map'))
        self._samples = data.get('trajectory', [])
        self._waypoints = data.get('waypoints', [])
        visited_ids = {v['waypoint_id'] for v in data.get('visits', [])}

        self._pose_pub = self.create_publisher(PoseStamped, '/navigation/playback_pose', 10)
        self._driven_pub = self.create_publisher(Path, '/navigation/driven_path', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/navigation/waypoint_markers', 10)

        self._full_path = Path()
        self._full_path.header.frame_id = self._frame_id
        self._played_path = Path()
        self._played_path.header.frame_id = self._frame_id
        for s in self._samples:
            ps = PoseStamped()
            ps.header.frame_id = self._frame_id
            ps.pose.position.x = float(s['x'])
            ps.pose.position.y = float(s['y'])
            ps.pose.orientation = yaw_to_quat(float(s.get('yaw', 0.0)))
            self._full_path.poses.append(ps)
        self._driven_pub.publish(self._full_path)
        self._publish_markers(visited_ids)

        self._idx = 0
        self._playback_start = self.get_clock().now()
        self._mission_start_t = (
            float(self._samples[0]['t_sec']) if self._samples else 0.0
        )
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'Playing {len(self._samples)} samples from {mission_dir} at {self._rate}x'
        )

    def _publish_markers(self, visited_ids: set) -> None:
        arr = MarkerArray()
        for wp in self._waypoints:
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'detected_paperclips'
            m.id = int(wp['id'])
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wp['x'])
            m.pose.position.y = float(wp['y'])
            m.pose.position.z = 0.05
            m.scale.x = m.scale.y = m.scale.z = 0.15
            if int(wp['id']) in visited_ids:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.95, 0.1, 0.95
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.1, 0.1, 0.95
            arr.markers.append(m)
        self._marker_pub.publish(arr)

    def _tick(self) -> None:
        if not self._samples:
            return
        elapsed = (self.get_clock().now() - self._playback_start).nanoseconds * 1e-9
        mission_t = self._mission_start_t + elapsed * self._rate

        while (
            self._idx + 1 < len(self._samples)
            and float(self._samples[self._idx + 1]['t_sec']) <= mission_t
        ):
            self._idx += 1

        if self._idx >= len(self._samples):
            if self._loop:
                self._idx = 0
                self._playback_start = self.get_clock().now()
                self._mission_start_t = float(self._samples[0]['t_sec'])
                return
            self._timer.cancel()
            return

        s = self._samples[self._idx]
        msg = PoseStamped()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(s['x'])
        msg.pose.position.y = float(s['y'])
        msg.pose.orientation = yaw_to_quat(float(s.get('yaw', 0.0)))
        self._pose_pub.publish(msg)
        ps = PoseStamped()
        ps.header.frame_id = self._frame_id
        ps.header.stamp = msg.header.stamp
        ps.pose = msg.pose
        self._played_path.poses.append(ps)
        self._played_path.header.stamp = ps.header.stamp
        self._driven_pub.publish(self._played_path)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Playback patrol mission in RViz.')
    p.add_argument(
        '--mission',
        default='',
        help='Mission directory (default: result/path_latest)',
    )
    p.add_argument('--rate', type=float, default=2.0, help='Playback speed multiplier.')
    p.add_argument('--loop', action='store_true', help='Loop playback.')
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if args.mission:
        mission_dir = Path(args.mission).expanduser().resolve()
    else:
        mission_dir = resolve_path_latest()

    rclpy.init()
    node = MissionPlayback(mission_dir, args.rate, args.loop)
    node.get_logger().info(f'Playback mission dir: {mission_dir}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
