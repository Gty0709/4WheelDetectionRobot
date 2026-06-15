#!/usr/bin/env python3
"""Playback saved patrol mission trajectory at configurable rate in RViz."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path as PathLib
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from navigation_pkg.waypoint_io import resolve_path_latest

STATIC_VIZ_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MissionPlayback(Node):
    def __init__(self, mission_dir: PathLib, rate: float, loop: bool):
        super().__init__('mission_playback')
        self._rate = max(rate, 0.1)
        self._loop = loop
        self._finished = False

        mission_file = mission_dir / 'mission.yaml'
        if not mission_file.is_file():
            raise FileNotFoundError(f'Missing {mission_file}')
        with open(mission_file, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        self._frame_id = str(data.get('frame_id', 'map'))
        self._samples = data.get('trajectory', [])
        self._waypoints = data.get('waypoints', [])
        self._visited_ids = {v['waypoint_id'] for v in data.get('visits', [])}

        self._pose_pub = self.create_publisher(PoseStamped, '/navigation/playback_pose', 10)
        self._mission_path_pub = self.create_publisher(
            Path, '/navigation/mission_path', STATIC_VIZ_QOS,
        )
        self._driven_pub = self.create_publisher(Path, '/navigation/driven_path', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/navigation/waypoint_markers', STATIC_VIZ_QOS,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

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
        self._mission_path_pub.publish(self._full_path)
        self._publish_markers()

        self._idx = 0
        self._wall_start = time.monotonic()
        self._mission_start_t = (
            float(self._samples[0]['t_sec']) if self._samples else 0.0
        )
        self._timer = self.create_timer(0.05, self._tick)
        self._static_viz_timer = self.create_timer(2.0, self._republish_static_viz)
        if self._samples:
            s0 = self._samples[0]
            stamp = self.get_clock().now().to_msg()
            self._publish_tf(
                float(s0['x']), float(s0['y']), float(s0.get('yaw', 0.0)), stamp,
            )
        self.get_logger().info(
            f'Playing {len(self._samples)} samples from {mission_dir} at {self._rate}x'
        )
        if not self._samples:
            self.get_logger().warn('Mission has no trajectory samples')
            self.create_timer(0.5, self._finish_playback)
        else:
            duration = float(self._samples[-1]['t_sec']) - float(self._samples[0]['t_sec'])
            self.get_logger().info(
                f'Mission duration {duration:.1f}s -> playback ~{duration / self._rate:.1f}s'
            )

    def _republish_static_viz(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._full_path.header.stamp = stamp
        self._mission_path_pub.publish(self._full_path)
        self._publish_markers()

    def _publish_markers(self) -> None:
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
            if int(wp['id']) in self._visited_ids:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.95, 0.1, 0.95
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.1, 0.1, 0.95
            arr.markers.append(m)
        self._marker_pub.publish(arr)

    def _publish_tf(self, x: float, y: float, yaw: float, stamp) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._frame_id
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quat(yaw)
        self._tf_broadcaster.sendTransform(t)

    def _reset_playback(self) -> None:
        self._idx = 0
        self._wall_start = time.monotonic()
        self._mission_start_t = float(self._samples[0]['t_sec'])
        self._played_path = Path()
        self._played_path.header.frame_id = self._frame_id

    def _finish_playback(self) -> None:
        if self._finished:
            return
        self._finished = True
        if hasattr(self, '_static_viz_timer'):
            self._static_viz_timer.cancel()
        self.get_logger().info('Playback finished; closing session')
        self.create_timer(0.1, self._shutdown_node_once)

    def _shutdown_node_once(self) -> None:
        if hasattr(self, '_timer'):
            self._timer.cancel()
        try:
            self.destroy_node()
        except Exception:
            pass
        sys.exit(0)

    def _tick(self) -> None:
        if self._finished:
            return
        if not self._samples:
            return
        elapsed = time.monotonic() - self._wall_start
        mission_t = self._mission_start_t + elapsed * self._rate

        while (
            self._idx + 1 < len(self._samples)
            and float(self._samples[self._idx + 1]['t_sec']) <= mission_t
        ):
            self._idx += 1

        at_end = (
            self._idx >= len(self._samples) - 1
            and mission_t >= float(self._samples[-1]['t_sec'])
        )
        if at_end:
            if self._loop:
                self._reset_playback()
                return
            self._finish_playback()
            return

        s = self._samples[self._idx]
        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = float(s['x'])
        msg.pose.position.y = float(s['y'])
        msg.pose.orientation = yaw_to_quat(float(s.get('yaw', 0.0)))
        self._pose_pub.publish(msg)
        self._publish_tf(float(s['x']), float(s['y']), float(s.get('yaw', 0.0)), stamp)

        ps = PoseStamped()
        ps.header.frame_id = self._frame_id
        ps.header.stamp = stamp
        ps.pose = msg.pose
        self._played_path.poses.append(ps)
        self._played_path.header.stamp = stamp
        self._driven_pub.publish(self._played_path)


def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Playback patrol mission in RViz.')
    p.add_argument(
        '--mission',
        default='',
        help='Mission directory (default: result/path_latest)',
    )
    p.add_argument('--rate', type=float, default=2.0, help='Playback speed multiplier.')
    p.add_argument('--loop', action='store_true', help='Loop playback.')
    return p.parse_args(argv)


def main() -> None:
    from rclpy.utilities import remove_ros_args

    argv = remove_ros_args(sys.argv)
    args = parse_args(argv[1:])
    if args.mission:
        mission_dir = PathLib(args.mission).expanduser().resolve()
    else:
        mission_dir = resolve_path_latest()

    rclpy.init(args=sys.argv)
    node = MissionPlayback(mission_dir, args.rate, args.loop)
    node.get_logger().info(f'Playback mission dir: {mission_dir}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
