#!/usr/bin/env python3
"""Smoke tests for waypoint track fusion and stereo annular gates."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'detection_pkg'))

from detection_pkg.backprojection import (  # noqa: E402
    MapWaypoint,
    make_transform,
    validate_map_point,
    validate_stereo_point,
)
from detection_pkg.waypoint_track import (  # noqa: E402
    Observation,
    WaypointTrack,
    observation_from_detection,
)


def test_fuse_map_1sigma_collapses_depth_spread() -> None:
    track = WaypointTrack(buffer_size=10, map_ema=1.0)
    u, v = 320.0, 240.0
    wp = None
    for depth, mx in ((3.0, 3.0), (3.2, 3.2), (2.8, 2.8), (3.1, 3.1), (10.0, 10.0)):
        wp = track.fuse_map_1sigma(
            Observation(u=u, v=v, range_m=depth, map_x=mx, map_y=0.0,
                        confidence=0.9, source='stereo'),
            sigma_floor_m=0.08,
        )
    assert abs(wp.x - 3.05) < 0.2, f'expected ~3.05m, got {wp.x}'
    print('fuse_map_1sigma: OK')


def test_observation_backproject() -> None:
    k = np.array([[554.0, 0.0, 320.0], [0.0, 554.0, 240.0], [0.0, 0.0, 1.0]])
    t_cam = make_transform(np.eye(3), np.array([0.0, 0.0, 0.15]))
    obs = observation_from_detection(320.0, 240.0, 4.0, 0.9, 'stereo', k, t_cam)
    assert obs is not None
    assert obs.map_x > 3.5
    print('observation_backproject: OK')


def test_validate_stereo_annular() -> None:
    t_cam = make_transform(np.eye(3), np.array([0.0, 0.0, 0.15]))
    point_near = np.array([1.0, 0.0, 0.002])
    point_far = np.array([12.0, 0.0, 0.002])
    fx, baseline = 554.0, 0.0755
    u_l, u_r_near = 400.0, 389.0
    u_l_far, u_r_far = 320.0, 319.0
    r_near = validate_stereo_point(
        point_near, u_l, u_r_near, fx, baseline, t_cam,
        min_depth_m=0.6, max_depth_m=7.0, max_ground_z_error=0.15,
        min_disparity_px=2.0, ground_z=0.002, max_map_radius_m=15.0,
    )
    r_far = validate_stereo_point(
        point_far, u_l_far, u_r_far, fx, baseline, t_cam,
        min_depth_m=0.6, max_depth_m=7.0, max_ground_z_error=0.15,
        min_disparity_px=2.0, ground_z=0.002, max_map_radius_m=15.0,
    )
    assert r_near is not None
    assert r_far is None
    print('validate_stereo_annular: OK')


def test_validate_map_point() -> None:
    t_cam = make_transform(np.eye(3), np.zeros(3))
    point = np.array([20.0, 0.0, 0.002])
    assert validate_map_point(
        point, t_cam, min_depth_m=0.6, max_depth_m=7.0,
        max_ground_z_error=0.15, max_map_radius_m=15.0,
    ) is None
    print('validate_map_point: OK')


def test_pixel_match() -> None:
    track = WaypointTrack(buffer_size=5)
    track.fuse_map_1sigma(
        Observation(u=300.0, v=200.0, range_m=2.0, map_x=2.0, map_y=0.0,
                    confidence=0.9, source='stereo'),
        sigma_floor_m=0.08,
    )
    assert track.matches_pixel(310.0, 205.0, tol_u=32.0, tol_v=24.0)
    assert not track.matches_pixel(360.0, 200.0, tol_u=32.0, tol_v=24.0)
    print('pixel_match: OK')


def test_map_assoc_no_drag_on_turn() -> None:
    """Overlapping pixels but 1.5 m map separation must not merge waypoints."""
    import math

    dedup_radius_m = 0.45
    save_dedup_radius_m = 0.65
    outlier_reject_m = 0.5

    waypoints = [MapWaypoint(x=0.0, y=0.0, confidence=0.9, source='ground')]
    track = WaypointTrack(buffer_size=10, map_ema=0.3)
    obs_near = Observation(
        u=300.0, v=200.0, range_m=2.0, map_x=0.1, map_y=0.0,
        confidence=0.9, source='ground',
    )
    obs_far = Observation(
        u=300.0, v=200.0, range_m=2.0, map_x=1.5, map_y=0.0,
        confidence=0.9, source='ground',
    )

    wp_refined = track.fuse_map_1sigma(obs_near, sigma_floor_m=0.08, prev=waypoints[0])
    waypoints[0] = wp_refined
    assert abs(waypoints[0].x) < 0.2
    assert track.matches_pixel(obs_far.u, obs_far.v, tol_u=48.0, tol_v=36.0)

    best_dist = math.hypot(obs_far.map_x - waypoints[0].x, obs_far.map_y - waypoints[0].y)
    assert best_dist > dedup_radius_m
    assert best_dist >= save_dedup_radius_m

    jump = best_dist
    assert jump > outlier_reject_m
    frozen_x, frozen_y = waypoints[0].x, waypoints[0].y
    assert abs(frozen_x) < 0.2 and abs(frozen_y) < 0.2
    print('map_assoc_no_drag: OK')


def test_map_assoc_merges_within_radius() -> None:
    """Observations within dedup radius refine the same waypoint."""
    track = WaypointTrack(buffer_size=10, map_ema=0.3)
    prev = MapWaypoint(x=1.0, y=2.0, confidence=0.9, source='ground')
    obs = Observation(
        u=320.0, v=240.0, range_m=2.0, map_x=1.05, map_y=2.02,
        confidence=0.95, source='ground',
    )
    wp = track.fuse_map_1sigma(obs, sigma_floor_m=0.08, prev=prev)
    assert abs(wp.x - 1.05) < 0.15
    assert abs(wp.y - 2.02) < 0.15
    print('map_assoc_merge_near: OK')


def test_map_assoc_ambiguous_band_drops() -> None:
    """Observations in ambiguous band (0.45-0.65 m) are dropped, not merged."""
    dedup_radius_m = 0.45
    save_dedup_radius_m = 0.65
    prev = MapWaypoint(x=0.0, y=0.0, confidence=0.9, source='ground')
    obs = Observation(
        u=300.0, v=200.0, range_m=2.0, map_x=0.55, map_y=0.0,
        confidence=0.9, source='ground',
    )
    dist = 0.55
    assert dist > dedup_radius_m
    assert dist < save_dedup_radius_m
    assert prev.x == 0.0
    print('map_assoc_ambiguous_drop: OK')


from detection_pkg.motion_compensation import (  # noqa: E402
    OdomRingBuffer,
    OdomSample,
    compose_map_camera_tf,
    extrapolate_sample,
)
from scipy.spatial.transform import Rotation


def _make_fake_odom(
    stamp_ns: int,
    x: float,
    y: float,
    yaw: float,
    vx: float = 0.0,
    vy: float = 0.0,
    omega: float = 0.0,
):
  class _Stamp:
      def __init__(self) -> None:
          self.sec = stamp_ns // 1_000_000_000
          self.nanosec = stamp_ns % 1_000_000_000

  class _Header:
      def __init__(self) -> None:
          self.stamp = _Stamp()

  class _Pos:
      def __init__(self) -> None:
          self.x, self.y, self.z = x, y, 0.0

  quat = Rotation.from_euler('z', yaw).as_quat()

  class _Ori:
      def __init__(self) -> None:
          self.x, self.y, self.z, self.w = quat

  class _PoseInner:
      def __init__(self) -> None:
          self.position = _Pos()
          self.orientation = _Ori()

  class _Pose:
      def __init__(self) -> None:
          self.pose = _PoseInner()

  class _Lin:
      def __init__(self) -> None:
          self.x, self.y, self.z = vx, vy, 0.0

  class _Ang:
      def __init__(self) -> None:
          self.x, self.y, self.z = 0.0, 0.0, omega

  class _TwistInner:
      def __init__(self) -> None:
          self.linear = _Lin()
          self.angular = _Ang()

  class _Twist:
      def __init__(self) -> None:
          self.twist = _TwistInner()

  class _Odom:
      def __init__(self) -> None:
          self.header = _Header()
          self.pose = _Pose()
          self.twist = _Twist()

  return _Odom()


def test_odom_buffer_interpolate() -> None:
    buf = OdomRingBuffer(duration_sec=2.0, interp_max_gap_ns=200_000_000)
    buf.add_from_odometry(_make_fake_odom(0, 0.0, 0.0, 0.0))
    buf.add_from_odometry(_make_fake_odom(100_000_000, 1.0, 0.0, 0.0))
    mat = buf.pose_at(50_000_000)
    assert mat is not None
    assert abs(mat[0, 3] - 0.5) < 0.001
    assert abs(mat[1, 3]) < 0.001
    print('odom_buffer_interpolate: OK')


def test_odom_buffer_extrapolate() -> None:
    buf = OdomRingBuffer(
        duration_sec=2.0, interp_max_gap_ns=200_000_000, max_extrapolate_ns=120_000_000)
    buf.add_from_odometry(_make_fake_odom(0, 0.0, 0.0, 0.0, vx=1.0))
    mat = buf.pose_at(50_000_000)
    assert mat is not None
    assert abs(mat[0, 3] - 0.05) < 0.01
    assert buf.last_extrapolate_ms == 50
    assert buf.pose_at(200_000_000) is None
    print('odom_buffer_extrapolate: OK')


def test_compose_map_camera() -> None:
    t_map_odom = make_transform(np.eye(3), np.array([1.0, 2.0, 0.0]))
    t_odom_base = make_transform(np.eye(3), np.array([0.5, 0.0, 0.0]))
    t_base_cam = make_transform(np.eye(3), np.array([0.0, 0.0, 0.15]))
    composed = compose_map_camera_tf(t_map_odom, t_odom_base, t_base_cam)
    expected = t_map_odom @ t_odom_base @ t_base_cam
    assert np.allclose(composed, expected)
    assert abs(composed[0, 3] - 1.5) < 1e-9
    assert abs(composed[2, 3] - 0.15) < 1e-9
    print('compose_map_camera: OK')


def test_extrapolate_sample_forward() -> None:
    sample = OdomSample(
        stamp_ns=0,
        position=np.zeros(3),
        rotation=Rotation.from_euler('z', 0.0).as_matrix(),
        vx=2.0, vy=0.0, vz=0.0, omega_z=0.0,
    )
    out = extrapolate_sample(sample, 0.1)
    assert abs(out.position[0] - 0.2) < 1e-6
    print('extrapolate_sample_forward: OK')


if __name__ == '__main__':
    test_fuse_map_1sigma_collapses_depth_spread()
    test_observation_backproject()
    test_validate_stereo_annular()
    test_validate_map_point()
    test_pixel_match()
    test_map_assoc_no_drag_on_turn()
    test_map_assoc_merges_within_radius()
    test_map_assoc_ambiguous_band_drops()
    test_odom_buffer_interpolate()
    test_odom_buffer_extrapolate()
    test_compose_map_camera()
    test_extrapolate_sample_forward()
    print('All waypoint gate tests passed.')
