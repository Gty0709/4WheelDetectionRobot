#!/usr/bin/env python3
"""Analyze a saved patrol mission directory for leg timing and motion quality."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

FOCUS_LEGS = [
    (3, 17),
    (10, 14),
    (14, 11),
    (11, 4),
    (4, 8),
]


def _load_yaml(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as fh:
        return yaml.safe_load(fh) or {}


def _yaw_delta(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(b - a), math.cos(b - a)))


def _leg_times(visits: List[dict]) -> List[Tuple[int, int, float]]:
    legs: List[Tuple[int, int, float]] = []
    prev_id: Optional[int] = None
    prev_t: Optional[float] = None
    for row in visits:
        wp_id = int(row['waypoint_id'])
        t = float(row['visited_at_sec'])
        if prev_id is not None and prev_t is not None:
            legs.append((prev_id, wp_id, t - prev_t))
        prev_id = wp_id
        prev_t = t
    return legs


def _analyze_trajectory(samples: List[dict]) -> dict:
    if len(samples) < 2:
        return {
            'spin_sec': 0.0,
            'low_motion_sec': 0.0,
            'reverse_hint_sec': 0.0,
        }

    spin_sec = 0.0
    low_motion_sec = 0.0
    reverse_hint_sec = 0.0

    for i in range(1, len(samples)):
        a = samples[i - 1]
        b = samples[i]
        dt = float(b['t_sec']) - float(a['t_sec'])
        if dt <= 0.0:
            continue
        dx = float(b['x']) - float(a['x'])
        dy = float(b['y']) - float(a['y'])
        dist = math.hypot(dx, dy)
        dyaw = _yaw_delta(float(a['yaw']), float(b['yaw']))
        speed = dist / dt
        yaw_rate = dyaw / dt

        if dist < 0.03 and dyaw > 0.15:
            spin_sec += dt
        if dist < 0.05 and speed < 0.04:
            low_motion_sec += dt

        heading = float(a['yaw'])
        motion_yaw = math.atan2(dy, dx) if dist > 1e-3 else heading
        if dist > 0.04 and _yaw_delta(heading, motion_yaw) > 2.0:
            reverse_hint_sec += dt

    return {
        'spin_sec': spin_sec,
        'low_motion_sec': low_motion_sec,
        'reverse_hint_sec': reverse_hint_sec,
    }


def _slice_between(samples: List[dict], t0: float, t1: float) -> List[dict]:
    return [s for s in samples if t0 <= float(s['t_sec']) <= t1]


def analyze_run(mission_dir: Path) -> None:
    summary = _load_yaml(mission_dir / 'mission_summary.yaml')
    traj_doc = _load_yaml(mission_dir / 'trajectory.yaml')
    samples = traj_doc.get('samples') or []
    visits = summary.get('visits') or []

    print(f'Mission: {mission_dir.name}')
    print(f"Duration: {summary.get('duration_sec', '?')} s")
    print(f"Visits: {len(visits)}  aborted: {summary.get('aborted_waypoint_ids', [])}")
    print()

    visit_time: Dict[int, float] = {
        int(v['waypoint_id']): float(v['visited_at_sec']) for v in visits
    }
    legs = _leg_times(visits)

    print('Leg timings (visit order):')
    for frm, to, sec in legs:
        mark = '  <-- focus' if (frm, to) in FOCUS_LEGS else ''
        print(f'  {frm:>2} -> {to:<2}  {sec:7.1f} s{mark}')
    print()

    motion = _analyze_trajectory(samples)
    print('Trajectory motion (whole mission):')
    print(f"  in-place spin time:   {motion['spin_sec']:.1f} s")
    print(f"  low-motion time:      {motion['low_motion_sec']:.1f} s")
    print(f"  reverse-like motion:  {motion['reverse_hint_sec']:.1f} s")
    print()

    print('Focus leg motion slices:')
    for frm, to in FOCUS_LEGS:
        if frm not in visit_time or to not in visit_time:
            print(f'  {frm}->{to}: missing visit timestamps')
            continue
        t0 = visit_time[frm]
        t1 = visit_time[to]
        seg = _slice_between(samples, t0, t1)
        seg_motion = _analyze_trajectory(seg)
        print(
            f"  {frm}->{to}: {t1 - t0:.1f} s, "
            f"spin {seg_motion['spin_sec']:.1f}s, "
            f"low-motion {seg_motion['low_motion_sec']:.1f}s"
        )


def main() -> None:
    p = argparse.ArgumentParser(description='Analyze navigation mission result directory.')
    p.add_argument(
        'mission_dir',
        nargs='?',
        default='src/navigation_pkg/result/path_20260615_131011',
        help='Path to path_<timestamp> directory',
    )
    args = p.parse_args()
    mission_dir = Path(args.mission_dir)
    if not mission_dir.is_absolute():
        ws = Path(__file__).resolve().parents[3]
        mission_dir = ws / mission_dir
    if not mission_dir.is_dir():
        raise SystemExit(f'Not a directory: {mission_dir}')
    analyze_run(mission_dir)


if __name__ == '__main__':
    main()
