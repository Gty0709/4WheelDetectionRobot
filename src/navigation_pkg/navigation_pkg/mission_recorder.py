"""Record patrol mission trajectory and leg paths to result/path_<ts>/."""

from __future__ import annotations

import json
import os
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from navigation_pkg.mission_plot import MISSION_TRAJECTORY_PNG, render_mission_trajectory_png
from navigation_pkg.trajectory_tracker import filter_trajectory_outliers
from navigation_pkg.waypoint_io import Waypoint, resolve_result_dir


@dataclass
class TrajectorySample:
    t_sec: float
    x: float
    y: float
    yaw: float


@dataclass
class LegRecord:
    leg_index: int
    from_id: int
    to_id: int
    poses: List[Dict[str, float]] = field(default_factory=list)


@dataclass
class VisitRecord:
    waypoint_id: int
    visited_at_sec: float
    dwell_sec: float


@dataclass
class DynamicObstacle:
    x: float
    y: float
    first_seen_sec: float
    last_seen_sec: float
    hits: int = 1


class MissionRecorder:
    def __init__(
        self,
        session_dir: Path,
        waypoints: List[Waypoint],
        result_dir: Optional[Path] = None,
    ):
        self._session_dir = session_dir
        self._waypoints = waypoints
        self._result_root = result_dir or resolve_result_dir()
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._mission_dir = self._result_root / f'path_{ts}'
        self._legs_dir = self._mission_dir / 'legs'
        self._start_time: Optional[float] = None
        self._trajectory: List[TrajectorySample] = []
        self._legs: List[LegRecord] = []
        self._visits: List[VisitRecord] = []
        self._tsp_order: List[int] = []
        self._start_pose: Optional[Tuple[float, float]] = None
        self._optimal_manhattan_cost: Optional[float] = None
        self._distance_matrix: List[List[float]] = []
        self._adjacency_matrix: List[List[int]] = []
        self._finalized = False
        self._dynamic_obstacles: Dict[Tuple[int, int], DynamicObstacle] = {}
        self._obstacle_grid_m = 0.18

    @property
    def mission_dir(self) -> Path:
        return self._mission_dir

    def start(self, t_sec: float) -> None:
        self._start_time = t_sec
        self._mission_dir.mkdir(parents=True, exist_ok=True)
        self._legs_dir.mkdir(parents=True, exist_ok=True)
        self._update_latest_symlink()
        self.save_progress(t_sec, 'started', {})

    def save_progress(self, t_sec: float, phase: str, extra: Optional[Dict[str, Any]] = None) -> None:
        self._mission_dir.mkdir(parents=True, exist_ok=True)
        payload: Dict[str, Any] = {
            'phase': phase,
            'elapsed_sec': self.elapsed(t_sec),
            'trajectory_samples': len(self._trajectory),
            'visits': len(self._visits),
            'map_session_dir': str(self._session_dir),
            'mission_dir': str(self._mission_dir),
            'waypoint_origin': 'clip_detection',
            'waypoint_origin_note': (
                'waypoints.yaml from perception (CLIP paperclip detection); NOT ground-truth positions'
            ),
            'dynamic_obstacles': len(self._dynamic_obstacles),
        }
        if extra:
            payload.update(extra)
        with open(self._mission_dir / 'progress.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
        traj = [
            {'t_sec': s.t_sec, 'x': s.x, 'y': s.y, 'yaw': s.yaw}
            for s in self._trajectory
        ]
        traj = filter_trajectory_outliers(traj)
        with open(self._mission_dir / 'trajectory.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(
                {
                    'frame_id': 'map',
                    'integration': 'odom_dead_reckon',
                    'samples': traj,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )
        wp_snapshot = [
            {
                'id': w.id,
                'x': w.x,
                'y': w.y,
                'confidence': w.confidence,
                'source': w.source,
            }
            for w in self._waypoints
        ]
        with open(self._mission_dir / 'waypoints_snapshot.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(
                {
                    'waypoint_origin': 'clip_detection',
                    'waypoint_origin_note': 'NOT ground-truth paperclip positions',
                    'waypoints': wp_snapshot,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )
        if self._trajectory or self._dynamic_obstacles:
            visited_ids = [v.waypoint_id for v in self._visits]
            self._write_detected_obstacles_yaml()
            self._render_trajectory_png(traj, wp_snapshot, visited_ids)
        order_ids = [self._waypoints[i].id for i in self._tsp_order] if self._tsp_order else []
        mission = {
            'frame_id': 'map',
            'trajectory_integration': 'odom_dead_reckon',
            'trajectory': traj,
            'tsp_order_ids': order_ids,
            'waypoint_origin': 'clip_detection',
            'waypoints': wp_snapshot,
            'visits': [
                {
                    'waypoint_id': v.waypoint_id,
                    'visited_at_sec': v.visited_at_sec,
                    'dwell_sec': v.dwell_sec,
                }
                for v in self._visits
            ],
            'dynamic_obstacles': self.dynamic_obstacle_snapshot(),
        }
        with open(self._mission_dir / 'mission.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(mission, f, allow_unicode=True, sort_keys=False)

    def elapsed(self, t_sec: float) -> float:
        if self._start_time is None:
            return 0.0
        return t_sec - self._start_time

    def add_pose_sample(self, t_sec: float, x: float, y: float, yaw: float) -> None:
        self._trajectory.append(
            TrajectorySample(t_sec=self.elapsed(t_sec), x=x, y=y, yaw=yaw)
        )

    def add_dynamic_obstacles(self, t_sec: float, points: List[Tuple[float, float]]) -> None:
        if not points:
            return
        elapsed = self.elapsed(t_sec)
        g = self._obstacle_grid_m
        for x, y in points:
            key = (int(round(x / g)), int(round(y / g)))
            if key in self._dynamic_obstacles:
                obs = self._dynamic_obstacles[key]
                n = obs.hits + 1
                obs.x = (obs.x * obs.hits + x) / n
                obs.y = (obs.y * obs.hits + y) / n
                obs.hits = n
                obs.last_seen_sec = elapsed
            else:
                self._dynamic_obstacles[key] = DynamicObstacle(
                    x=x, y=y, first_seen_sec=elapsed, last_seen_sec=elapsed,
                )

    def dynamic_obstacle_snapshot(self) -> List[Dict[str, float]]:
        return [
            {
                'x': o.x,
                'y': o.y,
                'first_seen_sec': o.first_seen_sec,
                'last_seen_sec': o.last_seen_sec,
                'hits': o.hits,
            }
            for o in self._dynamic_obstacles.values()
        ]

    def _write_detected_obstacles_yaml(self) -> None:
        obstacles = self.dynamic_obstacle_snapshot()
        with open(self._mission_dir / 'detected_obstacles.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(
                {
                    'frame_id': 'map',
                    'source': 'local_costmap_vs_static_slam',
                    'note': 'Orange markers on mission_trajectory.png (local costmap vs static SLAM)',
                    'obstacles': obstacles,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )

    def _render_trajectory_png(
        self,
        traj: List[Dict[str, float]],
        wp_snapshot: List[Dict[str, float]],
        visited_ids: List[int],
    ) -> None:
        try:
            render_mission_trajectory_png(
                self._session_dir,
                self._mission_dir / MISSION_TRAJECTORY_PNG,
                traj,
                wp_snapshot,
                visited_ids,
            )
        except Exception:
            pass

    def set_tsp(
        self,
        order_indices: List[int],
        distance_matrix: List[List[float]],
        adjacency_matrix: List[List[int]],
        start_pose: Optional[Tuple[float, float]] = None,
        optimal_manhattan_cost: Optional[float] = None,
    ) -> None:
        self._tsp_order = order_indices
        self._start_pose = start_pose
        self._optimal_manhattan_cost = optimal_manhattan_cost
        self._distance_matrix = distance_matrix
        self._adjacency_matrix = adjacency_matrix

    def add_leg(
        self,
        leg_index: int,
        from_id: int,
        to_id: int,
        poses: List[Dict[str, float]],
    ) -> None:
        leg = LegRecord(leg_index=leg_index, from_id=from_id, to_id=to_id, poses=poses)
        self._legs.append(leg)
        fname = f'leg_{leg_index:02d}_{from_id}_to_{to_id}.yaml'
        with open(self._legs_dir / fname, 'w', encoding='utf-8') as f:
            yaml.safe_dump(
                {
                    'from_id': from_id,
                    'to_id': to_id,
                    'poses': poses,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )

    def record_visit(self, waypoint_id: int, t_sec: float, dwell_sec: float) -> None:
        self._visits.append(
            VisitRecord(
                waypoint_id=waypoint_id,
                visited_at_sec=self.elapsed(t_sec),
                dwell_sec=dwell_sec,
            )
        )

    def finalize(self, t_sec: float, aborted: Optional[List[int]] = None) -> Path:
        if self._finalized:
            return self._mission_dir
        self._finalized = True
        self._mission_dir.mkdir(parents=True, exist_ok=True)

        wp_snapshot = [
            {'id': w.id, 'x': w.x, 'y': w.y, 'confidence': w.confidence, 'source': w.source}
            for w in self._waypoints
        ]
        with open(self._mission_dir / 'waypoints_snapshot.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(
                {
                    'waypoint_origin': 'clip_detection',
                    'waypoint_origin_note': 'NOT ground-truth paperclip positions',
                    'waypoints': wp_snapshot,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )

        order_ids = [self._waypoints[i].id for i in self._tsp_order]
        tsp_payload: Dict[str, Any] = {
            'solver': 'held_karp',
            'optimality': 'globally_optimal_manhattan_fixed_start',
            'start_pose': (
                {'x': self._start_pose[0], 'y': self._start_pose[1]}
                if self._start_pose is not None else None
            ),
            'optimal_manhattan_cost_m': self._optimal_manhattan_cost,
            'order_indices': self._tsp_order,
            'order_ids': order_ids,
            'distance_matrix': self._distance_matrix,
            'adjacency_matrix': self._adjacency_matrix,
        }
        with open(self._mission_dir / 'tsp_order.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(tsp_payload, f, allow_unicode=True, sort_keys=False)

        traj = [
            {'t_sec': s.t_sec, 'x': s.x, 'y': s.y, 'yaw': s.yaw}
            for s in self._trajectory
        ]
        traj = filter_trajectory_outliers(traj)
        with open(self._mission_dir / 'trajectory.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(
                {
                    'frame_id': 'map',
                    'integration': 'odom_dead_reckon',
                    'samples': traj,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )

        summary: Dict[str, Any] = {
            'duration_sec': self.elapsed(t_sec),
            'visits': [
                {
                    'waypoint_id': v.waypoint_id,
                    'visited_at_sec': v.visited_at_sec,
                    'dwell_sec': v.dwell_sec,
                }
                for v in self._visits
            ],
            'aborted_waypoint_ids': aborted or [],
            'leg_count': len(self._legs),
        }
        with open(self._mission_dir / 'mission_summary.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(summary, f, allow_unicode=True, sort_keys=False)

        meta = {
            'session_id': self._mission_dir.name,
            'map_session_dir': str(self._session_dir),
            'created_at': datetime.now().isoformat(),
            'duration_sec': self.elapsed(t_sec),
            'waypoint_count': len(self._waypoints),
            'visit_count': len(self._visits),
            'waypoint_origin': 'clip_detection',
            'dynamic_obstacle_count': len(self._dynamic_obstacles),
        }
        with open(self._mission_dir / 'session_meta.json', 'w', encoding='utf-8') as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        # Combined mission file for playback
        mission = {
            'frame_id': 'map',
            'trajectory_integration': 'odom_dead_reckon',
            'trajectory': traj,
            'tsp_order_ids': order_ids,
            'waypoint_origin': 'clip_detection',
            'waypoints': wp_snapshot,
            'visits': summary['visits'],
            'dynamic_obstacles': self.dynamic_obstacle_snapshot(),
        }
        with open(self._mission_dir / 'mission.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(mission, f, allow_unicode=True, sort_keys=False)

        self._write_detected_obstacles_yaml()
        visited_ids = [v.waypoint_id for v in self._visits]
        try:
            self._render_trajectory_png(traj, wp_snapshot, visited_ids)
        except Exception as exc:
            print(f'[mission_recorder] trajectory PNG skipped: {exc}', flush=True)

        self._update_latest_symlink()
        return self._mission_dir

    def _update_latest_symlink(self) -> None:
        latest = self._result_root / 'path_latest'
        target = self._mission_dir.name
        if latest.is_symlink() or latest.exists():
            if latest.is_symlink():
                latest.unlink()
            elif latest.is_dir():
                shutil.rmtree(latest)
            else:
                latest.unlink()
        os.symlink(target, latest)
