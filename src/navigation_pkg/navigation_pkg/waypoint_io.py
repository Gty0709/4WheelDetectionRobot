"""Load waypoints and session paths from map_latest session directory."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import yaml


@dataclass
class Waypoint:
    id: int
    x: float
    y: float
    confidence: float = 1.0
    source: str = 'ground'


@dataclass
class InitialPose:
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'


def resolve_ws_root() -> Path:
    """Resolve workspace root from installed or source layout."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'src' / 'perception_pkg').is_dir():
            return parent
    return Path.cwd()


def resolve_session_dir(session_dir: Optional[str] = None) -> Path:
    """Return absolute session directory; default map_latest under src."""
    if session_dir:
        p = Path(session_dir).expanduser()
        if not p.is_absolute():
            p = resolve_ws_root() / p
        return p.resolve()
    default = resolve_ws_root() / 'src' / 'perception_pkg' / 'maps' / 'map_latest'
    return default.resolve()


def resolve_result_dir(result_dir: Optional[str] = None) -> Path:
    """Return navigation_pkg/result directory."""
    if result_dir:
        p = Path(result_dir).expanduser()
        if not p.is_absolute():
            p = resolve_ws_root() / p
        return p.resolve()
    return (resolve_ws_root() / 'src' / 'navigation_pkg' / 'result').resolve()


def resolve_path_latest(result_dir: Optional[str] = None) -> Path:
    """Resolve result/path_latest symlink to the mission directory."""
    root = resolve_result_dir(result_dir)
    latest = root / 'path_latest'
    if latest.is_symlink():
        target = Path(os.readlink(latest))
        if not target.is_absolute():
            target = latest.parent / target
        return target.resolve()
    return latest.resolve()


def load_waypoints(path: Path) -> List[Waypoint]:
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    out: List[Waypoint] = []
    for item in data.get('waypoints', []):
        out.append(
            Waypoint(
                id=int(item['id']),
                x=float(item['x']),
                y=float(item['y']),
                confidence=float(item.get('confidence', 1.0)),
                source=str(item.get('source', 'ground')),
            )
        )
    return out


def load_initial_pose(path: Path) -> InitialPose:
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return InitialPose(
        x=float(data['x']),
        y=float(data['y']),
        yaw=float(data['yaw']),
        frame_id=str(data.get('frame_id', 'map')),
    )


def session_files(session_dir: Path) -> dict:
    """Standard file paths inside a map session."""
    return {
        'map_yaml': session_dir / 'slam_map.yaml',
        'waypoints_yaml': session_dir / 'waypoints.yaml',
        'initial_pose_yaml': session_dir / 'initial_pose.yaml',
    }
