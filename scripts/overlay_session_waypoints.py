#!/usr/bin/env python3
"""Generate slam_map_waypoints.png for an existing session directory."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _find_ws_src() -> Path:
    return Path(__file__).resolve().parents[1] / 'src' / 'detection_pkg'


def main() -> int:
    parser = argparse.ArgumentParser(description='Overlay waypoints onto slam_map.png')
    parser.add_argument('session_dir', type=Path, help='maps/map_<timestamp>/')
    args = parser.parse_args()

    sys.path.insert(0, str(_find_ws_src()))
    from detection_pkg.map_overlay import render_waypoints_overlay

    session = args.session_dir.expanduser().resolve()
    if not session.is_dir():
        print(f'[overlay] not a directory: {session}', file=sys.stderr)
        return 1
    out = render_waypoints_overlay(session)
    if out is None:
        print(f'[overlay] no waypoints.yaml in {session}', file=sys.stderr)
        return 1
    print(f'[overlay] wrote {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
