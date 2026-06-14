#!/usr/bin/env python3
"""Unit tests for Manhattan TSP helpers."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV_SRC = ROOT / 'src' / 'navigation_pkg'
sys.path.insert(0, str(NAV_SRC))

from navigation_pkg.manhattan_tsp import (  # noqa: E402
    adjacency_matrix,
    brute_force_wp_order_from_pose,
    distance_matrix,
    manhattan_xy,
    path_cost_from_pose,
    tsp_fixed_start,
    tsp_from_start_pose,
)
from navigation_pkg.waypoint_io import Waypoint  # noqa: E402


def test_distance_matrix_symmetry():
    wps = [
        Waypoint(id=1, x=0.0, y=0.0),
        Waypoint(id=2, x=1.0, y=2.0),
        Waypoint(id=3, x=3.0, y=1.0),
    ]
    d = distance_matrix(wps)
    assert len(d) == 3
    for i in range(3):
        assert d[i][i] == 0.0
        for j in range(3):
            assert d[i][j] == d[j][i]
    assert d[0][1] == manhattan_xy(0, 0, 1, 2)


def test_adjacency_complete():
    a = adjacency_matrix(4)
    for i in range(4):
        assert a[i][i] == 0
        for j in range(4):
            if i != j:
                assert a[i][j] == 1


def test_tsp_three_points_known():
    wps = [
        Waypoint(id=1, x=0.0, y=0.0),
        Waypoint(id=2, x=2.0, y=0.0),
        Waypoint(id=3, x=2.0, y=2.0),
    ]
    d = distance_matrix(wps)
    order = tsp_fixed_start(d, start=0)
    assert order[0] == 0
    assert set(order) == {0, 1, 2}
    assert order == [0, 1, 2]


def test_tsp_single_node():
    d = [[0.0]]
    assert tsp_fixed_start(d, 0) == [0]


def test_tsp_from_start_pose_matches_brute_force():
    wps = [
        Waypoint(id=1, x=0.0, y=0.0),
        Waypoint(id=2, x=2.0, y=0.0),
        Waypoint(id=3, x=2.0, y=2.0),
        Waypoint(id=4, x=0.0, y=3.0),
    ]
    sx, sy = 1.0, -1.0
    order, _, _, cost = tsp_from_start_pose(wps, sx, sy)
    bf_order, bf_cost = brute_force_wp_order_from_pose(wps, sx, sy)
    assert cost == bf_cost
    assert path_cost_from_pose(wps, sx, sy, order) == bf_cost
    assert set(order) == set(bf_order)


def test_tsp_from_start_pose_single():
    wps = [Waypoint(id=1, x=3.0, y=4.0)]
    order, _, _, cost = tsp_from_start_pose(wps, 0.0, 0.0)
    assert order == [0]
    assert cost == manhattan_xy(0.0, 0.0, 3.0, 4.0)


def main() -> int:
    test_distance_matrix_symmetry()
    test_adjacency_complete()
    test_tsp_three_points_known()
    test_tsp_single_node()
    test_tsp_from_start_pose_matches_brute_force()
    test_tsp_from_start_pose_single()
    print('All Manhattan TSP tests passed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
