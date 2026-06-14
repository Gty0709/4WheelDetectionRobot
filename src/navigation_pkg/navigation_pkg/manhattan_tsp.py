"""Manhattan-distance TSP via Held-Karp DP (fixed start, globally optimal)."""

from __future__ import annotations

import math
from itertools import permutations
from typing import List, Sequence, Tuple

from navigation_pkg.waypoint_io import Waypoint


def manhattan(a: Waypoint, b: Waypoint) -> float:
    return abs(a.x - b.x) + abs(a.y - b.y)


def manhattan_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return abs(ax - bx) + abs(ay - by)


def distance_matrix(waypoints: Sequence[Waypoint]) -> List[List[float]]:
    n = len(waypoints)
    d = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                d[i][j] = manhattan(waypoints[i], waypoints[j])
    return d


def adjacency_matrix(n: int) -> List[List[int]]:
    """Complete graph: edge exists between distinct nodes."""
    return [[0 if i == j else 1 for j in range(n)] for i in range(n)]


def nearest_index(
    waypoints: Sequence[Waypoint],
    x: float,
    y: float,
) -> int:
    best = 0
    best_d = math.inf
    for i, wp in enumerate(waypoints):
        d = manhattan_xy(x, y, wp.x, wp.y)
        if d < best_d:
            best_d = d
            best = i
    return best


def tsp_fixed_start(distance: List[List[float]], start: int) -> List[int]:
    """
    Visit every node exactly once starting at `start`.
    Returns visit order [start, ..., last] (length n).
    """
    n = len(distance)
    if n == 0:
        return []
    if n == 1:
        return [start]

    start_mask = 1 << start
    inf = float('inf')
    # dp[mask][j] = min cost to reach j having visited mask (mask includes start and j)
    dp = [[inf] * n for _ in range(1 << n)]
    parent = [[-1] * n for _ in range(1 << n)]

    for j in range(n):
        if j == start:
            continue
        mask = start_mask | (1 << j)
        dp[mask][j] = distance[start][j]
        parent[mask][j] = start

    for mask in range(1 << n):
        if not (mask & start_mask):
            continue
        for j in range(n):
            if not (mask & (1 << j)):
                continue
            cost_j = dp[mask][j]
            if cost_j == inf:
                continue
            for k in range(n):
                if mask & (1 << k):
                    continue
                nmask = mask | (1 << k)
                nc = cost_j + distance[j][k]
                if nc < dp[nmask][k]:
                    dp[nmask][k] = nc
                    parent[nmask][k] = j

    full = (1 << n) - 1
    end = min(range(n), key=lambda j: dp[full][j])
    order: List[int] = []
    mask = full
    cur = end
    while cur != -1:
        order.append(cur)
        prev = parent[mask][cur]
        mask ^= 1 << cur
        cur = prev
    order.reverse()
    return order


def path_cost(order: Sequence[int], distance: Sequence[Sequence[float]]) -> float:
    """Sum edge costs along an index sequence."""
    if len(order) < 2:
        return 0.0
    total = 0.0
    for i in range(len(order) - 1):
        total += distance[order[i]][order[i + 1]]
    return total


def path_cost_from_pose(
    waypoints: Sequence[Waypoint],
    start_x: float,
    start_y: float,
    wp_order: Sequence[int],
) -> float:
    """Manhattan path length: start pose -> wp_order[0] -> ... -> wp_order[-1]."""
    if not wp_order:
        return 0.0
    total = manhattan_xy(start_x, start_y, waypoints[wp_order[0]].x, waypoints[wp_order[0]].y)
    for i in range(len(wp_order) - 1):
        total += manhattan(waypoints[wp_order[i]], waypoints[wp_order[i + 1]])
    return total


def brute_force_wp_order_from_pose(
    waypoints: Sequence[Waypoint],
    start_x: float,
    start_y: float,
) -> Tuple[List[int], float]:
    """Exhaustive search — for unit-test verification of optimality (small n only)."""
    n = len(waypoints)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [0], manhattan_xy(start_x, start_y, waypoints[0].x, waypoints[0].y)

    best_order: List[int] = []
    best_cost = math.inf
    for perm in permutations(range(n)):
        order = list(perm)
        cost = path_cost_from_pose(waypoints, start_x, start_y, order)
        if cost < best_cost:
            best_cost = cost
            best_order = order
    return best_order, best_cost


def tsp_from_start_pose(
    waypoints: Sequence[Waypoint],
    start_x: float,
    start_y: float,
) -> Tuple[List[int], List[List[float]], List[List[int]], float]:
    """
    Globally optimal open TSP (Held-Karp): visit every waypoint exactly once,
    starting from arbitrary map pose (not necessarily on a waypoint).

    Optimality is exact for the Manhattan distance metric on a complete graph.
    """
    n_wp = len(waypoints)
    wp_d = distance_matrix(waypoints)
    wp_a = adjacency_matrix(n_wp)
    if n_wp == 0:
        return [], wp_d, wp_a, 0.0
    if n_wp == 1:
        return [0], wp_d, wp_a, manhattan_xy(start_x, start_y, waypoints[0].x, waypoints[0].y)

    # Extended graph: depot index 0 at relocalization pose, waypoint i -> index i+1
    n = n_wp + 1
    ext = [[0.0] * n for _ in range(n)]
    for j in range(n_wp):
        c = manhattan_xy(start_x, start_y, waypoints[j].x, waypoints[j].y)
        ext[0][j + 1] = c
        ext[j + 1][0] = c
    for i in range(n_wp):
        for j in range(n_wp):
            if i != j:
                ext[i + 1][j + 1] = wp_d[i][j]

    full_order = tsp_fixed_start(ext, 0)
    wp_order = [idx - 1 for idx in full_order if idx != 0]
    total_cost = path_cost(full_order, ext)
    return wp_order, wp_d, wp_a, total_cost


def tsp_order_after_first(
    waypoints: Sequence[Waypoint],
    start_idx: int,
) -> Tuple[List[int], List[List[float]], List[List[int]]]:
    """Build matrices and return full TSP order from fixed start."""
    d = distance_matrix(waypoints)
    a = adjacency_matrix(len(waypoints))
    order = tsp_fixed_start(d, start_idx)
    return order, d, a
