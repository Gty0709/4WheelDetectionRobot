#!/usr/bin/env python3
"""Waypoint patrol orchestrator: relocalize, optimal TSP tour, dwell markers."""

from __future__ import annotations

import argparse
import math
import threading
from enum import Enum, auto
from typing import List, Optional, Set

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Point
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from navigation_pkg.collision_check import (
    backup_target_yaw,
    estimate_free_heading,
    footprint_free,
    footprint_max_cost,
)
from navigation_pkg.manhattan_tsp import tsp_from_start_pose
from navigation_pkg.mission_recorder import MissionRecorder
from navigation_pkg.obstacle_detect import StaticMapIndex, extract_dynamic_obstacles
from navigation_pkg.trajectory_tracker import OdomTrajectoryTracker
from navigation_pkg.waypoint_io import (
    InitialPose,
    Waypoint,
    load_initial_pose,
    load_waypoints,
    resolve_session_dir,
    session_files,
)


class Phase(Enum):
    WAIT_NAV2 = auto()
    WAIT_RELOCALIZE = auto()
    DWELL = auto()
    COMPUTE_TSP = auto()
    GO_NEXT = auto()
    BACKUP = auto()
    SPIN = auto()
    SAVE = auto()
    DONE = auto()


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


class WaypointPatrol(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__('waypoint_patrol')
        self._args = args
        self._session_dir = resolve_session_dir(args.session_dir)
        files = session_files(self._session_dir)
        self._waypoints = load_waypoints(files['waypoints_yaml'])
        if not self._waypoints:
            raise RuntimeError(f'No waypoints in {files["waypoints_yaml"]}')

        self._phase = Phase.WAIT_NAV2
        self._visited_ids: Set[int] = set()
        self._tsp_order: List[int] = []
        self._tsp_targets: List[Waypoint] = []
        self._tsp_cursor = 0
        self._in_sweep_pass = False
        self._sweep_round = 0
        self._current_goal_idx = -1
        self._start_pose: Optional[tuple] = None
        self._dwell_start: Optional[rclpy.time.Time] = None
        self._relocalize_start: Optional[rclpy.time.Time] = None
        self._nav_retries = 0
        self._retry_timer = None
        self._tsp_computing = False
        self._tsp_result = None
        self._backup_client: Optional[ActionClient] = None
        self._spin_client: Optional[ActionClient] = None
        self._backup_handle = None
        self._spin_handle = None
        self._backup_continue = None
        self._spin_continue = None
        self._backup_attempts = 0
        self._aborted_ids: List[int] = []
        self._last_plan_poses: List[dict] = []
        self._leg_index = 0
        self._last_waypoint_id = -1
        self._prior_pose: Optional[InitialPose] = None
        self._initialpose_sent = False
        self._shift_stable_start: Optional[rclpy.time.Time] = None
        self._nav2_wait_log_time: Optional[rclpy.time.Time] = None
        self._relocalize_log_time: Optional[rclpy.time.Time] = None
        self._traj_tracker = OdomTrajectoryTracker()
        self._last_odom_pose: Optional[tuple] = None
        self._stall_anchor: Optional[tuple] = None  # x, y, dist, time, yaw
        self._ignore_next_nav_result = False
        self._handling_nav_failure = False

        self.get_logger().info(
            f'Patrol targets: {len(self._waypoints)} positions from waypoints.yaml '
            '(CLIP paperclip detection on map; NOT ground-truth GT)'
        )

        if files['initial_pose_yaml'].is_file():
            self._prior_pose = load_initial_pose(files['initial_pose_yaml'])
            self.get_logger().info(
                f'AMCL mapping prior (map frame): x={self._prior_pose.x:.2f} '
                f'y={self._prior_pose.y:.2f} yaw={self._prior_pose.yaw:.2f} '
                f'from {files["initial_pose_yaml"]}'
            )

        self._recorder = MissionRecorder(self._session_dir, self._waypoints)
        t0 = self.get_clock().now().nanoseconds * 1e-9
        self._recorder.start(t0)
        self.get_logger().info(f'Mission recording: {self._recorder.mission_dir}')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._backup_client = ActionClient(self, BackUp, 'backup')
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )
        self._clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._goal_handle = None
        self._nav_future = None

        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._costmap: Optional[OccupancyGrid] = None
        self._local_costmap: Optional[OccupancyGrid] = None
        self._static_map = StaticMapIndex(self._session_dir)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._on_costmap, qos_map
        )
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._on_local_costmap, qos_map
        )
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._marker_pub = self.create_publisher(MarkerArray, '/navigation/waypoint_markers', 10)
        self._driven_pub = self.create_publisher(Path, '/navigation/driven_path', 10)
        self._driven_path = Path()
        self._driven_path.header.frame_id = 'map'
        self._last_driven_xy: Optional[tuple] = None

        self._sample_timer = self.create_timer(0.2, self._sample_pose)
        self._tick_timer = self.create_timer(0.1, self._tick)
        self._autosave_timer = self.create_timer(3.0, self._autosave)
        self._obstacle_scan_timer = self.create_timer(1.0, self._scan_dynamic_obstacles)

        self.get_logger().info(
            f'Loaded {len(self._waypoints)} waypoints from {self._session_dir}'
        )
        self._publish_markers()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._local_costmap = msg

    def _scan_dynamic_obstacles(self) -> None:
        if self._local_costmap is None or self._phase in (Phase.WAIT_NAV2, Phase.DONE):
            return
        points = extract_dynamic_obstacles(
            self._local_costmap,
            self._static_map,
            self._tf_buffer,
            target_frame='map',
        )
        if not points:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self._recorder.add_dynamic_obstacles(t, points)

    def _on_plan(self, msg: Path) -> None:
        self._last_plan_poses = [
            {'x': p.pose.position.x, 'y': p.pose.position.y}
            for p in msg.poses
        ]

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._last_odom_pose = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))

    def _anchor_trajectory(self, map_pose: tuple) -> None:
        if self._traj_tracker.anchored or self._last_odom_pose is None:
            return
        self._traj_tracker.reset_anchor(map_pose, self._last_odom_pose)
        self.get_logger().info(
            'Driven trace anchored: odom integration in map frame (AMCL jumps filtered)'
        )

    def _get_pose(self) -> Optional[tuple]:
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = quat_to_yaw(t.transform.rotation)
        return x, y, yaw

    def _trace_pose(self) -> Optional[tuple]:
        """Odom-integrated map pose; None until anchored (never raw AMCL)."""
        if self._last_odom_pose is None:
            return None
        if not self._traj_tracker.anchored:
            if self._phase in (Phase.WAIT_NAV2, Phase.WAIT_RELOCALIZE):
                return None
            map_pose = self._get_pose()
            if map_pose is not None:
                self._anchor_trajectory(map_pose)
            if not self._traj_tracker.anchored:
                return None
        return self._traj_tracker.update_odom(self._last_odom_pose)

    def _sample_pose(self) -> None:
        pose = self._trace_pose()
        if pose is None:
            return
        x, y, yaw = pose
        t = self.get_clock().now().nanoseconds * 1e-9
        self._recorder.add_pose_sample(t, x, y, yaw)
        self._publish_driven_path(x, y, yaw)

    def _publish_driven_path(self, x: float, y: float, yaw: float) -> None:
        if self._last_driven_xy is not None:
            if math.hypot(x - self._last_driven_xy[0], y - self._last_driven_xy[1]) < 0.05:
                return
        self._last_driven_xy = (x, y)
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = yaw_to_quat(yaw)
        self._driven_path.poses.append(ps)
        self._driven_path.header.stamp = ps.header.stamp
        self._driven_pub.publish(self._driven_path)

    def _at_waypoint(self, wp: Waypoint) -> bool:
        pose = self._get_pose()
        if pose is None:
            return False
        x, y, yaw = pose
        if math.hypot(x - wp.x, y - wp.y) > self._args.position_tolerance:
            return False
        if self._costmap is None:
            return True
        info = self._costmap.info
        return footprint_free(
            wp.x, wp.y, yaw,
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            self._costmap.data,
        )

    def _publish_markers(self) -> None:
        arr = MarkerArray()
        for i, wp in enumerate(self._waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'detected_paperclips'
            m.id = wp.id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.x
            m.pose.position.y = wp.y
            m.pose.position.z = self._args.marker_z
            m.scale.x = m.scale.y = m.scale.z = self._args.marker_scale
            if wp.id in self._visited_ids:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.95, 0.1, 0.95
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.1, 0.1, 0.95
            arr.markers.append(m)
        self._marker_pub.publish(arr)

    def _autosave(self) -> None:
        if self._phase in (Phase.DONE, Phase.SAVE):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self._recorder.save_progress(t, self._phase.name.lower(), {})

    def _footprint_cost_high(self) -> bool:
        pose = self._get_pose()
        if pose is None or self._costmap is None:
            return False
        x, y, yaw = pose
        info = self._costmap.info
        mc = footprint_max_cost(
            x, y, yaw,
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            self._costmap.data,
        )
        if mc >= self._args.high_cost_threshold:
            return True
        return not footprint_free(
            x, y, yaw,
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            self._costmap.data,
        )

    def _backup_to_free(self, continue_fn) -> None:
        if self._backup_attempts >= self._args.max_backup_attempts:
            self.get_logger().warn('Backup attempts exhausted; continuing anyway')
            self._backup_attempts = 0
            continue_fn()
            return
        if not self._backup_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('backup action server unavailable; skipping backup')
            continue_fn()
            return
        self._backup_continue = continue_fn
        self._phase = Phase.BACKUP
        goal = BackUp.Goal()
        goal.target = Point(x=float(self._args.backup_dist), y=0.0, z=0.0)
        goal.speed = float(self._args.backup_speed)
        goal.time_allowance = DurationMsg(sec=20, nanosec=0)
        self.get_logger().info(
            f'Backing up {self._args.backup_dist:.2f} m (attempt '
            f'{self._backup_attempts + 1}/{self._args.max_backup_attempts})'
        )
        send_fut = self._backup_client.send_goal_async(goal)
        send_fut.add_done_callback(self._backup_response_cb)

    def _backup_response_cb(self, future) -> None:
        self._backup_handle = future.result()
        if self._backup_handle is None or not self._backup_handle.accepted:
            self.get_logger().warn('Backup goal rejected')
            self._finish_backup(continue_anyway=True)
            return
        result_fut = self._backup_handle.get_result_async()
        result_fut.add_done_callback(self._backup_result_cb)

    def _backup_result_cb(self, future) -> None:
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Backup failed status={status}')
            self._finish_backup(continue_anyway=True)
            return
        self._backup_attempts += 1
        if self._footprint_cost_high() and self._backup_attempts < self._args.max_backup_attempts:
            self.get_logger().info('Still near inflated cost after backup; backing up again')
            self._run_backup_again()
            return
        self.get_logger().info('Backup complete; footprint in freer space')
        self._finish_backup(continue_anyway=False)

    def _run_backup_again(self) -> None:
        goal = BackUp.Goal()
        goal.target = Point(x=float(self._args.backup_dist), y=0.0, z=0.0)
        goal.speed = float(self._args.backup_speed)
        goal.time_allowance = DurationMsg(sec=20, nanosec=0)
        send_fut = self._backup_client.send_goal_async(goal)
        send_fut.add_done_callback(self._backup_response_cb)

    def _clear_costmaps(self, local_only: bool = False) -> None:
        req = ClearEntireCostmap.Request()
        clients = [self._clear_local_costmap]
        if not local_only:
            clients.append(self._clear_global_costmap)
        for client in clients:
            if client.service_is_ready():
                client.call_async(req)

    def _estimate_escape_spin_yaw(self) -> Optional[float]:
        pose = self._get_pose()
        if pose is None or self._costmap is None:
            return None
        x, y, yaw = pose
        info = self._costmap.info
        free_h = estimate_free_heading(
            x, y, yaw,
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            self._costmap.data,
        )
        target_yaw = backup_target_yaw(yaw, free_h)
        delta = math.atan2(
            math.sin(target_yaw - yaw), math.cos(target_yaw - yaw)
        )
        if abs(delta) < 0.12:
            return None
        return delta

    def _begin_escape_recovery(self, continue_fn) -> None:
        spin_delta = self._estimate_escape_spin_yaw()
        if spin_delta is not None:
            self._spin_then_backup(spin_delta, continue_fn)
            return
        self._backup_to_free(continue_fn)

    def _spin_then_backup(self, delta_yaw: float, continue_fn) -> None:
        if not self._spin_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('spin action server unavailable; backup directly')
            self._backup_to_free(continue_fn)
            return
        self._spin_continue = continue_fn
        self._phase = Phase.SPIN
        goal = Spin.Goal()
        goal.target_yaw = float(delta_yaw)
        goal.time_allowance = DurationMsg(sec=12, nanosec=0)
        self.get_logger().info(f'Spin {delta_yaw:.2f} rad toward freer space before backup')
        send_fut = self._spin_client.send_goal_async(goal)
        send_fut.add_done_callback(self._spin_response_cb)

    def _spin_response_cb(self, future) -> None:
        self._spin_handle = future.result()
        if self._spin_handle is None or not self._spin_handle.accepted:
            self.get_logger().warn('Spin goal rejected; backup directly')
            self._finish_spin(continue_anyway=True)
            return
        result_fut = self._spin_handle.get_result_async()
        result_fut.add_done_callback(self._spin_result_cb)

    def _spin_result_cb(self, future) -> None:
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Spin failed status={status}; backup anyway')
        self._finish_spin(continue_anyway=False)

    def _finish_spin(self, continue_anyway: bool) -> None:
        fn = self._spin_continue
        self._spin_continue = None
        self._spin_handle = None
        self._phase = Phase.GO_NEXT
        if fn is None:
            return
        if continue_anyway:
            fn()
            return
        self._backup_to_free(fn)

    def _current_tsp_wp(self) -> Optional[Waypoint]:
        if self._tsp_cursor < 0 or self._tsp_cursor >= len(self._tsp_targets):
            return None
        return self._tsp_targets[self._tsp_cursor]

    def _dist_to_current_waypoint(self) -> Optional[float]:
        wp = self._current_tsp_wp()
        if wp is None:
            return None
        pose = self._get_pose()
        if pose is None:
            return None
        return math.hypot(pose[0] - wp.x, pose[1] - wp.y)

    def _approach_tolerance_m(self) -> float:
        if self._in_sweep_pass:
            return self._args.sweep_approach_tolerance
        return self._args.approach_tolerance

    def _max_nav_retries(self) -> int:
        if self._in_sweep_pass:
            return self._args.sweep_max_nav_retries
        return self._args.max_nav_retries

    def _unvisited_targets(self) -> List[Waypoint]:
        return [wp for wp in self._tsp_targets if wp.id not in self._visited_ids]

    def _try_approach_success(self) -> bool:
        """Mark visited if close enough when exact nav fails (waypoint in cost)."""
        dist = self._dist_to_current_waypoint()
        wp = self._current_tsp_wp()
        tol = self._approach_tolerance_m()
        if wp is None or dist is None or dist > tol:
            return False
        self.get_logger().info(
            f'Approach success: within {dist:.2f} m of TSP leg id={wp.id} '
            f'({wp.x:.2f}, {wp.y:.2f}); marking visited'
        )
        self._mark_visited(wp)
        self._nav_retries = 0
        self._phase = Phase.DWELL
        self._dwell_start = self.get_clock().now()
        return True

    def _finish_backup(self, continue_anyway: bool) -> None:
        fn = self._backup_continue
        self._backup_continue = None
        self._backup_attempts = 0
        self._phase = Phase.GO_NEXT
        if fn is not None:
            fn()

    def _send_goal(self, wp: Optional[Waypoint] = None) -> None:
        if wp is None:
            wp = self._current_tsp_wp()
        if wp is None:
            self.get_logger().error('No TSP target for current leg')
            return
        pose = self._get_pose()
        # 只要求到达航点位置；用当前朝向作目标 yaw，避免到点后绕圈对齐
        if pose is not None:
            yaw = pose[2]
        else:
            yaw = 0.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        goal.pose.pose.orientation = yaw_to_quat(yaw)

        self._stall_anchor = None
        self._ignore_next_nav_result = False
        leg_no = self._tsp_cursor + 1
        total = len(self._tsp_targets)
        self.get_logger().info(
            f'TSP leg {leg_no}/{total}: id={wp.id} goal=({wp.x:.4f}, {wp.y:.4f})'
        )

        send_fut = self._nav_client.send_goal_async(goal)
        send_fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        self._goal_handle = future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self._on_nav_failed()
            return
        self._nav_future = self._goal_handle.get_result_async()
        self._nav_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future) -> None:
        if self._ignore_next_nav_result:
            self._ignore_next_nav_result = False
            return
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            wp = self._current_tsp_wp()
            if wp is None:
                return
            self.get_logger().info(
                f'Navigation leg succeeded (TSP id={wp.id} at {wp.x:.2f}, {wp.y:.2f}); '
                f'dwell {self._args.dwell_sec}s'
            )
            self._mark_visited(wp)
            self._nav_retries = 0
            self._phase = Phase.DWELL
            self._dwell_start = self.get_clock().now()
            return
        self.get_logger().warn(f'Navigation leg failed status={status}')
        self._on_nav_failed()

    def _cancel_nav_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

    def _schedule_nav_retry(
        self,
        delay_sec: Optional[float] = None,
        *,
        local_only: bool = False,
    ) -> None:
        if delay_sec is None:
            delay_sec = self._args.retry_delay_sec
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

        def _fire():
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
            if self._current_tsp_wp() is None:
                return
            self._clear_costmaps(local_only=local_only)
            label = 'local clear' if local_only else 'clear'
            self.get_logger().info(f'Re-sending TSP leg goal after {label} and wait')
            self._send_goal()

        self._retry_timer = self.create_timer(delay_sec, _fire)

    def _check_nav_stall(self) -> None:
        """Detect back-and-forth oscillation with no progress toward goal."""
        if self._phase != Phase.GO_NEXT or self._goal_handle is None:
            self._stall_anchor = None
            return
        pose = self._get_pose()
        dist = self._dist_to_current_waypoint()
        if pose is None or dist is None:
            return
        now = self.get_clock().now()
        if self._stall_anchor is None:
            self._stall_anchor = (pose[0], pose[1], dist, now, pose[2])
            return
        ax, ay, adist, atime, ayaw = self._stall_anchor
        elapsed = (now - atime).nanoseconds * 1e-9
        moved = math.hypot(pose[0] - ax, pose[1] - ay)
        dyaw = abs(math.atan2(
            math.sin(pose[2] - ayaw), math.cos(pose[2] - ayaw)
        ))
        if dist < adist - 0.05:
            self._stall_anchor = (pose[0], pose[1], dist, now, pose[2])
            return
        if moved < 0.15 and dyaw > 0.35:
            if (
                elapsed >= self._args.stall_spin_time_sec
                and dist >= adist - 0.03
            ):
                wp = self._current_tsp_wp()
                if wp is None:
                    return
                self.get_logger().warn(
                    f'Nav spin-stall on TSP leg id={wp.id}: '
                    f'dyaw {dyaw:.2f} rad in {elapsed:.0f}s, goal dist {dist:.2f} m'
                )
                self._stall_anchor = None
                self._ignore_next_nav_result = True
                self._cancel_nav_goal()
                self._on_nav_failed()
                return
            self._stall_anchor = (pose[0], pose[1], dist, atime, pose[2])
            return
        if (
            elapsed >= self._args.stall_time_sec
            and moved >= self._args.stall_min_travel_m
            and dist >= adist - 0.03
        ):
            wp = self._current_tsp_wp()
            if wp is None:
                return
            self.get_logger().warn(
                f'Nav stall on TSP leg id={wp.id} ({wp.x:.2f}, {wp.y:.2f}): '
                f'moved {moved:.2f} m in {elapsed:.0f}s but goal dist {dist:.2f} m '
                f'(was {adist:.2f} m)'
            )
            self._stall_anchor = None
            self._ignore_next_nav_result = True
            self._cancel_nav_goal()
            self._on_nav_failed()

    def _on_nav_failed(self) -> None:
        if self._handling_nav_failure:
            return
        self._handling_nav_failure = True
        try:
            self._on_nav_failed_impl()
        finally:
            self._handling_nav_failure = False
            self._stall_anchor = None

    def _on_nav_failed_impl(self) -> None:
        self._cancel_nav_goal()
        if self._try_approach_success():
            return
        self._nav_retries += 1
        max_retries = self._max_nav_retries()
        if self._nav_retries == 1:
            self.get_logger().info(
                f'Nav failed; quick local clear + retry in '
                f'{self._args.quick_retry_delay_sec:.1f}s (1/{max_retries})'
            )
            self._schedule_nav_retry(
                self._args.quick_retry_delay_sec,
                local_only=True,
            )
            return
        if self._nav_retries <= max_retries:
            self.get_logger().info(
                f'Nav failed; directed escape then retry in '
                f'{self._args.retry_delay_sec:.1f}s '
                f'({self._nav_retries}/{max_retries})'
            )
            self._begin_escape_recovery(
                lambda: self._schedule_nav_retry(local_only=False)
            )
            return
        if self._try_approach_success():
            return
        wp = self._current_tsp_wp()
        if wp is None:
            return
        if self._in_sweep_pass:
            self._handle_sweep_leg_failed(wp)
            return
        self.get_logger().error(
            f'Deferring TSP leg id={wp.id} ({wp.x:.2f}, {wp.y:.2f}) to sweep pass'
        )
        self._nav_retries = 0
        self._advance_after_dwell(skipped=True)

    def _save_leg(self, to_id: int) -> None:
        from_id = self._last_waypoint_id
        self._recorder.add_leg(
            self._leg_index, from_id, to_id, self._last_plan_poses
        )
        self._leg_index += 1
        self._last_plan_poses = []
        self._last_waypoint_id = to_id
        t = self.get_clock().now().nanoseconds * 1e-9
        self._recorder.save_progress(t, 'leg_saved', {'to_id': to_id})

    def _mark_visited(self, wp: Waypoint) -> None:
        self._visited_ids.add(wp.id)
        t = self.get_clock().now().nanoseconds * 1e-9
        self._recorder.record_visit(wp.id, t, self._args.dwell_sec)
        self._recorder.save_progress(t, 'visited', {'waypoint_id': wp.id})
        self._publish_markers()

    def _begin_sweep_leg_for_wp(self, wp: Waypoint) -> None:
        for i, twp in enumerate(self._tsp_targets):
            if twp.id == wp.id:
                self._tsp_cursor = i
                self._current_goal_idx = self._tsp_order[i]
                break
        self._in_sweep_pass = True
        self._nav_retries = 0
        self._phase = Phase.GO_NEXT
        if self._footprint_cost_high():
            self.get_logger().info(
                f'Robot in inflated cost before sweep leg id={wp.id}; backup first'
            )
            self._begin_escape_recovery(lambda: self._send_goal(wp))
            return
        self._send_goal(wp)

    def _start_next_sweep_round(self) -> bool:
        unvisited = self._unvisited_targets()
        if not unvisited:
            return False
        if self._sweep_round >= self._args.max_sweep_rounds:
            return False
        self._sweep_round += 1
        ids = [wp.id for wp in unvisited]
        self.get_logger().warn(
            f'TSP sweep round {self._sweep_round}/{self._args.max_sweep_rounds}: '
            f'{len(unvisited)} unvisited ids={ids}'
        )
        self._clear_costmaps()
        self._begin_sweep_leg_for_wp(unvisited[0])
        return True

    def _handle_sweep_leg_failed(self, wp: Waypoint) -> None:
        self._clear_costmaps()
        self._nav_retries = 0
        unvisited = self._unvisited_targets()
        remaining = [u for u in unvisited if u.id != wp.id]
        if remaining:
            self.get_logger().warn(
                f'Sweep could not reach id={wp.id}; trying next unvisited id={remaining[0].id}'
            )
            self._begin_escape_recovery(lambda: self._begin_sweep_leg_for_wp(remaining[0]))
            return
        self._in_sweep_pass = False
        if self._start_next_sweep_round():
            return
        self._finalize_mission_incomplete()

    def _finalize_mission_incomplete(self) -> None:
        unvisited = self._unvisited_targets()
        self._aborted_ids = [wp.id for wp in unvisited]
        self.get_logger().error(
            f'Mission finished with {len(unvisited)} unvisited TSP targets after '
            f'{self._sweep_round} sweep round(s): '
            f'{[(w.id, w.x, w.y) for w in unvisited]}'
        )
        self._phase = Phase.SAVE

    def _start_next_tsp_leg(self) -> None:
        """Navigate to next unvisited target in frozen TSP order (exact x,y)."""
        while self._tsp_cursor < len(self._tsp_targets):
            wp = self._tsp_targets[self._tsp_cursor]
            self._current_goal_idx = self._tsp_order[self._tsp_cursor]
            if wp.id in self._visited_ids:
                self._tsp_cursor += 1
                continue
            self._in_sweep_pass = False
            self._phase = Phase.GO_NEXT
            self._nav_retries = 0
            if self._footprint_cost_high():
                self.get_logger().info(
                    f'Robot in inflated cost before TSP leg id={wp.id}; backup first'
                )
                self._begin_escape_recovery(lambda: self._send_goal(wp))
                return
            self._send_goal(wp)
            return

        unvisited = self._unvisited_targets()
        if unvisited:
            if self._start_next_sweep_round():
                return
            self._finalize_mission_incomplete()
            return

        self.get_logger().info('All TSP legs visited; saving mission')
        self._phase = Phase.SAVE

    def _advance_after_dwell(self, skipped: bool = False) -> None:
        wp = self._current_tsp_wp()
        if not skipped and wp is not None:
            self._save_leg(wp.id)

        if self._in_sweep_pass:
            unvisited = self._unvisited_targets()
            if unvisited:
                self._begin_sweep_leg_for_wp(unvisited[0])
                return
            self._in_sweep_pass = False
            if self._start_next_sweep_round():
                return
            if self._unvisited_targets():
                self._finalize_mission_incomplete()
                return
            self.get_logger().info('All TSP legs visited after sweep; saving mission')
            self._phase = Phase.SAVE
            return

        self._tsp_cursor += 1
        self._start_next_tsp_leg()

    def _publish_amcl_prior(self) -> None:
        if self._prior_pose is None or self._initialpose_sent:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._prior_pose.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self._prior_pose.x
        msg.pose.pose.position.y = self._prior_pose.y
        msg.pose.pose.orientation = yaw_to_quat(self._prior_pose.yaw)
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        self._initialpose_pub.publish(msg)
        self._initialpose_sent = True
        self.get_logger().info('Published /initialpose with mapping prior for AMCL relocalization')

    def _relocalize_ready(self, pose: tuple, now: rclpy.time.Time) -> bool:
        elapsed = (now - self._relocalize_start).nanoseconds * 1e-9
        if elapsed < self._args.relocalize_min_sec:
            return False

        if self._prior_pose is None:
            if elapsed >= self._args.relocalize_settle_sec:
                return True
            return False

        shift = math.hypot(
            pose[0] - self._prior_pose.x,
            pose[1] - self._prior_pose.y,
        )
        if shift < self._args.relocalize_min_shift:
            if self._shift_stable_start is None:
                self._shift_stable_start = now
            stable = (now - self._shift_stable_start).nanoseconds * 1e-9
            if stable >= self._args.relocalize_settle_sec:
                self.get_logger().info(
                    f'AMCL aligned with mapping prior (shift={shift:.2f} m); starting navigation'
                )
                return True
            return False

        if shift >= self._args.relocalize_min_shift:
            if self._shift_stable_start is None:
                self._shift_stable_start = now
                self.get_logger().info(
                    f'AMCL shifted {shift:.2f} m from mapping prior; waiting stable...'
                )
                return False
            stable = (now - self._shift_stable_start).nanoseconds * 1e-9
            if stable >= self._args.relocalize_settle_sec:
                self.get_logger().info(
                    f'Relocalization converged: shift={shift:.2f} m from prior'
                )
                return True
            return False

        return False

    def _begin_tsp_from_pose(self, pose: tuple) -> None:
        self._anchor_trajectory(pose)
        self._start_pose = (pose[0], pose[1], pose[2])
        self._phase = Phase.COMPUTE_TSP
        t = self.get_clock().now().nanoseconds * 1e-9
        self._recorder.save_progress(
            t, 'relocalized',
            {'start_x': pose[0], 'start_y': pose[1], 'start_yaw': pose[2]},
        )
        self.get_logger().info(
            f'Relocalized at ({pose[0]:.2f}, {pose[1]:.2f}); computing optimal TSP from here'
        )

    def _compute_and_start_tsp(self) -> None:
        if self._start_pose is None or self._tsp_computing:
            return
        self._tsp_computing = True
        sx, sy, _ = self._start_pose
        waypoints = list(self._waypoints)

        def _worker() -> None:
            try:
                order, d_mat, a_mat, total_cost = tsp_from_start_pose(waypoints, sx, sy)
                self._tsp_result = (order, d_mat, a_mat, total_cost)
            except Exception as exc:
                self.get_logger().error(f'TSP computation failed: {exc}')
                self._tsp_result = ([], [], [], 0.0)

        threading.Thread(target=_worker, daemon=True).start()

    def _apply_tsp_result(self) -> None:
        if self._tsp_result is None or self._start_pose is None:
            return
        order, d_mat, a_mat, total_cost = self._tsp_result
        self._tsp_result = None
        self._tsp_computing = False
        self._tsp_order = order
        self._tsp_targets = [self._waypoints[i] for i in order]
        self._in_sweep_pass = False
        self._sweep_round = 0
        sx, sy, _ = self._start_pose
        self._recorder.set_tsp(
            order, d_mat, a_mat,
            start_pose=(sx, sy),
            optimal_manhattan_cost=total_cost,
        )
        ids = [wp.id for wp in self._tsp_targets]
        coords = [f'id={wp.id}({wp.x:.2f},{wp.y:.2f})' for wp in self._tsp_targets]
        self.get_logger().info(
            f'Optimal Manhattan TSP (Held-Karp, globally exact): '
            f'ids={ids} total={total_cost:.3f} m'
        )
        self.get_logger().info(f'TSP targets frozen: {coords}')
        self._tsp_cursor = 0
        if not self._tsp_targets:
            self._phase = Phase.SAVE
            return
        self._start_next_tsp_leg()

    def _tick(self) -> None:
        if self._phase == Phase.WAIT_NAV2:
            if self._nav_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info('Nav2 ready; waiting for AMCL relocalization')
                self._phase = Phase.WAIT_RELOCALIZE
                self._relocalize_start = self.get_clock().now()
                self._publish_amcl_prior()
                t = self.get_clock().now().nanoseconds * 1e-9
                self._recorder.save_progress(t, 'wait_relocalize', {})
            else:
                now = self.get_clock().now()
                if self._nav2_wait_log_time is None:
                    self._nav2_wait_log_time = now
                elif (now - self._nav2_wait_log_time).nanoseconds * 1e-9 >= 10.0:
                    self.get_logger().warn(
                        'Still waiting for Nav2 navigate_to_pose action server. '
                        'Terminal 2 must show lifecycle_manager_navigation: '
                        'Managed nodes are active (wait ~25–30s after terminal 2 start).'
                    )
                    self._nav2_wait_log_time = now
            return

        if self._phase == Phase.WAIT_RELOCALIZE:
            pose = self._get_pose()
            if pose is None:
                now = self.get_clock().now()
                if self._relocalize_log_time is None:
                    self._relocalize_log_time = now
                elif (now - self._relocalize_log_time).nanoseconds * 1e-9 >= 8.0:
                    self.get_logger().warn(
                        'Waiting for map→base_footprint TF (AMCL not publishing yet)'
                    )
                    self._relocalize_log_time = now
                return
            now = self.get_clock().now()
            elapsed = (now - self._relocalize_start).nanoseconds * 1e-9
            if elapsed >= self._args.relocalize_timeout_sec:
                self.get_logger().warn(
                    f'Relocalization timeout ({elapsed:.0f}s); '
                    'starting TSP with current AMCL pose'
                )
                self._begin_tsp_from_pose(pose)
                return
            if self._relocalize_ready(pose, now):
                self._begin_tsp_from_pose(pose)
            return

        if self._phase == Phase.COMPUTE_TSP:
            if self._tsp_result is not None:
                self._apply_tsp_result()
            elif not self._tsp_computing:
                self._compute_and_start_tsp()
            return

        if self._phase == Phase.GO_NEXT:
            self._check_nav_stall()
            return

        if self._phase == Phase.DWELL:
            if self._dwell_start is None:
                self._dwell_start = self.get_clock().now()
                return
            elapsed = (self.get_clock().now() - self._dwell_start).nanoseconds * 1e-9
            if elapsed >= self._args.dwell_sec:
                self._dwell_start = None
                self._advance_after_dwell()
            return

        if self._phase == Phase.SAVE:
            t = self.get_clock().now().nanoseconds * 1e-9
            out = self._recorder.finalize(t, self._aborted_ids)
            self.get_logger().info(f'Mission saved to {out}')
            self._phase = Phase.DONE

    def destroy_node(self) -> bool:
        if self._phase not in (Phase.DONE, Phase.SAVE):
            try:
                t = self.get_clock().now().nanoseconds * 1e-9
                self._recorder.finalize(t, self._aborted_ids)
            except Exception:
                pass
        return super().destroy_node()


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Waypoint patrol with Manhattan TSP tour.')
    p.add_argument(
        '--session-dir',
        default='',
        help='Map session dir (default: src/perception_pkg/maps/map_latest)',
    )
    p.add_argument('--dwell-sec', type=float, default=0.75)
    p.add_argument('--position-tolerance', type=float, default=0.12)
    p.add_argument('--approach-tolerance', type=float, default=0.40,
                   help='Mark waypoint visited if nav fails but within this distance (m).')
    p.add_argument('--stall-time-sec', type=float, default=12.0,
                   help='Trigger recovery if no goal progress while moving this long.')
    p.add_argument('--stall-spin-time-sec', type=float, default=8.0,
                   help='Treat in-place spin without goal progress as stall after this long.')
    p.add_argument('--stall-min-travel-m', type=float, default=0.25,
                   help='Min path length in stall window to count as oscillation.')
    p.add_argument('--relocalize-settle-sec', type=float, default=2.0,
                   help='Stable seconds after AMCL shifts from mapping prior.')
    p.add_argument('--relocalize-min-sec', type=float, default=5.0,
                   help='Minimum wait before accepting relocalization.')
    p.add_argument('--relocalize-min-shift', type=float, default=0.25,
                   help='AMCL pose must move this far from mapping prior (map frame).')
    p.add_argument('--relocalize-timeout-sec', type=float, default=25.0)
    p.add_argument('--max-nav-retries', type=int, default=3)
    p.add_argument('--sweep-max-nav-retries', type=int, default=6,
                   help='Nav retries per leg during post-TSP sweep pass.')
    p.add_argument('--sweep-approach-tolerance', type=float, default=0.55,
                   help='Approach success radius (m) during sweep pass.')
    p.add_argument('--max-sweep-rounds', type=int, default=4,
                   help='How many full sweep rounds over unvisited TSP legs.')
    p.add_argument('--retry-delay-sec', type=float, default=1.0,
                   help='Wait after escape backup before re-sending navigation goal.')
    p.add_argument('--quick-retry-delay-sec', type=float, default=0.8,
                   help='Wait after first-failure local clear before re-sending goal.')
    p.add_argument('--backup-dist', type=float, default=0.40,
                   help='BackUp behavior distance (m) when leaving high-cost cells.')
    p.add_argument('--backup-speed', type=float, default=0.18)
    p.add_argument('--max-backup-attempts', type=int, default=3)
    p.add_argument('--high-cost-threshold', type=int, default=140,
                   help='Footprint max cost above this triggers backup.')
    p.add_argument('--marker-scale', type=float, default=0.15)
    p.add_argument('--marker-z', type=float, default=0.05)
    return p.parse_args(argv)


def main() -> None:
    import signal
    import sys
    from rclpy.utilities import remove_ros_args

    argv = remove_ros_args(sys.argv)
    args = parse_args(argv[1:])
    rclpy.init(args=sys.argv)
    node = WaypointPatrol(args)

    def _flush_on_signal(signum, frame):
        node.get_logger().info('Signal received; flushing mission result...')
        try:
            t = node.get_clock().now().nanoseconds * 1e-9
            if node._phase in (Phase.DONE, Phase.SAVE):
                node._recorder.save_progress(t, 'interrupted', {})
            else:
                node._recorder.finalize(t, node._aborted_ids)
        except Exception:
            pass
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _flush_on_signal)
    signal.signal(signal.SIGTERM, _flush_on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
