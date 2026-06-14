#!/usr/bin/env python3
"""
patrol_mapper.py — small_house 贴墙巡逻

路径规则（与用户标注一致）：
  1. 起点 → 东墙 → 南下至东南角（首段直行+转弯）
  2. 航点 = 柱心到最近外墙角的连线，取 patrol 车道上的目标点
  3. 第二点 → 第三点若直走会被内墙挡，走「几」字：西↑→横穿→东↓，内墙在豁口内
控制：拐角提前转向 + 行驶中允许小角误差，避免停走停走卡顿
"""
from __future__ import annotations

import math
import sys
import threading
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger

WALL_CLEAR = 0.68
PILLAR_CLEAR = 0.82

# 外墙内缘（world 坐标）
_WALL_XE = 5.0 - 0.075
_WALL_XW = -5.0 + 0.075
_WALL_YN = 4.0 - 0.075
_WALL_YS = -4.0 + 0.075

# patrol 车道（外墙内缩 WALL_CLEAR）
_X_E = _WALL_XE - WALL_CLEAR
_X_W = _WALL_XW + WALL_CLEAR
_Y_N = _WALL_YN - WALL_CLEAR
_Y_S = _WALL_YS + WALL_CLEAR
_X_INNER_E = -3.0 + 0.075 + WALL_CLEAR
_X_INNER_W = -3.0 - 0.075 - WALL_CLEAR
_Y_INNER_HALF = 2.075
_Y_S_BRIDGE = -(_Y_INNER_HALF + 0.10)
_Y_N_BRIDGE = _Y_INNER_HALF + 0.10

# 四柱中心 (world)
_OBS = {
    'SE': (3.0, -2.5),
    'SW': (-3.0, -2.5),
    'NE': (3.0, 2.5),
    'NW': (-3.0, 2.5),
}


def _axis_bearing(x0: float, y0: float, x1: float, y1: float) -> float:
    dx, dy = x1 - x0, y1 - y0
    if abs(dx) >= abs(dy):
        return 0.0 if dx >= 0 else math.pi
    return math.pi / 2 if dy >= 0 else -math.pi / 2


def _expand_manhattan(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not pts:
        return pts
    out = [pts[0]]
    for x, y in pts[1:]:
        px, py = out[-1]
        if abs(x - px) > 0.05 and abs(y - py) > 0.05:
            out.append((x, py))
        if out[-1] != (x, y):
            out.append((x, y))
    return out


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _nearest_outer_corner(ob_x: float, ob_y: float) -> tuple[float, float]:
    """柱心到最近外墙角（内缘）。"""
    corners = [
        (_WALL_XE, _WALL_YS),
        (_WALL_XE, _WALL_YN),
        (_WALL_XW, _WALL_YS),
        (_WALL_XW, _WALL_YN),
    ]
    ox, oy = min(corners, key=lambda c: (ob_x - c[0]) ** 2 + (ob_y - c[1]) ** 2)
    return ox, oy


def _patrol_point(ob_x: float, ob_y: float) -> tuple[float, float]:
    """
    柱心→最近墙角连线，在 patrol 车道上的落点。
    优先取与南/北车道 (y=±Y_S/N) 的交点；交点超出车道则取对应 inset 墙角。
    """
    cx, cy = _nearest_outer_corner(ob_x, ob_y)
    dx, dy = cx - ob_x, cy - ob_y
    if abs(dy) < 1e-9:
        lane_y = _Y_S if ob_y < 0 else _Y_N
        x = ob_x
    else:
        lane_y = _Y_S if ob_y < 0 else _Y_N
        t = (lane_y - ob_y) / dy
        x = ob_x + t * dx
    # 贴 patrol 外廓
    x = clip(x, _X_W, _X_E)
    # 柱体外侧：沿柱心→角方向留 PILLAR_CLEAR
    rad = 0.25
    if ob_x > 0:
        x = max(x, ob_x + rad + PILLAR_CLEAR)
    else:
        x = min(x, ob_x - rad - PILLAR_CLEAR)
    return (x, lane_y)


def _inner_south_zigzag() -> list[tuple[float, float]]:
    """第二点(西南) → 第三点(东北) 南墙几字，内墙在豁口内。"""
    return [
        (_X_INNER_W, _Y_S),
        (_X_INNER_W, _Y_S_BRIDGE),
        (_X_INNER_E, _Y_S_BRIDGE),
        (_X_INNER_E, _Y_S),
    ]


def _inner_north_zigzag() -> list[tuple[float, float]]:
    """东墙北 → 西北 几字。"""
    return [
        (_X_INNER_E, _Y_N),
        (_X_INNER_E, _Y_N_BRIDGE),
        (_X_INNER_W, _Y_N_BRIDGE),
        (_X_INNER_W, _Y_N),
    ]


def _build_waypoints() -> list[tuple[float, float]]:
    p_se = _patrol_point(*_OBS['SE'])
    p_sw = _patrol_point(*_OBS['SW'])
    p_ne = _patrol_point(*_OBS['NE'])
    p_nw = _patrol_point(*_OBS['NW'])

    pts: list[tuple[float, float]] = [
        (0.0, 0.0),
        (_X_E, 0.0),
        (_X_E, _Y_S),          # ① 首段直行+转弯（东南 inset 角）
        p_se,                  # ② SE 柱：柱心→东南角连线落点
        p_sw,                  # ③ SW 柱：柱心→西南角连线落点
        *_inner_south_zigzag(),  # 几字穿内墙 → 可到东侧
        p_se,                  # 回到 SE 车道
        (_X_E, _Y_S),
        (_X_E, _Y_N),          # ④ 第三点方向：东北（直走会被内墙挡，上面已几字）
        p_ne,
        *_inner_north_zigzag(),
        p_nw,
        (_X_W, _Y_N),
        (_X_W, _Y_S),
        (_X_W, 0.0),
        (0.0, 0.0),
    ]
    return _expand_manhattan(pts)


SMALL_HOUSE_WAYPOINTS = _build_waypoints()


class _Phase(str, Enum):
    WAIT_ODOM = 'wait'
    DRIVE = 'drive'
    DONE = 'done'


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PatrolMapper(Node):
    CORNER_APPROACH = 0.25
    TURN_IN_PLACE = 0.50
    STUCK_SEC = 12.0
    STUCK_MOVE = 0.04

    def __init__(self):
        super().__init__('patrol_mapper')
        self.declare_parameter('save_map_on_finish', True)
        self.declare_parameter('linear_speed', 0.11)
        self.declare_parameter('angular_speed', 0.48)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('yaw_kp', 2.4)
        self.declare_parameter('xtrack_kp', 0.9)

        self._save_on_finish = self.get_parameter('save_map_on_finish').get_parameter_value().bool_value
        self._v_max = self.get_parameter('linear_speed').get_parameter_value().double_value
        self._w_max = self.get_parameter('angular_speed').get_parameter_value().double_value
        self._goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self._yaw_kp = self.get_parameter('yaw_kp').get_parameter_value().double_value
        self._xtrack_kp = self.get_parameter('xtrack_kp').get_parameter_value().double_value

        self._pose: tuple[float, float, float] | None = None
        self._phase = _Phase.WAIT_ODOM
        self._wp_i = 1
        self._segment_yaw = 0.0
        self._seg_horizontal = True
        self._seg_from = (0.0, 0.0)
        self._running = True
        self._stuck_t0: float | None = None
        self._stuck_xy: tuple[float, float] | None = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self._save_client = self.create_client(Trigger, '/map_snapshot_saver/save_map')

        self.get_logger().info(
            f'贴墙巡逻 {len(SMALL_HOUSE_WAYPOINTS)-1} 段，'
            f'墙距 {WALL_CLEAR}m，kp={self._yaw_kp}'
        )
        for i, p in enumerate(SMALL_HOUSE_WAYPOINTS):
            self.get_logger().info(f'  wp{i}: ({p[0]:.2f}, {p[1]:.2f})')
        self.create_timer(0.05, self._control_tick)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if self._phase == _Phase.WAIT_ODOM:
            self._begin_segment()
            self._phase = _Phase.DRIVE
            gx, gy = SMALL_HOUSE_WAYPOINTS[self._wp_i]
            self.get_logger().info(f'开始巡逻 → ({gx:.2f}, {gy:.2f})')

    def _safe_publish(self, lin: float, ang: float) -> None:
        if not rclpy.ok() or not self._running:
            return
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        try:
            self._cmd_pub.publish(msg)
        except Exception:
            pass

    def _stop(self) -> None:
        self._safe_publish(0.0, 0.0)

    def _goal(self) -> tuple[float, float]:
        return SMALL_HOUSE_WAYPOINTS[self._wp_i]

    def _next_goal(self) -> tuple[float, float] | None:
        if self._wp_i + 1 >= len(SMALL_HOUSE_WAYPOINTS):
            return None
        return SMALL_HOUSE_WAYPOINTS[self._wp_i + 1]

    def _begin_segment(self) -> None:
        self._seg_from = SMALL_HOUSE_WAYPOINTS[self._wp_i - 1]
        fx, fy = self._seg_from
        gx, gy = self._goal()
        self._segment_yaw = _axis_bearing(fx, fy, gx, gy)
        self._seg_horizontal = abs(gx - fx) >= abs(gy - fy)

    def _along_dist(self, x: float, y: float) -> float:
        gx, gy = self._goal()
        if self._seg_horizontal:
            return abs(gx - x)
        return abs(gy - y)

    def _dist_to_goal(self, x: float, y: float) -> float:
        gx, gy = self._goal()
        return math.hypot(gx - x, gy - y)

    def _segment_done(self, x: float, y: float) -> bool:
        """仅看沿段方向是否到位（横向漂移不阻塞换点）。"""
        gx, gy = self._goal()
        if self._seg_horizontal:
            return abs(gx - x) < self._goal_tol
        return abs(gy - y) < self._goal_tol

    def _cross_track_err(self, x: float, y: float) -> float:
        gx, gy = self._goal()
        if self._seg_horizontal:
            return y - gy
        return x - gx

    def _xtrack_w(self, xte: float) -> float:
        """横向偏差 → 角速度修正（符号随段方向）。"""
        if self._seg_horizontal:
            # 东向(0)偏北需右转(负)；西向(π)偏北需左转(正)
            sign = -1.0 if abs(self._segment_yaw) < 0.01 else 1.0
        else:
            # 北向(π/2)偏东需左转(正)；南向(-π/2)偏东需右转(负)
            sign = 1.0 if self._segment_yaw > 0 else -1.0
        return sign * self._xtrack_kp * xte

    def _steer_bearing(self, x: float, y: float) -> float:
        """到当前航点足够近时提前朝下一航向转（用欧氏距离，避免未到时提前转）。"""
        nxt = self._next_goal()
        if nxt is None:
            return self._segment_yaw
        if self._segment_done(x, y) or self._dist_to_goal(x, y) < self.CORNER_APPROACH:
            gx, gy = self._goal()
            return _axis_bearing(gx, gy, nxt[0], nxt[1])
        return self._segment_yaw

    def _check_stuck(self, x: float, y: float, now: float) -> bool:
        if self._stuck_xy is None:
            self._stuck_xy = (x, y)
            self._stuck_t0 = now
            return False
        assert self._stuck_t0 is not None
        moved = math.hypot(x - self._stuck_xy[0], y - self._stuck_xy[1])
        if moved > self.STUCK_MOVE:
            self._stuck_xy = (x, y)
            self._stuck_t0 = now
            return False
        if now - self._stuck_t0 > self.STUCK_SEC:
            return True
        return False

    def _reset_stuck(self) -> None:
        self._stuck_t0 = None
        self._stuck_xy = None

    def _advance_waypoint(self) -> None:
        self._wp_i += 1
        if self._wp_i >= len(SMALL_HOUSE_WAYPOINTS):
            self._phase = _Phase.DONE
            self._stop()
            self.get_logger().info('墙边巡逻完成')
            threading.Thread(target=self._request_save, daemon=True).start()
            return
        self._begin_segment()
        self._reset_stuck()
        gx, gy = self._goal()
        self.get_logger().info(f'→ 航点 ({gx:.2f}, {gy:.2f})')

    def _control_tick(self) -> None:
        if not self._running or self._phase in (_Phase.WAIT_ODOM, _Phase.DONE):
            return
        if self._pose is None:
            return

        x, y, yaw = self._pose
        dist = self._along_dist(x, y)
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._check_stuck(x, y, now):
            d = self._dist_to_goal(x, y)
            self.get_logger().warn(
                f'卡住 {self.STUCK_SEC:.0f}s，强制换点 '
                f'(距目标 {d:.2f}m, pos=({x:.2f},{y:.2f}))'
            )
            self._advance_waypoint()
            return

        if self._segment_done(x, y):
            self._advance_waypoint()
            return

        target_yaw = self._steer_bearing(x, y)
        herr = norm_angle(target_yaw - yaw)
        xte = self._cross_track_err(x, y)
        w_corr = clip(self._yaw_kp * herr + self._xtrack_w(xte), -self._w_max, self._w_max)

        if abs(herr) > self.TURN_IN_PLACE:
            v = 0.0 if abs(herr) > 0.85 else min(0.04, self._v_max * 0.25)
        elif abs(herr) > 0.18:
            v = min(0.06, self._v_max * 0.45)
        else:
            v = min(self._v_max, max(0.06, dist * 0.75))

        self._safe_publish(v, w_corr)

    def _request_save(self) -> None:
        if not self._save_on_finish or not rclpy.ok():
            return
        time.sleep(2.5)
        if not self._save_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().warn('save_map 服务不可用')
            return
        future = self._save_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.done() and future.result().success:
            self.get_logger().info('巡逻结束，地图已保存')

    def destroy_node(self) -> None:
        self._running = False
        self._stop()
        super().destroy_node()


def main():
    rclpy.init(args=remove_ros_args(args=sys.argv))
    node = PatrolMapper()
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
