#!/usr/bin/env python3
"""
cmd_vel_guard.py — 唯一向 Gazebo diff_drive 发布 /cmd_vel

- 默认持续发零速（30Hz），防止 DDS/僵尸节点残留非零速度
- 转发 /cmd_vel_nav（Nav2）与 /cmd_vel_teleop（滑条/键盘）；遥控优先
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

CMD_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# Nav2 controller_server 发布 depth=1 的 cmd_vel
NAV_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
RATE_HZ = 30.0
TELEOP_STALE_SEC = 0.35
NAV_STALE_SEC = 0.5


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self._out = self.create_publisher(Twist, '/cmd_vel', CMD_QOS)
        self.create_subscription(
            Twist, '/cmd_vel_teleop', self._on_teleop, CMD_QOS)
        self.create_subscription(
            Twist, '/cmd_vel_nav', self._on_nav, NAV_QOS)
        self._teleop = Twist()
        self._nav = Twist()
        self._have_teleop = False
        self._have_nav = False
        self._last_teleop_ns = 0
        self._last_nav_ns = 0
        self._nav_log_left = 3
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self._publish_zero_burst(40)
        self.get_logger().info(
            'Forwarding /cmd_vel_nav (Nav2) and /cmd_vel_teleop → /cmd_vel; teleop wins'
        )

    def _publish_zero_burst(self, count: int) -> None:
        if not rclpy.ok():
            return
        z = Twist()
        for _ in range(count):
            self._out.publish(z)

    def _teleop_active(self, now_ns: int) -> bool:
        if not self._have_teleop:
            return False
        age = (now_ns - self._last_teleop_ns) * 1e-9
        return age <= TELEOP_STALE_SEC

    def _on_teleop(self, msg: Twist) -> None:
        self._teleop = msg
        self._have_teleop = True
        self._last_teleop_ns = self.get_clock().now().nanoseconds
        self._out.publish(msg)

    def _on_nav(self, msg: Twist) -> None:
        self._nav = msg
        self._have_nav = True
        self._last_nav_ns = self.get_clock().now().nanoseconds
        if self._nav_log_left > 0 and (
            abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4
        ):
            self.get_logger().info(
                f'Nav2 cmd_vel_nav: vx={msg.linear.x:.3f} wz={msg.angular.z:.3f}'
            )
            self._nav_log_left -= 1
        if not self._teleop_active(self._last_nav_ns):
            self._out.publish(msg)

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._teleop_active(now_ns):
            self._out.publish(self._teleop)
            return
        if self._have_nav:
            age = (now_ns - self._last_nav_ns) * 1e-9
            if age <= NAV_STALE_SEC:
                self._out.publish(self._nav)
                return
        self._out.publish(Twist())


def main():
    rclpy.init()
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero_burst(10)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
