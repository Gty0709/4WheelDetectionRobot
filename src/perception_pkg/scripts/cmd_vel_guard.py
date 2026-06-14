#!/usr/bin/env python3
"""
cmd_vel_guard.py — 唯一向 Gazebo diff_drive 发布 /cmd_vel

- 默认持续发零速（30Hz），防止 DDS/僵尸节点残留非零速度
- 仅转发 /cmd_vel_teleop（滑条/键盘）的指令
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
RATE_HZ = 30.0
STALE_SEC = 0.35


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self._out = self.create_publisher(Twist, '/cmd_vel', CMD_QOS)
        self.create_subscription(
            Twist, '/cmd_vel_teleop', self._on_teleop, CMD_QOS)
        self._cmd = Twist()
        self._have_teleop = False
        self._last_teleop_ns = 0
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self._publish_zero_burst(40)

    def _publish_zero_burst(self, count: int) -> None:
        if not rclpy.ok():
            return
        z = Twist()
        for _ in range(count):
            self._out.publish(z)

    def _on_teleop(self, msg: Twist) -> None:
        self._cmd = msg
        self._have_teleop = True
        self._last_teleop_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        out = Twist()
        if self._have_teleop:
            age = (self.get_clock().now().nanoseconds - self._last_teleop_ns) * 1e-9
            if age <= STALE_SEC:
                out = self._cmd
        self._out.publish(out)


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
