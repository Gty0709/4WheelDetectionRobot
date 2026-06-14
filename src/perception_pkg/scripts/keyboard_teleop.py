#!/usr/bin/env python3
"""
keyboard_teleop.py — 键盘遥控 /cmd_vel

[Humble迁移] 从 /dev/tty 读键，兼容 Cursor/IDE 集成终端（teleop_twist_keyboard 易卡死）。
"""
import os
import sys
import select
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

HELP = """
键盘遥控 → /cmd_vel（请保持【本终端】焦点）
  u  i  o
  j  k  l
  m  ,  .
k/其他: 停止 | q/z: 加减总速度 | Ctrl+C 退出
"""

MOVE = {
    'i': (0.2, 0.0),
    'j': (0.0, 0.5),
    'l': (0.0, -0.5),
    'u': (0.15, 0.4),
    'o': (0.15, -0.4),
    ',': (-0.2, 0.0),
    'm': (-0.15, -0.4),
    '.': (-0.15, 0.4),
}


class TtyReader:
    """优先使用控制终端 /dev/tty，避免 stdin 非 TTY 时阻塞。"""

    def __init__(self):
        self._fd = None
        self._own_fd = False
        self._old = None

        if sys.stdin.isatty():
            self._fd = sys.stdin.fileno()
        else:
            try:
                self._fd = os.open(os.ctermid(), os.O_RDWR)
                self._own_fd = True
            except OSError:
                self._fd = os.open('/dev/tty', os.O_RDWR)
                self._own_fd = True

        self._old = termios.tcgetattr(self._fd)

    def read_key(self, timeout=0.1):
        if select.select([self._fd], [], [], timeout)[0]:
            tty.setraw(self._fd)
            try:
                ch = os.read(self._fd, 1).decode('utf-8', errors='ignore')
            finally:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            return ch
        return ''

    def close(self):
        if self._old is not None and self._fd is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        if self._own_fd and self._fd is not None:
            os.close(self._fd)


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))

    speed_scale = 1.0
    reader = TtyReader()
    print(HELP, flush=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            key = reader.read_key()
            if not key:
                continue

            twist = Twist()
            if key in MOVE:
                lin, ang = MOVE[key]
                twist.linear.x = lin * speed_scale
                twist.angular.z = ang * speed_scale
                print(f'cmd: lin={twist.linear.x:.2f} ang={twist.angular.z:.2f}', flush=True)
            elif key == 'k' or key == '\x03':
                if key == '\x03':
                    break
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print('cmd: stop', flush=True)
            elif key == 'q':
                speed_scale = min(speed_scale * 1.1, 3.0)
                print(f'速度倍率: {speed_scale:.2f}', flush=True)
                continue
            elif key == 'z':
                speed_scale = max(speed_scale * 0.9, 0.1)
                print(f'速度倍率: {speed_scale:.2f}', flush=True)
                continue
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            pub.publish(twist)
    finally:
        pub.publish(Twist())
        reader.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
