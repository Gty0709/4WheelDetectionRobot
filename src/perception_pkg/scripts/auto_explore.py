#!/usr/bin/env python3
"""
auto_explore.py
基于激光雷达的自主探索：实时检测障碍，自动转向，无需固定路径。
用法：python3 auto_explore.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import math


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # 运动参数
        self.linear_vel  = 0.2   # 前进速度 m/s
        self.turn_vel    = 0.5   # 转向速度 rad/s
        self.safe_dist   = 0.5   # 前方安全距离 m（小于此距离则转向）
        self.side_dist   = 0.3   # 侧方安全距离 m

        self.state       = 'forward'   # forward / turn
        self.turn_dir    = 1.0         # 1=左转, -1=右转
        self.turn_count  = 0           # 转向持续帧数
        self.turn_target = 0           # 需要转多少帧

        self.scan_data   = None
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('AutoExplorer 启动，按 Ctrl+C 停止')

    def scan_data_range(self, scan, angle_min_deg, angle_max_deg):
        """取激光雷达某角度范围内的最小距离（过滤无效值）"""
        n = len(scan.ranges)
        total_deg = math.degrees(scan.angle_max - scan.angle_min)
        idx_min = int((angle_min_deg - math.degrees(scan.angle_min)) / total_deg * n)
        idx_max = int((angle_max_deg - math.degrees(scan.angle_min)) / total_deg * n)
        idx_min = max(0, min(idx_min, n - 1))
        idx_max = max(0, min(idx_max, n - 1))
        if idx_min > idx_max:
            idx_min, idx_max = idx_max, idx_min
        values = [
            scan.ranges[i] for i in range(idx_min, idx_max + 1)
            if scan.range_min < scan.ranges[i] < scan.range_max
        ]
        return min(values) if values else float('inf')

    def scan_cb(self, msg):
        self.scan_data = msg

    def control_loop(self):
        if self.scan_data is None:
            return

        scan = self.scan_data

        # 检测各方向距离
        front  = self.scan_data_range(scan, -30,  30)   # 前方 ±30°
        front_l = self.scan_data_range(scan,  30,  60)  # 左前
        front_r = self.scan_data_range(scan, -60, -30)  # 右前

        cmd = Twist()

        if self.state == 'forward':
            if front < self.safe_dist:
                # 前方有障碍，决定转向方向（哪边空旷转哪边）
                self.state = 'turn'
                if front_l >= front_r:
                    self.turn_dir = 1.0   # 左转
                else:
                    self.turn_dir = -1.0  # 右转
                # 随机转 60°~150°
                angle = random.uniform(math.pi / 3, math.pi * 5 / 6)
                self.turn_target = int(angle / self.turn_vel / 0.1)
                self.turn_count = 0
            else:
                # 前方安全，前进（侧边太近则轻微修正）
                cmd.linear.x = self.linear_vel
                if front_l < self.side_dist:
                    cmd.angular.z = -0.3   # 右偏
                elif front_r < self.side_dist:
                    cmd.angular.z = 0.3    # 左偏
                # 随机小幅转向，避免一直走直线
                elif random.random() < 0.05:
                    cmd.angular.z = random.uniform(-0.2, 0.2)

        if self.state == 'turn':
            cmd.angular.z = self.turn_dir * self.turn_vel
            self.turn_count += 1
            if self.turn_count >= self.turn_target:
                self.state = 'forward'

        self.pub.publish(cmd)

    def stop(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info('已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
