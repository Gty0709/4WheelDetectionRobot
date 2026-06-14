#!/usr/bin/env python3
"""
slider_teleop.py — 滑条遥控 /cmd_vel（差速六轮运动学解算）

控制量：车体中心线速度 v (m/s)、角速度 ω (rad/s)
轮速解算（左右两侧等速，参数与 URDF diff_drive 一致）：
  v_left  = v - ω * L / 2
  v_right = v + ω * L / 2
  ω_wheel_left  = v_left  / r
  ω_wheel_right = v_right / r
  L = 0.18 m, r = 0.04 m
"""
import math
import signal
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

WHEEL_SEPARATION = 0.18   # m，与 mickrobot_ugv_classic.urdf.xacro 一致
WHEEL_RADIUS = 0.04       # m
V_MAX = 0.8                # m/s（与 diff_drive max_accel 匹配，防弹射）
W_MAX = 1.5                # rad/s
PUBLISH_HZ = 20.0

# /cmd_vel 用标准 VOLATILE：TRANSIENT_LOCAL 会在 diff_drive 重连时重放旧速度→鬼畜自走
CMD_VEL_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def kinematics(v: float, omega: float):
    v_left = v - omega * WHEEL_SEPARATION / 2.0
    v_right = v + omega * WHEEL_SEPARATION / 2.0
    w_left = v_left / WHEEL_RADIUS
    w_right = v_right / WHEEL_RADIUS
    return v_left, v_right, w_left, w_right


class SliderTeleopNode:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('slider_teleop')
        self.node.declare_parameter('save_map_on_exit', True)
        # use_sim_time 由 launch 注入，勿在此 declare（会 ParameterAlreadyDeclaredException）
        self._save_on_exit = (
            self.node.get_parameter('save_map_on_exit').get_parameter_value().bool_value
        )
        self.cmd_pub = self.node.create_publisher(
            Twist, '/cmd_vel_teleop', CMD_VEL_QOS)
        self.wheel_pub = self.node.create_publisher(
            Float64MultiArray, '/teleop/wheel_speeds', QoSProfile(depth=10))
        self._save_client = self.node.create_client(
            Trigger, '/map_snapshot_saver/save_map')
        self.linear = 0.0
        self.angular = 0.0
        self._lock = threading.Lock()
        self._stopped = False
        self.timer = self.node.create_timer(
            1.0 / PUBLISH_HZ, self._publish)
        self._publish_stop_burst()

    def _publish_stop_burst(self, count: int = 8):
        """启动时连发零速，覆盖 DDS 里可能残留的非零 cmd_vel"""
        stop = Twist()
        for _ in range(count):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)

    def set_vel(self, linear: float, angular: float):
        # 直行时抑制滑条微量 ω，减轻 Gazebo 左右打摆
        if abs(linear) >= 0.05 and abs(angular) < 0.08:
            angular = 0.0
        if abs(angular) < 0.02:
            angular = 0.0
        if abs(linear) < 0.01:
            linear = 0.0
        with self._lock:
            self.linear = linear
            self.angular = angular

    def _publish(self):
        if self._stopped:
            return
        with self._lock:
            v, w = self.linear, self.angular
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        vl, vr, wl, wr = kinematics(v, w)
        arr = Float64MultiArray()
        arr.data = [vl, vr, wl, wr]
        self.wheel_pub.publish(arr)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def stop_publishing(self):
        self._stopped = True
        if self.timer is not None:
            self.node.destroy_timer(self.timer)
            self.timer = None

    def shutdown(self):
        self.stop_publishing()
        self.publish_stop()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def request_save_map(self) -> None:
        if not self._save_on_exit or not rclpy.ok():
            return
        if not self._save_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('save_map 服务不可用（请先启动 SLAM）')
            return
        future = self._save_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.done() and future.result().success:
            self.node.get_logger().info(f'地图已保存: {future.result().message}')
        elif future.done():
            self.node.get_logger().warn('地图保存失败')


class SliderTeleopGui:
    def __init__(self, ros_node: SliderTeleopNode):
        self.ros = ros_node
        self._closing = False
        self.root = tk.Tk()
        self.root.title('MickRobot 滑条遥控')
        self.root.geometry('420x320')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, text='中心线速度 v (m/s)', font=('', 11, 'bold')).pack(anchor=tk.W)
        self.lin_var = tk.DoubleVar(value=0.0)
        lin = ttk.Scale(main, from_=-V_MAX, to=V_MAX, variable=self.lin_var,
                        orient=tk.HORIZONTAL, command=self._on_change)
        lin.pack(fill=tk.X, pady=(4, 2))
        self.lin_label = ttk.Label(main, text='v = 0.000 m/s')
        self.lin_label.pack(anchor=tk.W)

        ttk.Label(main, text='角速度 ω (rad/s)', font=('', 11, 'bold')).pack(anchor=tk.W, pady=(12, 0))
        self.ang_var = tk.DoubleVar(value=0.0)
        ang = ttk.Scale(main, from_=-W_MAX, to=W_MAX, variable=self.ang_var,
                        orient=tk.HORIZONTAL, command=self._on_change)
        ang.pack(fill=tk.X, pady=(4, 2))
        self.ang_label = ttk.Label(main, text='ω = 0.000 rad/s')
        self.ang_label.pack(anchor=tk.W)

        ttk.Separator(main, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=12)

        ttk.Label(main, text='差速运动学解算（左右侧轮）', font=('', 10, 'bold')).pack(anchor=tk.W)
        self.wheel_label = ttk.Label(
            main,
            text='v_L=0.000  v_R=0.000 m/s\nω_L=0.00  ω_R=0.00 rad/s',
            justify=tk.LEFT,
        )
        self.wheel_label.pack(anchor=tk.W, pady=4)

        btn_row = ttk.Frame(main)
        btn_row.pack(fill=tk.X, pady=8)
        ttk.Button(btn_row, text='停止', command=self._stop).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text='保存地图', command=self._save_map).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text='退出', command=self._on_close).pack(side=tk.RIGHT, padx=4)

        ttk.Label(
            main,
            text=f'发布: /cmd_vel_teleop @ {PUBLISH_HZ:.0f}Hz  |  L={WHEEL_SEPARATION}m  r={WHEEL_RADIUS}m',
            foreground='gray',
        ).pack(anchor=tk.W)

    def _on_change(self, _=None):
        if self._closing:
            return
        v = self.lin_var.get()
        w = self.ang_var.get()
        self.ros.set_vel(v, w)
        vl, vr, wl, wr = kinematics(v, w)
        self.lin_label.config(text=f'v = {v:+.3f} m/s')
        self.ang_label.config(text=f'ω = {w:+.3f} rad/s')
        self.wheel_label.config(
            text=f'v_L={vl:+.3f}  v_R={vr:+.3f} m/s\nω_L={wl:+.2f}  ω_R={wr:+.2f} rad/s')

    def _stop(self):
        self.lin_var.set(0.0)
        self.ang_var.set(0.0)
        self.ros.set_vel(0.0, 0.0)
        self.ros.publish_stop()
        self._on_change()

    def _save_map(self):
        threading.Thread(target=self.ros.request_save_map, daemon=True).start()

    def _on_close(self):
        if self._closing:
            return
        self._closing = True
        self._stop()
        self.ros.stop_publishing()
        if self.ros._save_on_exit:
            threading.Thread(target=self.ros.request_save_map, daemon=True).start()
        try:
            self.root.quit()
            self.root.destroy()
        except tk.TclError:
            pass

    def _spin_ros(self):
        if self._closing or not rclpy.ok():
            return
        rclpy.spin_once(self.ros.node, timeout_sec=0)
        self.root.after(int(1000 / PUBLISH_HZ), self._spin_ros)

    def run(self):
        self.root.after(int(1000 / PUBLISH_HZ), self._spin_ros)
        self.root.mainloop()


def main():
    ros = SliderTeleopNode()
    gui = SliderTeleopGui(ros)

    def handle_signal(_signum, _frame):
        # Ctrl-C / launch SIGTERM：必须在 Tk 主线程里关窗口
        if not gui._closing:
            gui.root.after(0, gui._on_close)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        gui.run()
    finally:
        ros.shutdown()


if __name__ == '__main__':
    main()
