#!/usr/bin/env python3
"""
odom_to_tf.py — 将 /odom 话题转换为 odom→base_footprint TF

避免使用 Gazebo /tf 桥接（会带入所有实体位姿，与 rtabmap map→odom 冲突）。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, '/odom', self._cb, qos)

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id        # 'odom'
        t.child_frame_id  = msg.child_frame_id         # 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
