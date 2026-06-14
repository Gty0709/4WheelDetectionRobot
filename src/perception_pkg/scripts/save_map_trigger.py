#!/usr/bin/env python3
"""调用 /map_snapshot_saver/save_map 服务后退出。"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SaveMapTrigger(Node):
    def __init__(self):
        super().__init__('save_map_trigger')
        self._client = self.create_client(Trigger, '/map_snapshot_saver/save_map')
        self._done = False
        self.create_timer(0.5, self._tick)

    def _tick(self):
        if self._done:
            return
        if not self._client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info('等待 map_snapshot_saver 服务...')
            return
        self._done = True
        future = self._client.call_async(Trigger.Request())
        future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f'地图保存成功: {res.message}')
            else:
                self.get_logger().warn(f'地图保存失败: {res.message}')
        except Exception as exc:
            self.get_logger().error(f'调用 save_map 失败: {exc}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SaveMapTrigger()
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
