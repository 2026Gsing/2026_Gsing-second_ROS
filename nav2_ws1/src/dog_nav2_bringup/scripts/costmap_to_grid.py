#!/usr/bin/env python3
"""
costmap_to_grid.py — Nav2 costmap 话题转发（用于 RViz 可视化）

背景：
  Nav2 的 costmap 话题 (/global_costmap/costmap, /local_costmap/costmap)
  使用 transient_local QoS，这意味着：
  1. 只有话题发布后才订阅的节点才能收到最新数据
  2. 部分默认 RViz 配置无法直接显示这些话题

解决方案：
  本节点以同样的 transient_local QoS 订阅 Nav2 costmap 话题，
  然后以相同 QoS 发布到新话题：
    - /global_costmap/costmap → /global_costmap_grid
    - /local_costmap/costmap  → /local_costmap_grid
  这样 RViz 可以轻松显示这些重发布的话题。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid


class CostmapToGrid(Node):
    def __init__(self):
        super().__init__("costmap_to_grid")

        # Nav2 costmap 话题使用 transient_local 可靠性
        # 这样新订阅者（包括延迟启动的 RViz）也能收到最新的 costmap
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 发布重命名的 costmap 话题供 RViz 显示
        self.global_pub = self.create_publisher(
            OccupancyGrid, "/global_costmap_grid", qos
        )
        self.local_pub = self.create_publisher(
            OccupancyGrid, "/local_costmap_grid", qos
        )

        # 订阅原始 Nav2 costmap
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._global_cb, qos
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._local_cb, qos
        )

        self.get_logger().info(
            "Publishing /global_costmap_grid and /local_costmap_grid for RViz"
        )

    def _global_cb(self, msg: OccupancyGrid):
        """转发全局 costmap"""
        self.global_pub.publish(msg)

    def _local_cb(self, msg: OccupancyGrid):
        """转发局部 costmap"""
        self.local_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapToGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

