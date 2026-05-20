#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid


class CostmapToGrid(Node):
    def __init__(self):
        super().__init__("costmap_to_grid")

        # Nav2 costmap topics are latched/transient_local.
        # Use transient_local here so late RViz subscribers still receive the latest map.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.global_pub = self.create_publisher(
            OccupancyGrid, "/global_costmap_grid", qos
        )
        self.local_pub = self.create_publisher(
            OccupancyGrid, "/local_costmap_grid", qos
        )

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
        self.global_pub.publish(msg)

    def _local_cb(self, msg: OccupancyGrid):
        self.local_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapToGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

