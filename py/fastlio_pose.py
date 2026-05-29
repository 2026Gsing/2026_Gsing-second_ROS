#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fast-LIO2 定位坐标输出节点
订阅 fastlio 的里程计话题，实时打印当前定位坐标 (x, y, z)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class FastLioPose(Node):
    def __init__(self):
        super().__init__('fastlio_pose')

        # Fast-LIO2 默认话题名，如果不对可改为 /Odometry 或其他
        topic = self.declare_parameter('topic', '/Odometry').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f"Fast-LIO2 定位节点已启动，订阅话题: {topic}")
        self.get_logger().info("=" * 50)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        # 输出带 3 位小数的坐标
        print(f"\r  x: {p.x:8.3f}   y: {p.y:8.3f}   z: {p.z:8.3f}   ", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
