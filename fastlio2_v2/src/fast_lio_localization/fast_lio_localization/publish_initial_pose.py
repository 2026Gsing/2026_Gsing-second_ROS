#!/usr/bin/env python3
"""
publish_initial_pose.py — 命令行初始位姿发布工具

功能：
  从命令行参数读取位姿 (x, y, z, yaw, pitch, roll)，发布 /initialpose 话题。
  等同于点 RViz 的 "2D Pose Estimate" 工具，但适用于无 GUI 环境或脚本调用。

使用方式：
  ros2 run fast_lio_localization publish_initial_pose.py 0.0 0.0 0.0 0.0 0.0 0.0

参数顺序：x y z yaw pitch roll（角度制）
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import tf_transformations


class PublishInitialPose(Node):
    """发布初始位姿到 /initialpose 话题，用于初始化 FAST-LIO2 全局定位"""

    def __init__(self):
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        """发布初始位姿（欧拉角转四元数）"""
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        xyz = [x, y, z]

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        self.pub_pose.publish(initial_pose)

        self.get_logger().info(f"Initial Pose: x={x}, y={y}, z={z}, yaw={yaw}, pitch={pitch}, roll={roll}")


def main(args=None):
    rclpy.init(args=args)
    node = PublishInitialPose()

    parser = argparse.ArgumentParser(
        description="发布初始位姿到 FAST-LIO2 全局定位节点"
    )
    parser.add_argument("x", type=float, help="X 坐标 (m)")
    parser.add_argument("y", type=float, help="Y 坐标 (m)")
    parser.add_argument("z", type=float, help="Z 坐标 (m)")
    parser.add_argument("yaw", type=float, help="偏航角 (rad)")
    parser.add_argument("pitch", type=float, help="俯仰角 (rad)")
    parser.add_argument("roll", type=float, help="横滚角 (rad)")
    args = parser.parse_args()

    node.publish_pose(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
