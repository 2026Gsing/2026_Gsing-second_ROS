#!/usr/bin/env python3
"""
goal_pose_to_nav2.py — RViz 2D Goal Pose → Nav2 NavigateToPose 桥接

功能：
  订阅 /goal_pose 话题（RViz "2D Goal Pose" 工具的发布话题），
  将其转换为 Nav2 NavigateToPose action 发送给 bt_navigator。

工作流程：
  RViz 点击地图上某点 → 发布 /goal_pose (PoseStamped)
  → 本节点接收 → 创建 NavigateToPose action goal → 发送到 bt_navigator
  → Nav2 开始规划路径并控制机器人运动

为什么需要这个桥接：
  RViz 的 "2D Goal Pose" 默认发布 topic /goal_pose (PoseStamped)，
  但 Nav2 通过 action (NavigateToPose) 接收目标。本节点做话题→动作的转换。
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseToNav2(Node):
    def __init__(self):
        super().__init__("goal_pose_to_nav2")
        # Action client: 连接 bt_navigator 的 navigate_to_pose action server
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 订阅 RViz 的 2D Goal Pose 话题
        self._sub = self.create_subscription(PoseStamped, "goal_pose", self._goal_cb, 10)
        self.get_logger().info("Bridge ready: goal_pose -> NavigateToPose (navigate_to_pose)")

    def _goal_cb(self, msg: PoseStamped):
        """收到 RViz 目标点 → 发送 NavigateToPose action"""
        self.get_logger().info(
            "Received goal (%.2f, %.2f) in frame '%s'"
            % (msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)
        )

        # 等待 Nav2 action server 就绪（最长 15 秒）
        if not self._action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warn("NavigateToPose action server not available (yet).")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseToNav2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

