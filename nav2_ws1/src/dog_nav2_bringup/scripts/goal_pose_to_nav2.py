#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseToNav2(Node):
    def __init__(self):
        super().__init__("goal_pose_to_nav2")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._sub = self.create_subscription(PoseStamped, "goal_pose", self._goal_cb, 10)
        self.get_logger().info("Bridge ready: goal_pose -> NavigateToPose (navigate_to_pose)")

    def _goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            "Received goal (%.2f, %.2f) in frame '%s'"
            % (msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)
        )

        # Wait a bit longer; Nav2 action server appears only after lifecycle activation
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

