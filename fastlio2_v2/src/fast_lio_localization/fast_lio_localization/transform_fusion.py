#!/usr/bin/env python3
"""
transform_fusion.py — TF 变换融合节点（简化版：camera_init 为中心）

功能：
  将 FAST-LIO2 里程计与 ICP 全局定位融合，使用 camera_init 作为主要参考帧。
  /localization 的位姿直接来自 ICP（无里程计漂移），速度来自 FAST-LIO2。

TF 树：
  map ← ICP ← camera_init ← FAST-LIO2 → base_link → body
                                         └──→ unilidar_lidar
  （已移除冗余的 odom 帧）
"""

import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import rclpy.timer
import tf_transformations
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Header


class TransformFusion(Node):
    """融合 FAST-LIO2 里程计和 ICP 定位，发布 /localization 和 TF"""

    def __init__(self):
        super().__init__("transform_fusion")

        # ============ 状态变量 ============
        self.cur_odom_to_baselink = None  # FAST-LIO2 里程计 (odom→base_link)
        self.cur_map_to_odom = None       # ICP 全局定位 (map→odom)

        # ============ TF 广播器 ============
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # ============ 发布器 ============
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        # ============ 订阅器 ============
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)

        # ============ 定时器 ============
        self.freq_pub_localization = 50
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def _now_stamp(self):
        return self.get_clock().now().to_msg()

    def transform_fusion(self):
        """
        使用 ICP 纯定位（无里程计）发布 /localization 和 TF 树。

        ICP 位姿（map→camera_init）来自 /map_to_odom，直接用作位姿。
        速度始终为零（里程计已禁用），Nav2 仅靠 ICP 位置更新驱动。
        """
        # 获取 ICP 定位结果 map→camera_init（来自 /map_to_odom）
        if self.cur_map_to_odom is not None:
            T_map_to_camerainit = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_camerainit = np.eye(4)

        # ── 广播 map→camera_init TF ──
        xyz = tf_transformations.translation_from_matrix(T_map_to_camerainit)
        quat = tf_transformations.quaternion_from_matrix(T_map_to_camerainit)

        transform_msg = Transform()
        transform_msg.translation.x = xyz[0]
        transform_msg.translation.y = xyz[1]
        transform_msg.translation.z = xyz[2]
        transform_msg.rotation.x = quat[0]
        transform_msg.rotation.y = quat[1]
        transform_msg.rotation.z = quat[2]
        transform_msg.rotation.w = quat[3]

        header = Header()
        header.stamp = self._now_stamp()
        header.frame_id = "map"

        transform_stamped_msg = tf2_ros.TransformStamped(
            header=header,
            child_frame_id="camera_init",
            transform=transform_msg
        )
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        # ── 发布 /localization（位姿来自 ICP，速度=0） ──
        localization = Odometry()
        localization.pose.pose = Pose(
            position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )
        # 速度始终为零（里程计禁用，ICP 2Hz 更新不产生平滑速度）
        localization.twist.twist.linear.x = 0.0
        localization.twist.twist.linear.y = 0.0
        localization.twist.twist.linear.z = 0.0
        localization.twist.twist.angular.x = 0.0
        localization.twist.twist.angular.y = 0.0
        localization.twist.twist.angular.z = 0.0

        localization.header.stamp = self.get_clock().now().to_msg()
        localization.header.frame_id = "map"
        localization.child_frame_id = "camera_init"
        self.pub_localization.publish(localization)

        # ── 广播 map→base_link TF（让 Nav2 直接从 TF 拿到 ICP 位置，规避 FAST-LIO2 漂移） ──
        tf_map_to_baselink = tf2_ros.TransformStamped(
            header=Header(stamp=self._now_stamp(), frame_id="map"),
            child_frame_id="base_link",
            transform=Transform(
                translation=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
            ),
        )
        self.tf_broadcaster.sendTransform(tf_map_to_baselink)


    def cb_save_cur_odom(self, msg):
        """保存 FAST-LIO2 里程计"""
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        """保存 ICP 定位结果 (map→camera_init)"""
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
