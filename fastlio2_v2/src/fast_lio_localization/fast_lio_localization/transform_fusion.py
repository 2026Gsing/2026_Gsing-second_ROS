#!/usr/bin/env python3
"""
transform_fusion.py — TF 变换融合节点

功能：
  将 FAST-LIO2 里程计 (odom→base_link) 与全局定位 (map→odom) 融合，
  发布完整的 map→camera_init（即 map→odom）TF 变换和
  map 坐标系下的定位话题 /localization。

工作流程：
  1. 订阅 FAST-LIO2 的 /Odometry (odom→base_link)
  2. 订阅 global_localization 的 /map_to_odom (map→odom)
  3. 融合两个变换：T_map_to_base_link = T_map_to_odom * T_odom_to_base_link
  4. 广播 map→camera_init TF 变换
  5. 发布 /localization (Odometry) 话题供 Nav2 使用

TF 树关系：
  map ← map_to_odom ← camera_init
                    ↓ (static)
                  odom ← FAST-LIO2 → base_link
  Nav2 使用：map → odom → base_link（通过 static camera_init→odom 桥接）
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
    """融合 FAST-LIO2 里程计和全局定位，发布 map→base_link 变换"""

    def __init__(self):
        super().__init__("transform_fusion")

        # ============ 状态变量 ============
        self.cur_odom_to_baselink = None  # FAST-LIO2 里程计 (odom→base_link)
        self.cur_map_to_odom = None       # 全局定位 (map→odom)

        # ============ TF 广播器 ============
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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

    def transform_fusion(self):
        """
        融合 map→odom 和 odom→base_link，发布 TF 和 /localization

        情况 1: 里程计未就绪 → 发布恒等 TF (map→camera_init) 保持 TF 树存活
        情况 2: 里程计就绪 + 全局定位就绪 → 融合并发布完整变换
        情况 3: 里程计就绪但全局定位未就绪 → 仅使用恒等 map→odom (纯里程计模式)
        """
        if self.cur_odom_to_baselink is None:
            # 里程计未就绪：发布恒等 TF 保持 TF 树存活，
            # 这样 RViz 的 fixed frame "map" 不会报错
            transform_msg = Transform()
            transform_msg.translation.x = 0.0
            transform_msg.translation.y = 0.0
            transform_msg.translation.z = 0.0
            transform_msg.rotation.x = 0.0
            transform_msg.rotation.y = 0.0
            transform_msg.rotation.z = 0.0
            transform_msg.rotation.w = 1.0

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"

            transform_stamped_msg = tf2_ros.TransformStamped(
                header=header,
                child_frame_id="camera_init",
                transform=transform_msg
            )
            self.tf_broadcaster.sendTransform(transform_stamped_msg)
            return

        # 获取 map→odom 变换（若无全局定位则使用单位矩阵，即纯里程计模式）
        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        # 广播 map→camera_init TF 变换
        transform_msg = Transform()
        transform_msg.translation.x = T_map_to_odom[0, 3]
        transform_msg.translation.y = T_map_to_odom[1, 3]
        transform_msg.translation.z = T_map_to_odom[2, 3]

        quat = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        transform_msg.rotation.x = quat[0]
        transform_msg.rotation.y = quat[1]
        transform_msg.rotation.z = quat[2]
        transform_msg.rotation.w = quat[3]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # 使用当前时间戳发布 map→camera_init，避免 RViz TF 滤波器丢弃
        transform_stamped_msg = tf2_ros.TransformStamped(
            header=header,
            child_frame_id="camera_init",
            transform=transform_msg
        )
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        # 融合 map→base_link = map→odom * odom→base_link
        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

            xyz = tf_transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf_transformations.quaternion_from_matrix(T_map_to_base_link)

            # 发布 /localization（map 坐标系下的完整位姿 + 速度）
            localization = Odometry()
            localization.pose.pose = Pose(
                position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            )
            localization.twist = cur_odom.twist

            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = "map"
            localization.child_frame_id = "body"
            self.pub_localization.publish(localization)


    def cb_save_cur_odom(self, msg):
        """保存 FAST-LIO2 里程计 (odom→base_link)"""
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        """保存全局定位结果 (map→odom)"""
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
