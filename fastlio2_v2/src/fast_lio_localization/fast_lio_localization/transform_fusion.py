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
        使用 ICP 位姿（纯定位）发布 /localization 和 TF 树

        ICP 位姿（map→camera_init）来自 /map_to_odom，直接用作精准定位，
        不乘以 FAST-LIO2 的 odom→base_link 以避免里程计漂移累积。
        FAST-LIO2 的速度（twist）保留用于平滑控制。
        """
        if self.cur_odom_to_baselink is None:
            # 里程计未就绪：发布恒等 TF 保持 TF 树存活
            transform_msg = Transform()
            transform_msg.translation.x = 0.0
            transform_msg.translation.y = 0.0
            transform_msg.translation.z = 0.0
            transform_msg.rotation.x = 0.0
            transform_msg.rotation.y = 0.0
            transform_msg.rotation.z = 0.0
            transform_msg.rotation.w = 1.0

            header = Header()
            header.stamp = self._now_stamp()
            header.frame_id = "map"

            transform_stamped_msg = tf2_ros.TransformStamped(
                header=header,
                child_frame_id="camera_init",
                transform=transform_msg
            )
            self.tf_broadcaster.sendTransform(transform_stamped_msg)
            return

        # 获取 ICP 定位结果 map→camera_init（来自 /map_to_odom）
        # 语义上 map_to_odom 即 map→camera_init（odom 已废弃，由 camera_init 取代）
        if self.cur_map_to_odom is not None:
            T_map_to_camerainit = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_camerainit = np.eye(4)

        # ── 广播 map→camera_init TF ──
        transform_msg = Transform()
        transform_msg.translation.x = T_map_to_camerainit[0, 3]
        transform_msg.translation.y = T_map_to_camerainit[1, 3]
        transform_msg.translation.z = T_map_to_camerainit[2, 3]
        quat = tf_transformations.quaternion_from_matrix(T_map_to_camerainit)
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

        # camera_init→base_link = identity（base_link 不单独偏移动，位置全靠 ICP）
        # map→base_link 同理 = map→camera_init（不叠加快速里程计偏移）
        # 两个均通过 launch 文件的静态恒等 TF 提供，这里不做动态发布
        # 速度（twist）在 /localization 中保留，用于平滑控制

        # ── 广播 odom→map 直接变换（静态，用于 controller 坐标转换） ──
        # camera_init→odom 为静态恒等（launch 中定义），odom→map 提供 camera_init→map 等效路径
        if self.cur_odom_to_baselink is not None:
            T_camerainit_to_map = np.linalg.inv(T_map_to_camerainit)
            xyz = tf_transformations.translation_from_matrix(T_camerainit_to_map)
            quat = tf_transformations.quaternion_from_matrix(T_camerainit_to_map)
            odom_map_tf = Transform()
            odom_map_tf.translation.x = xyz[0]
            odom_map_tf.translation.y = xyz[1]
            odom_map_tf.translation.z = xyz[2]
            odom_map_tf.rotation.x = quat[0]
            odom_map_tf.rotation.y = quat[1]
            odom_map_tf.rotation.z = quat[2]
            odom_map_tf.rotation.w = quat[3]
            # 通过 /tf_static 发布（Time(0)），避免 controller 外推失败
            odom_map_static_header = Header()
            odom_map_static_header.stamp = rclpy.time.Time().to_msg()
            odom_map_static_header.frame_id = "odom"
            odom_map_static = tf2_ros.TransformStamped(
                header=odom_map_static_header,
                child_frame_id="map",
                transform=odom_map_tf
            )
            self.tf_static_broadcaster.sendTransform(odom_map_static)

        # ── 发布 /localization（位姿来自纯 ICP，速度来自 FAST-LIO2） ──
        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            # 位姿 = 纯 ICP（map→camera_init），无里程计漂移
            xyz = tf_transformations.translation_from_matrix(T_map_to_camerainit)
            quat = tf_transformations.quaternion_from_matrix(T_map_to_camerainit)

            localization = Odometry()
            localization.pose.pose = Pose(
                position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            )
            # 速度 = FAST-LIO2 里程计（高频平滑）
            localization.twist = cur_odom.twist

            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = "map"
            localization.child_frame_id = "camera_init"
            self.pub_localization.publish(localization)


    def cb_save_cur_odom(self, msg):
        """保存 FAST-LIO2 里程计"""
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        """保存 ICP 定位结果 (map→camera_init)"""
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
