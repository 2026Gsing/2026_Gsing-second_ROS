#!/usr/bin/env python3
"""
global_localization.py — FAST-LIO2 全局重定位节点

功能：
  将 FAST-LIO2 实时扫描点云与预存的全局 PCD 地图进行 ICP 配准，
  计算 map→odom 变换矩阵，为 Nav2 提供全局定位信息。

工作流程：
  1. 启动时加载预存的全局 PCD 地图（体素下采样）
  2. 订阅 FAST-LIO2 的 /Odometry（里程计）和 /cloud_registered（配准点云）
  3. 订阅 /initialpose 进行初始化（来自 RViz "2D Pose Estimate" 或 publish_initial_pose.py）
  4. 配准成功后发布 /map_to_odom (Odometry) → transform_fusion.py 通过 TF 广播给 Nav2
  5. 以固定频率持续运行 ICP 配准，保持定位精度

与 Nav2 的交互：
  本节点 → /map_to_odom → transform_fusion.py → TF (map→camera_init/odom) → Nav2
"""

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import tf_transformations
import ros2_numpy


class FastLIOLocalization(Node):
    """FAST-LIO2 全局重定位：将实时激光扫描与预存地图 ICP 配准，计算 map→odom 变换"""

    def __init__(self):
        super().__init__("fast_lio_localization")

        # ============ 状态变量 ============
        self.global_map = None          # 预存的全局 PCD 地图（Open3D 点云）
        self.T_map_to_odom = np.eye(4)  # map→odom 变换矩阵（ICP 配准结果）
        self.cur_odom = None            # 最新的 FAST-LIO2 里程计
        self.cur_scan = None            # 最新的 FAST-LIO2 扫描点云
        self.initialized = False        # 是否已收到初始位姿

        # ============ ROS2 参数 ============
        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),           # 地图下采样体素大小 (m)
                ("scan_voxel_size", 0.1),           # 扫描下采样体素大小 (m)
                ("freq_localization", 0.5),         # 定位频率 (Hz)
                ("freq_global_map", 0.25),          # 地图发布频率 (Hz)
                ("localization_threshold", 0.8),    # ICP 配准 fitness 阈值
                ("fov", 6.28319),                   # 视场角 (弧度)，全向=2π
                ("fov_far", 300),                   # FOV 最远距离 (m)
                ("pcd_map_topic", "/map"),          # 地图话题名
                ("pcd_map_path", ""),               # PCD 地图文件路径（优先级高于话题）
            ],
        )

        # ============ TF 监听 ============
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ============ 发布器 ============
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        # ============ 加载全局地图 ============
        self.get_logger().info("Waiting for global map...")
        self.initialize_global_map()
        self.get_logger().info("Global map received.")

        # ============ 订阅器 ============
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        # ============ 定时器 ============
        self.timer_localisation = self.create_timer(
            1.0 / self.get_parameter("freq_localization").value,
            self.localisation_timer_callback
        )

    def global_map_callback(self):
        """（未启用）将全局地图发布到话题"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))

    def pose_to_mat(self, pose):
        """将 ROS Pose 消息转为 4×4 齐次变换矩阵"""
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def msg_to_array(self, pc_msg):
        """将 ROS PointCloud2 消息转为 numpy 点云数组"""
        pc_array = ros2_numpy.numpify(pc_msg)
        return pc_array["xyz"]

    def registration_at_scale(self, scan, map, initial, scale):
        """
        在指定尺度下执行 ICP 配准

        多尺度配准策略：
        - 大尺度 (scale=5)：粗配准，用大体素快速收敛到大体位置
        - 小尺度 (scale=1)：精配准，用小体素精确微调位姿
        """
        result_icp = o3d.pipelines.registration.registration_icp(
            self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
            self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
            1.0 * scale,
            initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness

    def inverse_se3(self, trans):
        """计算 SE3 变换的逆变换：
        [R t; 0 1]^{-1} = [R^T  -R^T*t; 0 1]
        """
        trans_inverse = np.eye(4)
        trans_inverse[:3, :3] = trans[:3, :3].T          # R^{-1} = R^T
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])  # t' = -R^T * t
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc):
        """将 numpy 点云发布为 ROS PointCloud2 消息"""
        data = dict()
        data["xyz"] = pc[:, :3]

        if pc.shape[1] == 4:
            data["intensity"] = pc[:, 3]
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        if len(msg.fields) == 4:
            msg.point_step = 16
        else:
            msg.point_step = 12

        publisher.publish(msg)

    def crop_global_map_in_FOV(self, pose_estimation):
        """
        根据当前位姿估计，从全局地图中裁剪出视野范围内的局部子地图

        步骤：
        1. 将全局地图点云变换到 base_link 坐标系
        2. 根据 FOV 角度和距离过滤（只保留传感器视野内的点）
        3. 发布子地图用于可视化
        4. 返回裁剪后的 Open3D 点云用于 ICP 配准
        """
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # 将全局地图变换到 base_link 系
        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        # 根据 FOV 过滤
        if self.get_parameter("fov").value > 3.14:
            # 全向模式：只做距离过滤
            indices = np.where(
                global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value
            )
        else:
            # 有限 FOV：做距离 + 角度过滤
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )

        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        # 发布子地图（降采样到 1/10 用于 RViz 可视化）
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        """
        执行全局定位（多尺度 ICP 配准）

        策略：
        1. 粗配准 (scale=5)：大尺度体素下采样，快速收敛
        2. 精配准 (scale=1)：小尺度体素下采样，精确微调
        3. 如果 fitness > 阈值则更新 map→odom 变换
        """
        scan_tobe_mapped = copy.copy(self.cur_scan)
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)

        # 粗配准
        transformation, _ = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5
        )

        # 精配准
        transformation, fitness = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=1
        )

        if fitness > self.get_parameter("localization_threshold").value:
            self.T_map_to_odom = transformation
            self.publish_odom(transformation)
        else:
            self.get_logger().warn(
                f"Fitness score {fitness} less than localization threshold "
                f"{self.get_parameter('localization_threshold').value}"
            )

    def voxel_down_sample(self, pcd, voxel_size):
        """体素下采样（兼容 Open3D 不同版本 API）"""
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        except Exception:
            # Open3D <=0.7 的低版本兼容
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        """保存最新的 FAST-LIO2 里程计"""
        self.cur_odom = msg

    def cb_save_cur_scan(self, msg):
        """保存最新的 FAST-LIO2 配准点云并发布到 map 坐标系下可视化"""
        pc = self.msg_to_array(msg)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)
        # 使用当前时间戳发布点云，避免 RViz TF 滤波器因传感器时间戳滞后而丢弃消息
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id
        self.publish_point_cloud(self.pub_pc_in_map, header, pc)

    def initialize_global_map(self):
        """加载 PCD 全局地图并进行体素下采样"""
        self.global_map = o3d.io.read_point_cloud(self.get_parameter("pcd_map_path").value)
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        self.get_logger().info("Global map received.")

    def cb_initialize_pose(self, msg):
        """
        收到初始位姿（来自 RViz "2D Pose Estimate" 或 publish_initial_pose.py）
        后，执行首次全局定位
        """
        initial_pose = self.pose_to_mat(msg.pose.pose)
        self.initialized = True
        self.get_logger().info("Initial pose received.")

        if self.cur_scan is not None:
            self.global_localization(initial_pose)

    def publish_odom(self, transform):
        """将 map→odom 变换矩阵发布为 Odometry 消息"""
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(transform)
        odom_msg.pose.pose = Pose(
            position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self):
        """定时定位任务：持续执行 ICP 配准以维持定位精度"""
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...")
            return

        if self.cur_scan is not None:
            self.global_localization(self.T_map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()