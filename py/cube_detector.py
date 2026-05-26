#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cube_detector.py — 3D OBB 立方体检测节点

功能：
  使用 FAST-LIO2 的雷达点云，通过 DBSCAN 聚类 + PCA 主成分分析，
  实时检测前方 25cm 立方体物块，输出位置（x, y, z）、边长和朝向。

算法流程：
  1. 接收 /unilidar/cloud 点云
  2. 空间范围过滤（前方 0~0.8m，左右 ±0.8m，高度 0~0.6m）
  3. 去地面（去除 z 轴最低 5% 分位以下的点）
  4. 半径滤波去除离群点
  5. 体素下采样（8mm）
  6. DBSCAN 聚类（eps=4cm, min_samples=20）
  7. 每个聚类执行 3D PCA → 得到 OBB 包围盒三个主轴
  8. 边长校验：均值在 25cm ± 5cm 范围内接受
  9. 发布 /detected_cube (Marker) 供 catch.py 使用

坐标系：unilidar_lidar（雷达坐标系，x=前进方向）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import BallTree
import math

class CubeDetector(Node):
    """3D OBB 立方体检测器：从雷达点云中检测 25cm 立方体物块"""

    def __init__(self):
        super().__init__('cube_detector')

        # 订阅雷达点云（FAST-LIO2 配准后的 /unilidar/cloud）
        self.subscription = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.cloud_callback, 10)

        # 发布检测到的立方体 Marker（供 catch.py 机械臂控制节点订阅）
        self.marker_pub = self.create_publisher(Marker, '/detected_cube', 10)

        # ============ 点云预处理参数 ============
        self.x_range = (-0.2, 0.8)    # X 范围（雷达前方 0~80cm）
        self.y_range = (-0.8, 0.8)    # Y 范围（左右 ±80cm）
        self.z_range = (0, 0.6)       # Z 范围（高度 0~60cm）
        self.voxel_size = 0.008       # 体素下采样大小（8mm）

        # ============ DBSCAN 聚类参数 ============
        self.cluster_eps = 0.04       # 聚类邻域半径（4cm）
        self.cluster_min_samples = 20 # 最小聚类点数

        # ============ 立方体边长校验 ============
        self.edge_target = 0.25       # 目标边长 25cm
        self.edge_tol = 0.05          # 容差 ±5cm

        # ============ 半径滤波参数 ============
        self.radius = 0.04
        self.min_neighbors = 5

        self.get_logger().info('=' * 60)
        self.get_logger().info('3D OBB 立方体检测已启动')
        self.get_logger().info('算法: 3D PCA → OBB包围盒 → 均值边长')
        self.get_logger().info('=' * 60)

    def pc2_to_np(self, msg):
        """ROS PointCloud2 → numpy (N×3) 数组"""
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.array(list(gen))
        if arr.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32)

    def remove_statistical_outliers(self, pts, radius=0.03, min_neighbors=3):
        """半径统计滤波：移除周围邻域点过少的离群点"""
        if len(pts) < min_neighbors:
            return pts
        tree = BallTree(pts, metric='euclidean')
        counts = tree.query_radius(pts, r=radius, count_only=True)
        return pts[counts >= min_neighbors]

    def cloud_callback(self, msg):
        """
        点云回调：预处理 → 聚类 → OBB 分析 → 发布最佳立方体

        步骤：
        1. 空间裁剪（保留前方感兴趣区域）
        2. 去地面（移除 z 轴最低 5% 以下的点）
        3. 半径滤波 + 体素下采样
        4. DBSCAN 聚类
        5. 对每个聚类做 OBB 分析
        6. 选择边长最接近 25cm 的立方体发布 Marker
        """
        try:
            pts_raw = self.pc2_to_np(msg)
            if pts_raw.shape[0] < 100:
                return

            mask = (pts_raw[:, 0] > self.x_range[0]) & (pts_raw[:, 0] < self.x_range[1]) & \
                   (pts_raw[:, 1] > self.y_range[0]) & (pts_raw[:, 1] < self.y_range[1]) & \
                   (pts_raw[:, 2] > self.z_range[0]) & (pts_raw[:, 2] < self.z_range[1])
            pts = pts_raw[mask]

            # 去地面：只保留 z > 最低5%分位 + 2cm 的点
            z_min = np.percentile(pts[:, 2], 5)
            pts = pts[pts[:, 2] > z_min + 0.02]

            # 半径滤波
            pts = self.remove_statistical_outliers(pts, self.radius, self.min_neighbors)

            # 体素下采样
            pts = self.voxel_downsample(pts, self.voxel_size)
            if pts.shape[0] < self.cluster_min_samples:
                return

            db = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(pts)
            labels = db.labels_

            cubes = []
            for i in range(labels.max() + 1):
                cluster_pts = pts[labels == i]
                if cluster_pts.shape[0] < 20:
                    continue

                cube_info = self.analyze_cluster(cluster_pts)

                if cube_info:
                    cubes.append(cube_info)

            if cubes:
                best = min(cubes, key=lambda x: abs(x['edge'] - self.edge_target))

                marker = Marker()
                marker.header.frame_id = "unilidar_lidar"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "detected_cube"
                marker.id = 0
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.pose.position.x = float(best['x'])
                marker.pose.position.y = float(best['y'])
                marker.pose.position.z = float(best['z'])

                marker.scale.x = float(best['edge'])
                marker.scale.y = float(best['edge'])
                marker.scale.z = float(best['edge'])

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.6

                q = Quaternion()
                q.x = 0.0
                q.y = 0.0
                q.z = float(math.sin(best['yaw'] / 2.0))
                q.w = float(math.cos(best['yaw'] / 2.0))
                marker.pose.orientation = q

                marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

                self.marker_pub.publish(marker)

                self.get_logger().info(
                    f"立方体 x={best['x']:.3f} y={best['y']:.3f} z={best['z']:.3f} "
                    f"边长={best['edge']:.3f}m"
                )

        except Exception as e:
            self.get_logger().error(f'点云处理异常: {str(e)}')

    def analyze_cluster(self, pts):
        """
        3D OBB 包围盒法：
        1. 3D PCA → 三个主轴 = 立方体的三条边方向
        2. 旋转到局部坐标系 → 三个方向的范围 = 高/宽/厚
        3. 边长取中位数（自动滤掉"薄片"方向）
        4. 从局部中心反算全局中心
        """
        center = np.mean(pts, axis=0)
        centered = pts - center
        n = len(pts)

        # 3D PCA
        cov = (centered.T @ centered) / n
        evals, evecs = np.linalg.eigh(cov)
        # evals[0] ≤ evals[1] ≤ evals[2]
        # evecs[:,0]=最小特征值方向(法线), evecs[:,2]=最大特征值方向

        # 旋转到局部坐标系
        R = evecs.T  # 3x3 旋转矩阵，每行是一个主轴
        pts_local = centered @ R.T

        # 三个方向的范围（5%~95% 截断排除离群点）
        dims = []
        for i in range(3):
            low = np.percentile(pts_local[:, i], 5)
            high = np.percentile(pts_local[:, i], 95)
            dims.append(high - low)

        # dims = [厚度, 宽度, 高度]（排序后）
        d_sorted = sorted(dims)

        # 边长取三个边平均值
        edge = np.mean(dims)

        # 边长效验（放宽）
        if not (self.edge_target - self.edge_tol < edge < self.edge_target + self.edge_tol):
            return None

        # OBB 中心（局部坐标范围中心）
        local_center = np.array([
            (np.min(pts_local[:, 0]) + np.max(pts_local[:, 0])) / 2,
            (np.min(pts_local[:, 1]) + np.max(pts_local[:, 1])) / 2,
            (np.min(pts_local[:, 2]) + np.max(pts_local[:, 2])) / 2,
        ])
        # 转换回全局：先转置(evecs是正交矩阵，逆=转置)
        global_center = center + local_center @ evecs.T

        # Yaw = 最长主轴在 XY 平面的投影角度
        main_axis = evecs[:, 2]  # 最大特征值方向
        yaw = np.arctan2(main_axis[1], main_axis[0])

        return {
            'x': float(global_center[0]),
            'y': float(global_center[1]),
            'z': float(global_center[2]),
            'edge': edge,
            'yaw': yaw,
            'mode': '3d_obb'
        }

    def voxel_downsample(self, pts, size):
        """
        体素重心下采样（比体素中心下采样更准）

        将空间划分为 size×size×size 的体素网格，
        每个体素内取所有点的重心代替体素中心，
        保留更准确的几何信息。
        """
        if pts.shape[0] == 0:
            return pts
        idx = np.floor(pts / size).astype(np.int32)
        keys, inverse, counts = np.unique(idx, axis=0, return_inverse=True, return_counts=True)
        summed = np.zeros((len(keys), 3), dtype=np.float64)
        np.add.at(summed, inverse, pts.astype(np.float64))
        centroids = summed / counts[:, np.newaxis]
        return centroids.astype(np.float32)


def main():
    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
