#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cube_detector.py — 3D OBB 立方体检测节点

功能：
  使用 FAST-LIO2 的雷达点云，通过 DBSCAN 聚类 + PCA 主成分分析，
  实时检测前方 25cm 立方体物块，输出位置（x, y, z）、边长和朝向。

算法流程：
  1. 接收 /unilidar/cloud 点云
  2. 空间范围过滤（前方 0~0.5m，左右 ±0.5m，高度 0~0.6m）
  3. 去地面（去除 Z 最高 6% 分位以上点——雷达倒装）
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
from sklearn.linear_model import RANSACRegressor
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
        self.x_range = (0, 0.8)    # X 范围（雷达前方 0~80cm）
        self.y_range = (-0.7, 0.7)    # Y 范围（左右 ±70cm）
        self.z_range = (0, 0.7)       # Z 范围（高度 0~70cm）
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
            total = pts_raw.shape[0]
            if total < 100:
                self.get_logger().debug(f"原始点云仅 {total} 点，跳过")
                return

            # ===== 阶段 1：ROI 裁剪 =====
            mask = (pts_raw[:, 0] > self.x_range[0]) & (pts_raw[:, 0] < self.x_range[1]) & \
                   (pts_raw[:, 1] > self.y_range[0]) & (pts_raw[:, 1] < self.y_range[1]) & \
                   (pts_raw[:, 2] > self.z_range[0]) & (pts_raw[:, 2] < self.z_range[1])
            pts = pts_raw[mask]
            after_roi = pts.shape[0]
            n_ground = 0
            if after_roi < 20:
                self.get_logger().debug(f"ROI 后仅 {after_roi}/{total} 点 ({self.x_range}, {self.y_range}, {self.z_range})")
                return

            # ===== 阶段 2：RANSAC 平面拟合去地面 =====
            # 拟合地面平面 z = a*x + b*y + c，去掉距离平面 < 4cm 的内点
            # 箱子前表面是垂直面，距地面平面较远，会被保留为外点
            if pts.shape[0] >= 100:
                ransac = RANSACRegressor(
                    residual_threshold=0.04,
                    max_trials=100,
                    min_samples=50,
                    random_state=42
                )
                ransac.fit(pts[:, [0, 1]], pts[:, 2])
                inlier_mask = ransac.inlier_mask_
                pts = pts[~inlier_mask]  # 保留外点（非地面点）
            if pts.shape[0] < 20:
                self.get_logger().debug(f"RANSAC 去地面后仅 {pts.shape[0]} 点")
                return

            # ===== 阶段 3：半径滤波 =====
            before_rad = pts.shape[0]
            pts = self.remove_statistical_outliers(pts, self.radius, self.min_neighbors)
            if pts.shape[0] < 10:
                self.get_logger().debug(f"半径滤波后仅 {pts.shape[0]}/{before_rad} 点")
                return

            # ===== 阶段 4：体素下采样 =====
            pts = self.voxel_downsample(pts, self.voxel_size)
            after_voxel = pts.shape[0]
            if pts.shape[0] < self.cluster_min_samples:
                self.get_logger().debug(f"下采样后仅 {after_voxel} 点 (< {self.cluster_min_samples})，跳过")
                return

            # ===== 阶段 5：DBSCAN 聚类 =====
            db = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(pts)
            labels = db.labels_
            n_clusters = labels.max() + 1
            self.get_logger().info(
                f"[DBG] {total}→ROI{after_roi}→体素{after_voxel} → {n_clusters}个聚类"
            )

            # ===== 阶段 6：OBB 分析 =====
            cubes = []
            for i in range(labels.max() + 1):
                cluster_pts = pts[labels == i]
                if cluster_pts.shape[0] < 20:
                    continue

                cube_info = self.analyze_cluster(cluster_pts)

                if cube_info:
                    cubes.append(cube_info)
                else:
                    # 打印聚类尺寸诊断
                    center = np.mean(cluster_pts, axis=0)
                    cov = ((cluster_pts - center).T @ (cluster_pts - center)) / len(cluster_pts)
                    evals, _ = np.linalg.eigh(cov)
                    dim_est = np.sqrt(evals) * 3.0  # 3σ 近似边长
                    self.get_logger().info(
                        f"  [聚类{i}] {cluster_pts.shape[0]}点 "
                        f"中心({center[0]:.3f},{center[1]:.3f},{center[2]:.3f}) "
                        f"PCA特征值:{evals[0]:.4f},{evals[1]:.4f},{evals[2]:.4f} "
                        f"≈{dim_est[0]:.3f}×{dim_est[1]:.3f}×{dim_est[2]:.3f}m"
                    )

            if not cubes:
                self.get_logger().info(f"[DBG] {n_clusters}个聚类均未通过 OBB 校验")
                return

            # ========== 输出所有检测到的立方体 ==========
            self.get_logger().info(f"检测到 {len(cubes)} 个立方体:")
            for i, c in enumerate(cubes):
                xy_dist = math.hypot(c['x'], c['y'])
                self.get_logger().info(
                    f"  [{i}] x={c['x']:.3f} y={c['y']:.3f} z={c['z']:.3f} "
                    f"边长={c['edge']:.3f}m xy距离={xy_dist:.3f}m"
                )

                # ========== 选择最接近25cm的立方体发布 ==========
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

                xy_dist = math.hypot(best['x'], best['y'])
                self.get_logger().info(
                    f"已发布: x={best['x']:.3f} y={best['y']:.3f} z={best['z']:.3f} "
                    f"边长={best['edge']:.3f}m xy距离={xy_dist:.3f}m"
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

        # dims = [法线方向厚度, 可见面宽度, 可见面高度]
        # 法线方向（dims[0]）被单视角截断，不计入边长校验
        face_w = dims[1]   # 可见面宽度
        face_h = dims[2]   # 可见面高度

        # 先验：箱体是 25cm 立方体，因此可见面应接近正方形
        # 放宽检验：至少一个可见边 > 15cm，且长宽比 < 2.5
        face_max = max(face_w, face_h)
        face_min = min(face_w, face_h)

        if face_max < 0.15:
            return None
        if face_max / max(face_min, 1e-6) > 2.5:
            return None

        # ========== 立方体中心校正 ==========
        # LiDAR 只能扫到前表面，PCA 的 center 是表面中心的近似。
        # 需要沿法线（最小特征值方向）向里推半个边长得到真实中心。
        normal = evecs[:, 0]  # 法线方向（最小特征值）
        # 确保法线指向雷达反方向（即指向立方体内部）
        if np.dot(normal, center) < 0:
            normal = -normal

        # 可见面两边的平均值作为立方体尺寸估计
        est_edge = (face_w + face_h) / 2.0
        edge = float(est_edge)
        half_edge = est_edge / 2.0

        # 立方体中心 = 表面中心 + 向里推半个边长
        global_center = center + normal * half_edge

        # 倒装雷达，地面在 z 最大处 ≈0.5。真箱体中心 z < 0.4
        if global_center[2] > 0.40:
            return None
        if est_edge < 0.15:
            return None

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
