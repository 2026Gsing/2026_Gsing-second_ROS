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

        # 订阅单帧点云，内部累积多帧后再检测
        self.subscription = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.cloud_callback, 10)

        # 发布检测到的立方体 Marker（供 catch.py 机械臂控制节点订阅）
        self.marker_pub = self.create_publisher(Marker, '/detected_cube', 10)

        # ============ 点云预处理参数 ============
        self.x_range = (0, 0.35)    # X 范围（雷达前方 0~80cm）
        self.y_range = (-0.5, 0.5)    # Y 范围（左右 ±80cm）
        self.z_range = (0, 0.4)       # Z 范围（高度 0~60cm）
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

        # ============ 多帧累积参数 ============
        self.frame_buffer = []        # 点云帧缓存（累积多帧再检测）
        self.max_frames = 10          # 累积 10 帧（约 1 秒）后检测一次

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
            if pts_raw.shape[0] < 50:
                return

            # 多帧累积：攒够 max_frames 帧再检测
            self.frame_buffer.append(pts_raw)
            if len(self.frame_buffer) < self.max_frames:
                return
            pts_raw = np.vstack(self.frame_buffer)
            self.frame_buffer = []
            self.get_logger().info(f"[累积] {self.max_frames}帧合并为 {pts_raw.shape[0]} 点")

            mask = (pts_raw[:, 0] > self.x_range[0]) & (pts_raw[:, 0] < self.x_range[1]) & \
                   (pts_raw[:, 1] > self.y_range[0]) & (pts_raw[:, 1] < self.y_range[1]) & \
                   (pts_raw[:, 2] > self.z_range[0]) & (pts_raw[:, 2] < self.z_range[1])
            pts = pts_raw[mask]

            # 去地面（雷达倒装，地面在 Z 最大处）：去掉 Z 最高的 3% 分位以上点
            z_ground = np.percentile(pts[:, 2], 97)
            pts = pts[pts[:, 2] < z_ground - 0.02]
            n_after_ground = pts.shape[0]

            # 半径滤波
            pts = self.remove_statistical_outliers(pts, self.radius, self.min_neighbors)

            # 体素下采样
            pts = self.voxel_downsample(pts, self.voxel_size)
            if pts.shape[0] < self.cluster_min_samples:
                return

            db = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(pts)
            labels = db.labels_

            # 各阶段点数量调试
            n_clusters = labels.max() + 1
            n_noise = (labels == -1).sum()
            self.get_logger().info(
                f"[DBG] 原始{pts_raw.shape[0]}→ROI{pts_raw[mask].shape[0]}→"
                f"去地面{n_after_ground}→滤波{pts.shape[0]}点 → "
                f"{n_clusters}聚类({n_noise}噪点)"
            )

            cubes = []
            for i in range(labels.max() + 1):
                cluster_pts = pts[labels == i]
                if cluster_pts.shape[0] < 20:
                    continue

                cube_info = self.analyze_cluster(cluster_pts)

                if cube_info and abs(cube_info['y']) < 0.40:
                    cubes.append(cube_info)

            if cubes:
                # ========== 输出所有检测到的立方体 ==========
                self.get_logger().info(f"检测到 {len(cubes)} 个立方体:")
                for i, c in enumerate(cubes):
                    xy_dist = math.hypot(c['x'], c['y'])
                    self.get_logger().info(
                        f"  [{i}] pos=({c['x']:.3f},{c['y']:.3f},{c['z']:.3f}) "
                        f"edge={c['edge']:.3f} mode={c['face_mode']} "
                        f"dims=[{c['_dims0']:.3f},{c['_dims1']:.3f},{c['_dims2']:.3f}] "
                        f"ratio={c['_ratio']:.2f} xy_dist={xy_dist:.3f}"
                    )

                # ========== 选择最接近25cm的立方体发布 ==========
                # 优先选边长最接近25cm的，同等条件下选离中线最近（Y绝对值最小）的
                best = min(cubes, key=lambda x: abs(x['edge'] - self.edge_target) + abs(x['y']) * 0.05)

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

                marker.lifetime = rclpy.duration.Duration(seconds=1.5).to_msg()

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
        自适应 3D OBB 包围盒法（支持单面/三面可见）

        1. 3D PCA → 三个主轴 = 立方体的三条边方向
        2. 旋转到局部坐标系 → 三个方向的范围 = 厚/宽/高
        3. 根据法线方向厚度 dims[0] 判断可见面数：
           - dims[0] < 0.15（单面）：沿法线向里推半边长
           - dims[0] ≥ 0.15（三面）：PCA 中心已是几何中心，不推
        4. 边长取三轴均值，长宽比 < 1.8 校验
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

        # dims = [法线方向厚度, 宽度, 高度]
        thickness = dims[0]  # 法线方向（雷达视线方向）的厚度
        ratio = max(dims) / max(min(dims), 1e-6)

        # 边长取三个边平均值
        edge = np.mean(dims)

        # 长宽比校验：立方体真实比 = 1.0，允许被遮挡截断到 2.2
        if ratio > 3.0:
            self.get_logger().info(
                f"  [淘汰] 聚类中心({center[0]:.3f},{center[1]:.3f},{center[2]:.3f}) "
                f"长宽比 {ratio:.2f} > 1.8 dims=[{dims[0]:.3f},{dims[1]:.3f},{dims[2]:.3f}]"
            )
            return None

        # 边长效验（放宽）
        if not (self.edge_target - self.edge_tol < edge < self.edge_target + self.edge_tol):
            self.get_logger().info(
                f"  [淘汰] 边长 {edge:.3f}m 不在 [{self.edge_target-self.edge_tol}, "
                f"{self.edge_target+self.edge_tol}] 范围 dims=[{dims[0]:.3f},{dims[1]:.3f},{dims[2]:.3f}]"
            )
            return None

        # ========== 立方体中心校正 ==========
        # 根据可见面数选择不同的中心估算策略
        if thickness < 0.10:
            # ---- 单面可见 ----
            # LiDAR 只能扫到前表面，PCA 中心偏前。
            # 需要沿法线向里推半个边长得到真实中心。
            face_mode = "单面"
            normal = evecs[:, 0]  # 法线方向（最小特征值）
            if np.dot(normal, center) < 0:
                normal = -normal
            half_edge = edge / 2.0
            global_center = center + normal * half_edge
        else:
            # ---- 三面可见（侧面视角） ----
            # PCA 中心已接近几何中心，无需沿法线推入
            face_mode = "三面"
            global_center = center

        # Yaw = 最长主轴在 XY 平面的投影角度
        main_axis = evecs[:, 2]  # 最大特征值方向
        yaw = np.arctan2(main_axis[1], main_axis[0])

        return {
            'x': float(global_center[0]),
            'y': float(global_center[1]),
            'z': float(global_center[2]),
            'edge': edge,
            'yaw': yaw,
            'mode': '3d_obb',
            # 调试字段
            'face_mode': face_mode,
            '_dims0': round(dims[0], 3),
            '_dims1': round(dims[1], 3),
            '_dims2': round(dims[2], 3),
            '_ratio': round(ratio, 2),
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
