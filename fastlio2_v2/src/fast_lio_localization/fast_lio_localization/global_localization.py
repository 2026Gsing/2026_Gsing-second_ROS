#!/usr/bin/env python3
"""
global_localization.py — FAST-LIO2 全局重定位节点

功能：
  将 FAST-LIO2 实时扫描点云与预存的全局 PCD 地图进行 ICP 配准，
  计算 map→odom 变换矩阵，为 Nav2 提供全局定位信息。

不依赖 open3d / ros2_numpy（这些库在 Bay Trail CPU 上因 AVX 指令崩溃），
改用 numpy + scipy.spatial.KDTree 实现 ICP。

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
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import tf_transformations


# ======================================================================
# open3d 替代工具函数（纯 numpy/scipy，避免 AVX 指令崩溃）
# ======================================================================

_SCIPY_KD = None  # 延后导入（第一次用时才 import）


def _get_kdtree():
    """延后导入 scipy.spatial.KDTree，避免影响 ROS2 节点启动速度"""
    global _SCIPY_KD
    if _SCIPY_KD is None:
        from scipy.spatial import KDTree
        _SCIPY_KD = KDTree
    return _SCIPY_KD


def read_pcd(path):
    """
    读取二进制 PCD 文件，返回 (N, 3) float64 点云数组。
    支持大部分标准字段排列，仅提取 x/y/z。
    """
    with open(path, "rb") as f:
        header = {}
        fields = None
        size = None
        typ = None
        count = None
        points = 0
        data_fmt = None

        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            if line.upper().startswith("DATA"):
                header["DATA"] = line.split(" ", 1)[-1].strip()
                break
            parts = line.split(" ", 1)
            if len(parts) == 2:
                key, val = parts
                if key == "FIELDS":
                    fields = val.split()
                elif key == "SIZE":
                    size = list(map(int, val.split()))
                elif key == "TYPE":
                    typ = val.split()
                elif key == "COUNT":
                    count = list(map(int, val.split()))
                elif key == "POINTS":
                    points = int(val)
                elif key == "WIDTH":
                    width = int(val)
                elif key == "HEIGHT":
                    height = int(val)
                    points = width * height

        data_type = header.get("DATA", "binary")

        # 构建结构化 dtype
        if fields and size:
            dtype_fields = []
            xyz_indices = []
            for i, (name, sz, t) in enumerate(
                zip(fields, size, typ or ["F"] * len(fields))
            ):
                np_t = {4: "<f4", 8: "<f8"}.get(sz, "<f4")
                dtype_fields.append((name, np_t))
                if name.lower() in ("x", "y", "z"):
                    xyz_indices.append(i)
            dtype = np.dtype(dtype_fields)

            if data_type == "binary":
                raw = np.fromfile(f, dtype=dtype, count=points)
            else:
                # ASCII
                raw = np.loadtxt(f, dtype=dtype, max_rows=points)

            # 提取 x/y/z
            xyz = np.column_stack([raw[name] for name in ("x", "y", "z")])
            return np.asarray(xyz, dtype=np.float64)
        else:
            # 退化为纯二进制点云
            raw = np.fromfile(f, dtype="<f4", count=points * 3).reshape(-1, 3)
            return np.asarray(raw, dtype=np.float64)


def voxel_down_sample(points, voxel_size):
    """
    体素下采样，返回 (M, 3) float64 数组。
    - 将点云划分到体素网格中
    - 每个体素取所有点的重心作为输出点
    """
    if len(points) == 0 or voxel_size <= 0:
        return points.copy()

    # 计算体素坐标
    voxel = np.floor(points / voxel_size).astype(np.int64)

    # 将三维体素坐标哈希为一维（用位移防碰撞）
    shift = 20  # 2^20 足够区分 ~1000km @ 1mm
    hashed = (voxel[:, 0].astype(np.int64) << (2 * shift)) \
           ^ (voxel[:, 1].astype(np.int64) << shift) \
           ^ voxel[:, 2].astype(np.int64)

    # 按哈希分组，取每组平均
    _, inverse, counts = np.unique(hashed, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    centroids = sums / counts[:, np.newaxis]
    return centroids


def transform_points(points, T):
    """
    用 4x4 变换矩阵 T 变换 (N,3) 点云。
    """
    pts = np.column_stack([points, np.ones(len(points), dtype=points.dtype)])
    return (pts @ T.T)[:, :3]


def icp_registration(source, target, initial, max_distance, max_iteration=50):
    """
    点对点 ICP 配准。

    参数:
        source: (N,3) 源点云（将被变换到 target 系）
        target: (M,3) 目标点云（参考系）
        initial: (4,4) 初始变换矩阵
        max_distance: 有效对应点最大距离
        max_iteration: 最大迭代次数

    返回:
        transformation: (4,4) 累积变换矩阵（source→target）
        fitness: 内点比例（有对应点的点占总点数比例）
    """
    if len(source) < 3 or len(target) < 3:
        return initial.copy(), 0.0

    KDTree = _get_kdtree()
    tree = KDTree(target)

    # 用初始变换预处理源点云
    src = transform_points(source, initial)
    T = initial.copy()

    prev_count = -1
    for _ in range(max_iteration):
        # 最近邻搜索（带距离上限，KDTree 的 distance_upper_bound 不兼容 cKDTree 的老参数）
        dists, indices = tree.query(src, k=1, distance_upper_bound=max_distance)
        valid = np.isfinite(dists) & (indices < len(target))
        good = np.where(valid)[0]

        n_valid = len(good)
        if n_valid < 3:
            break

        # 对应点对
        src_pts = src[good]
        tgt_pts = target[indices[good]]

        # 去质心
        s_cent = src_pts.mean(axis=0)
        t_cent = tgt_pts.mean(axis=0)
        s_dev = src_pts - s_cent
        t_dev = tgt_pts - t_cent

        # SVD 求解旋转
        H = s_dev.T @ t_dev
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = t_cent - R @ s_cent

        # 构造增量变换
        delta = np.eye(4)
        delta[:3, :3] = R
        delta[:3, 3] = t
        T = delta @ T

        # 更新变换后点云
        src = transform_points(src, delta)

        # 收敛判断
        if n_valid == prev_count:
            break
        prev_count = n_valid

    # 计算 fitness
    dists, indices = tree.query(src, k=1, distance_upper_bound=max_distance)
    valid = np.isfinite(dists) & (indices < len(target))
    fitness = float(np.sum(valid)) / len(source) if len(source) > 0 else 0.0

    return T, fitness


# ======================================================================
# 主类
# ======================================================================


class FastLIOLocalization(Node):
    """FAST-LIO2 全局重定位：将实时激光扫描与预存地图 ICP 配准，计算 map→odom 变换"""

    def __init__(self):
        super().__init__("fast_lio_localization")

        # ============ 状态变量 ============
        self.global_map = None          # 预存的全局 PCD 地图（numpy 数组 (N,3)）
        self.T_map_to_odom = np.eye(4)  # map→odom 变换矩阵（ICP 配准结果）
        self.cur_odom = None            # 最新的 FAST-LIO2 里程计
        self.cur_scan = None            # 最新的 FAST-LIO2 扫描点云 (numpy (N,3))
        self.scan_buffer = []           # 多帧扫描累积缓存（提升 ICP 稳定性）
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
        self.create_subscription(PointCloud2, "/unilidar/cloud", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        # ============ 定时器 ============
        self.timer_localisation = self.create_timer(
            1.0 / self.get_parameter("freq_localization").value,
            self.localisation_timer_callback
        )

        # 自动初始化：5 秒后若未收到 /initialpose，以原点 X+ 自动开始 ICP
        # （替代手动在 RViz 中点 "2D Pose Estimate" 或外部运行 init_pose.py）
        self._auto_init_timer = self.create_timer(5.0, self._auto_init_if_needed)

    def pose_to_mat(self, pose):
        """将 ROS Pose 消息转为 4×4 齐次变换矩阵"""
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def msg_to_array(self, pc_msg):
        """将 ROS PointCloud2 消息转为 numpy 点云数组 (N,3)，不依赖 ros2_numpy"""
        # 获取字段名→偏移映射
        offsets = {}
        for f in pc_msg.fields:
            offsets[f.name] = f.offset

        # 按 point_step 切分二进制数据
        ps = pc_msg.point_step
        raw = np.frombuffer(pc_msg.data, dtype=np.uint8).reshape(-1, ps)
        n = raw.shape[0]
        pts = np.zeros((n, 3), dtype=np.float64)

        for i, name in enumerate(("x", "y", "z")):
            off = offsets.get(name)
            if off is not None and off + 4 <= ps:
                pts[:, i] = raw[:, off:off + 4].view("<f4").flatten().astype(np.float64)

        return pts

    def registration_at_scale(self, scan, map_, initial, scale):
        """
        在指定尺度下执行 ICP 配准

        多尺度配准策略：
        - 大尺度 (scale=5)：粗配准，用大体素快速收敛到大体位置
        - 小尺度 (scale=1)：精配准，用小体素精确微调位姿
        """
        src_down = voxel_down_sample(
            scan, self.get_parameter("scan_voxel_size").value * scale
        )
        tgt_down = voxel_down_sample(
            map_, self.get_parameter("map_voxel_size").value * scale
        )

        transformation, fitness = icp_registration(
            src_down, tgt_down,
            initial=initial,
            max_distance=0.5 * scale,
            max_iteration=50,
        )
        return transformation, fitness

    def inverse_se3(self, trans):
        """计算 SE3 变换的逆变换：
        [R t; 0 1]^{-1} = [R^T  -R^T*t; 0 1]
        """
        trans_inverse = np.eye(4)
        trans_inverse[:3, :3] = trans[:3, :3].T          # R^{-1} = R^T
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])  # t' = -R^T * t
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc):
        """将 numpy 点云发布为 ROS PointCloud2 消息，不依赖 ros2_numpy"""
        n = pc.shape[0]
        has_intensity = pc.shape[1] >= 4

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        if has_intensity:
            fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
            point_step = 16

        # 构建连续二进制 buffer
        buf = np.empty((n, 4 if has_intensity else 3), dtype=np.float32)
        buf[:, 0] = pc[:, 0].astype(np.float32)
        buf[:, 1] = pc[:, 1].astype(np.float32)
        buf[:, 2] = pc[:, 2].astype(np.float32)
        if has_intensity:
            buf[:, 3] = pc[:, 3].astype(np.float32)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n
        msg.data = buf.tobytes()
        msg.is_dense = True
        publisher.publish(msg)

    def crop_global_map_in_FOV(self, pose_estimation):
        """
        根据当前位姿估计，从全局地图中裁剪出视野范围内的局部子地图

        步骤：
        1. 将全局地图点云变换到 base_link 坐标系
        2. 根据 FOV 角度和距离过滤（只保留传感器视野内的点）
        3. 发布子地图用于可视化
        4. 返回裁剪后的点云用于 ICP 配准
        """
        if self.cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        else:
            T_odom_to_base_link = np.eye(4)
            self.get_logger().warn("No odometry yet, skip odom transform in crop")
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # 将全局地图变换到 base_link 系
        global_map_in_map = self.global_map
        ones = np.ones((len(global_map_in_map), 1), dtype=np.float64)
        global_map_h = np.column_stack([global_map_in_map, ones])
        global_map_in_base_link = (T_base_link_to_map @ global_map_h.T).T

        # 根据 FOV 过滤
        fov = self.get_parameter("fov").value
        fov_far = self.get_parameter("fov_far").value

        if fov > 3.14:
            # 全向模式：只做距离过滤
            mask = global_map_in_base_link[:, 0] < fov_far
        else:
            # 有限 FOV：做距离 + 角度过滤
            angles = np.abs(np.arctan2(
                global_map_in_base_link[:, 1],
                global_map_in_base_link[:, 0]
            ))
            mask = (
                (global_map_in_base_link[:, 0] > 0)
                & (global_map_in_base_link[:, 0] < fov_far)
                & (angles < fov / 2.0)
            )

        global_map_in_FOV = global_map_in_map[mask]

        # 发布子地图（降采样到 1/10 用于 RViz 可视化）
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        if len(global_map_in_FOV) > 0:
            self.publish_point_cloud(
                self.pub_submap, header, global_map_in_FOV[::10]
            )
        else:
            self.get_logger().warn("FOV crop returned empty point cloud")

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        """
        执行全局定位（多尺度 ICP 配准）

        策略：
        1. 粗配准 (scale=5)：大尺度体素下采样，快速收敛
        2. 精配准 (scale=1)：小尺度体素下采样，精确微调
        3. 如果 fitness > 阈值则更新 map→odom 变换
        """
        scan_tobe_mapped = self.cur_scan.copy()
        # 合并累积缓存中的多帧扫描点云，提升 ICP 稳定性
        if len(self.scan_buffer) > 1:
            merged = np.concatenate(self.scan_buffer, axis=0)
            scan_tobe_mapped = merged
        # 清空缓存，开始下一轮累积
        self.scan_buffer = []

        n_scan = len(scan_tobe_mapped)
        # 跳过单帧（< 2000 点），点数太少 ICP 不可靠
        if n_scan < 2000:
            self.get_logger().warn(f"Skipping ICP: only {n_scan} pts, need >= 2000")
            return

        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)
        n_map = len(global_map_in_FOV)
        self.get_logger().info(
            f"ICP input: scan={n_scan} pts, map_FOV={n_map} pts"
        )

        if n_map < 100:
            self.get_logger().warn(f"FOV map too small ({n_map} pts), skip ICP")
            return

        # 粗配准
        transformation, fitness_coarse = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5
        )

        # 精配准（以粗配准结果为初值）
        transformation, fitness = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1
        )

        self.get_logger().info(
            f"ICP fitness: coarse={fitness_coarse:.3f}, fine={fitness:.3f}, "
            f"threshold={self.get_parameter('localization_threshold').value}"
        )

        if fitness > self.get_parameter("localization_threshold").value:
            x, y, z = transformation[:3, 3]
            self.get_logger().info(
                f"map→odom: x={x:.3f} y={y:.3f} z={z:.3f}"
            )
            # 限制单次更新幅度，防止 ICP 跳变到错误位置
            dx = x - self.T_map_to_odom[0, 3]
            dy = y - self.T_map_to_odom[1, 3]
            dz = z - self.T_map_to_odom[2, 3]
            if abs(dx) > 1.0 or abs(dy) > 1.0 or abs(dz) > 0.3:
                self.get_logger().warn(
                    f"ICP jump too large ({dx:.2f}, {dy:.2f}, {dz:.2f}), rejecting"
                )
            else:
                self.T_map_to_odom = transformation
                self.publish_odom(transformation)
        else:
            self.get_logger().warn(
                f"Fitness score {fitness} less than localization threshold "
                f"{self.get_parameter('localization_threshold').value}"
            )

    def cb_save_cur_odom(self, msg):
        """保存最新的 FAST-LIO2 里程计"""
        self.cur_odom = msg

    def cb_save_cur_scan(self, msg):
        """保存原始雷达点云，累积到缓存中。对 raw scan 做 Y/Z 翻转以匹配地图坐标系"""
        pc = self.msg_to_array(msg)
        # Y/Z 翻转：匹配 fastlio preprocess 的 default_handler 坐标系
        pc = pc * [1.0, -1.0, -1.0]
        self.cur_scan = pc
        # 累积多帧到缓存（上限 10 帧 ≈ 1 秒 @ 10Hz，避免淹没稀疏地图）
        self.scan_buffer.append(pc)
        if len(self.scan_buffer) > 10:
            self.scan_buffer.pop(0)
        # 使用当前时间戳发布点云，避免 RViz TF 滤波器因传感器时间戳滞后而丢弃消息
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_pc_in_map, header, pc)

    def initialize_global_map(self):
        """加载 PCD 全局地图并进行体素下采样"""
        path = self.get_parameter("pcd_map_path").value
        v_size = self.get_parameter("map_voxel_size").value
        self.get_logger().info(f"Loading map: {path}, voxel_size={v_size}")

        pcd = read_pcd(path)
        n_raw = len(pcd)
        self.get_logger().info(f"Map loaded: raw={n_raw} pts")

        self.global_map = voxel_down_sample(pcd, v_size)
        n_down = len(self.global_map)
        self.get_logger().info(f"Map downsampled: {n_down} pts")

    def cb_initialize_pose(self, msg):
        """
        收到初始位姿（来自 RViz "2D Pose Estimate" 或 publish_initial_pose.py）
        后，执行首次全局定位

        RViz 的初始位姿是 map→base_link，但 ICP 需要 map→odom，
        用当前里程计做转换：T_map_to_odom = T_map_to_base_link * inv(T_odom_to_base_link)
        """
        T_map_to_base_link = self.pose_to_mat(msg.pose.pose)

        if self.cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
            T_map_to_odom = np.matmul(T_map_to_base_link, self.inverse_se3(T_odom_to_base_link))
            self.get_logger().info("Initial pose received, converted map→base_link → map→odom")
        else:
            T_map_to_odom = T_map_to_base_link
            self.get_logger().warn("No odometry yet, using map→base_link as map→odom (approximate)")

        self.initialized = True

        if self.cur_scan is not None:
            self.global_localization(T_map_to_odom)

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

    def _auto_init_if_needed(self):
        """自动初始化兜底：5 秒后若未收到 /initialpose，以原点 X+ 启动 ICP"""
        self.get_logger().info(f"_auto_init_if_needed: initialized={self.initialized} cur_scan={'✓' if self.cur_scan is not None else '✗'}")
        if not self.initialized:
            if self.cur_scan is not None:
                self.get_logger().warn(
                    "No /initialpose received within 5s — auto-initializing at (0, 0) facing X+"
                )
                self.T_map_to_odom = np.eye(4)
                self.initialized = True
                self.global_localization(np.eye(4))
            return  # 无点云则等下一次 tick

        # 已初始化，销毁本定时器
        self.destroy_timer(self._auto_init_timer)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
