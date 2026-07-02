#!/usr/bin/env python3
"""
arrival_detector.py — 到达检测工具

订阅 /localization (Odometry)，判断机器人是否到达目标点。
"""

import math
from typing import Optional
from geometry_msgs.msg import Pose


def quaternion_to_yaw(q) -> float:
    """四元数 → 偏航角 (rad)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ArrivalDetector:
    """
    到达检测器。
    - 位置阈值（欧氏距离） + 朝向阈值
    - 连续 N 帧满足才算到达（防抖动）
    """

    def __init__(
        self,
        position_threshold: float = 0.25,   # m
        angle_threshold: float = 0.30,       # rad (~17°)
        settle_frames: int = 5,              # 连续满足帧数
    ):
        self.pos_thresh = position_threshold
        self.angle_thresh = angle_threshold
        self.settle_frames = settle_frames
        self._count = 0
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_yaw: Optional[float] = None

    def set_target(self, x: float, y: float, yaw: float = 0.0):
        """设定目标点"""
        self._target_x = x
        self._target_y = y
        self._target_yaw = yaw
        self._count = 0

    def clear_target(self):
        """清除目标"""
        self._target_x = None
        self._count = 0

    def check(self, pose: Pose) -> tuple[bool, float]:
        """
        检查是否到达目标。
        返回 (arrived, distance)
        """
        if self._target_x is None:
            return False, float('inf')

        dx = pose.position.x - self._target_x
        dy = pose.position.y - self._target_y
        dist = math.sqrt(dx * dx + dy * dy)

        yaw = quaternion_to_yaw(pose.orientation)
        dyaw = abs(normalize_angle(yaw - self._target_yaw))

        arrived_now = dist < self.pos_thresh and dyaw < self.angle_thresh

        if arrived_now:
            self._count += 1
        else:
            self._count = 0

        return self._count >= self.settle_frames, dist

    @property
    def has_target(self) -> bool:
        return self._target_x is not None
