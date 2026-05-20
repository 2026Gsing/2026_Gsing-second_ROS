#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云保存节点
功能：
  1. 接收雷达点云
  2. 按空格键保存当前帧点云到文件
  3. 可选择保存原始点云或过滤后的点云
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
from datetime import datetime
import threading
import sys
import tty
import termios

# ==================== 配置 ====================
SAVE_DIR = "pointcloud_data"  # 保存目录
SAVE_FILTERED = False  # False=保存原始点云, True=保存过滤后的点云
X_FILTER = 0  # 保留 X>0 的点（0=不过滤）
Z_FILTER = 0  # 保留 Z>0 的点（0=不过滤）


class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')

        # 创建保存目录
        if not os.path.exists(SAVE_DIR):
            os.makedirs(SAVE_DIR)
            self.get_logger().info(f"创建保存目录: {SAVE_DIR}")

        # 订阅雷达点云
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10
        )

        # 保存最新的点云
        self.latest_pts = None
        self.latest_msg = None

        # 启动键盘监听线程
        self.running = True
        self.key_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_thread.start()

        self.get_logger().info("=" * 50)
        self.get_logger().info("点云保存节点已启动")
        self.get_logger().info(f"保存目录: {SAVE_DIR}")
        self.get_logger().info(f"保存模式: {'过滤后' if SAVE_FILTERED else '原始'}点云")
        self.get_logger().info("按空格键保存当前点云，按 Q 退出")
        self.get_logger().info("=" * 50)

    def cloud_callback(self, msg):
        """接收点云回调"""
        try:
            # 转换点云
            pts = self.pc2_to_np(msg)
            if len(pts) == 0:
                return

            # 可选：过滤点云
            if SAVE_FILTERED:
                if X_FILTER > 0:
                    pts = pts[pts[:, 0] > X_FILTER]
                if Z_FILTER > 0:
                    pts = pts[pts[:, 2] > Z_FILTER]

            self.latest_pts = pts
            self.latest_msg = msg

        except Exception as e:
            self.get_logger().error(f"点云处理异常: {e}")

    def pc2_to_np(self, msg):
        """ROS PointCloud2 → numpy (x,y,z)"""
        arr = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            arr.append([p[0], p[1], p[2]])
        return np.array(arr, dtype=np.float32) if arr else np.zeros((0, 3))

    def save_pointcloud(self):
        """保存点云到文件"""
        if self.latest_pts is None or len(self.latest_pts) == 0:
            self.get_logger().warn("没有点云数据，无法保存")
            return

        # 生成文件名（时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = os.path.join(SAVE_DIR, f"pointcloud_{timestamp}.npy")
        txt_filename = os.path.join(SAVE_DIR, f"pointcloud_{timestamp}.txt")

        try:
            # 保存为npy格式（二进制，推荐）
            np.save(filename, self.latest_pts)
            
            # 同时保存为txt格式（可选）
            np.savetxt(txt_filename, self.latest_pts, fmt='%.6f', delimiter=',')
            
            self.get_logger().info(f"✅ 点云已保存: {filename}")
            self.get_logger().info(f"   点数: {len(self.latest_pts)}, 格式: npy + txt")
        except Exception as e:
            self.get_logger().error(f"保存失败: {e}")

    def key_listener(self):
        """键盘监听线程"""
        # 保存原始终端设置
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(fd)
            
            while self.running and rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':  # 空格键保存
                    self.get_logger().info("正在保存点云...")
                    self.save_pointcloud()
                elif ch == 'q' or ch == 'Q':  # Q键退出
                    self.get_logger().info("退出中...")
                    self.running = False
                    rclpy.shutdown()
                    break
                elif ch == '\x03':  # Ctrl+C
                    break
        except Exception as e:
            self.get_logger().error(f"键盘监听错误: {e}")
        finally:
            # 恢复终端设置
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def __del__(self):
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()