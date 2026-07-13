#!/usr/bin/env python3
"""
点云 X 方向过滤节点
订阅 /unilidar/cloud，只保留 x > 0 的点，发布到 /unilidar/cloud_filtered

用法（独立运行）：
  python3 pointcloud_x_filter.py

自动建图（auto_map_save.sh 已内置此过滤）：
  bash auto_map_save.sh 50000000
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudXFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_x_filter')

        # 参数
        self.declare_parameter('x_min', 0.0)     # X 方向下限（前方）
        self.declare_parameter('radius_min', 0.0)  # 最小半径（去除狗身等近距离点）
        self.x_min = self.get_parameter('x_min').value
        self.radius_min = self.get_parameter('radius_min').value

        self.sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.cloud_callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/unilidar/cloud_filtered', 10)

        self.get_logger().info(
            f'点云过滤器已启动，仅保留 x > {self.x_min}, 半径 > {self.radius_min} 的点')

    def cloud_callback(self, msg):
        # 解析点云为 numpy 数组
        points = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity', 'ring'),
            skip_nans=True))

        if not points:
            return

        arr = np.array(points, dtype=[
            ('x', float), ('y', float), ('z', float),
            ('intensity', float), ('ring', float)
        ])

        # 计算每个点到 LiDAR 的距离
        dist = np.sqrt(arr['x']**2 + arr['y']**2 + arr['z']**2)

        # 只保留 x > x_min 且距离 > radius_min 的点
        mask = (arr['x'] > self.x_min) & (dist > self.radius_min)
        filtered = arr[mask]

        if len(filtered) == 0:
            self.get_logger().warn('过滤后点云为空！')
            return

        # 构建新的 PointCloud2 消息
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        out = pc2.create_cloud(
            msg.header, fields,
            [(p['x'], p['y'], p['z'], p['intensity'], p['ring'])
             for p in filtered]
        )

        self.pub.publish(out)
        self.get_logger().debug(
            f'过滤: {len(points)} → {len(filtered)} 点 (x>{self.x_min}, 半径>{self.radius_min})',
            throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = PointCloudXFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
