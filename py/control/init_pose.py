#!/usr/bin/env python3
"""发布 /initialpose 自动初始化 ICP 定位（原点, X正向）"""
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

rclpy.init()
n = Node("init_pose_pub")
pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
time.sleep(0.5)  # 等订阅器就绪

msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.position.x = 0.0
msg.pose.pose.orientation.w = 1.0  # yaw=0 → X正向
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.25

pub.publish(msg)
n.get_logger().info("Published /initialpose: (0,0,0) facing X+")
time.sleep(0.3)
# 直接 exit，避免 CycloneDDS rclpy.shutdown() 在 Celeron 上挂死
os._exit(0)
