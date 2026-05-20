#!/usr/bin/env python3
import rclpy
print("🔴 第一步：导入rclpy成功")  # 这行是纯Python打印，ROS2没初始化也能输出
rclpy.init()
print("🟢 第二步：ROS2初始化成功")
rclpy.shutdown()
print("🟡 第三步：ROS2关闭成功")
