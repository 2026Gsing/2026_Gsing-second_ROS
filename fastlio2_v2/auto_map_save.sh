#!/bin/bash
# 手动建图：启动 FAST-LIO2 建图 + x>0 点云过滤，按回车保存地图并退出
# 用法：bash auto_map_save.sh

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "建图已启动，仅保留 x>0 的点云。"
echo "按 Enter 保存地图并退出。"

# 后台启动 X 方向过滤器（只保留 x>0 的点）
python3 /home/hyper/program/2026_Gsing-second_ROS/py/pointcloud_x_filter.py &
FILTER_PID=$!
sleep 1

# 后台启动建图，使用过滤后的点云话题
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml \
  -p "common.lid_topic:=/unilidar/cloud_filtered" \
  -r /livox/imu:=/unilidar/imu &
MAPPING_PID=$!

# 等待用户按回车 → 保存地图并退出
read -r

echo ""
echo "============================================"
echo " 保存地图..."
echo "============================================"
ros2 service call /map_save std_srvs/srv/Trigger
echo "============================================"
echo " 地图已保存，停止建图。"
echo "============================================"

# 清理（强杀进程组和所有子进程）
pkill -9 -f "fastlio_mapping" 2>/dev/null
pkill -9 -f "pointcloud_x_filter" 2>/dev/null
wait $MAPPING_PID 2>/dev/null
wait $FILTER_PID 2>/dev/null

echo "建图已退出。"
