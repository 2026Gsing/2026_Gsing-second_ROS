#!/bin/bash
# 手动建图：启动 FAST-LIO2 建图 + x>0 点云过滤，按回车保存地图并退出
# 用法：bash auto_map_save.sh
# 每次保存为 ../map/PCD1.pcd, PCD2.pcd, ... 顺序编号，不覆盖已有文件

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 自动编号：找到下一个可用的 PCD 编号
MAP_DIR="../map"
mkdir -p "$MAP_DIR"
NEXT_NUM=1
for f in "$MAP_DIR"/PCD[0-9]*.pcd; do
    [ -e "$f" ] || continue
    BASENAME=$(basename "$f" .pcd)
    NUM=${BASENAME#PCD}
    if [[ "$NUM" =~ ^[0-9]+$ ]] && [ "$NUM" -ge "$NEXT_NUM" ]; then
        NEXT_NUM=$((NUM + 1))
    fi
done
SAVE_PATH="$MAP_DIR/PCD${NEXT_NUM}.pcd"

echo "建图已启动，仅保留 x>0.0 的点云。"
echo "本次保存目标: ${SAVE_PATH}"
echo "按 Enter 保存地图并退出。"

# 后台启动 X 方向过滤器（只保留 x>0 的点）
python3 ../py/pointcloud_x_filter.py &
FILTER_PID=$!
sleep 1

# 后台启动建图，直接指定 map_file_path 为编号文件，关闭 IMU 只用激光
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml \
  -p "common.lid_topic:=/unilidar/cloud_filtered" \
  -r /livox/imu:=/unilidar/imu \
  -p imu_en:=false \
  -p map_file_path:="${SAVE_PATH}" &
MAPPING_PID=$!

# 等待用户按回车 → 保存地图并退出
read -r

echo ""
echo "============================================"
echo " 保存地图 → ${SAVE_PATH} ..."
echo "============================================"
ros2 service call /map_save std_srvs/srv/Trigger
echo "============================================"
echo " 地图已保存，停止建图。"
echo "============================================"

# 清理
pkill -9 -f "fastlio_mapping" 2>/dev/null
pkill -9 -f "pointcloud_x_filter" 2>/dev/null
wait $MAPPING_PID 2>/dev/null
wait $FILTER_PID 2>/dev/null

echo "建图已退出。"
