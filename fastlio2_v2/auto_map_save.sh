#!/bin/bash
# 自动建图保存：监控 publish_map 日志，累计点数达目标时自动保存并退出
# 用法：bash auto_map_save.sh [目标点数，默认 1000000]
#       仅保留 x>0 的点云（过滤后方点云）

TARGET=${1:-1000000}
FIFO=$(mktemp -u)
mkfifo "$FIFO"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "自动建图已启动，目标点数: $TARGET，达到后自动保存并退出。"
echo "过滤器：仅保留 x>0 的点云"

# 后台启动 X 方向过滤器（只保留 x>0 的点）
python3 /home/hyper/program/2026_Gsing-second_ROS/py/pointcloud_x_filter.py &
FILTER_PID=$!
sleep 1

# 后台启动建图，使用过滤后的点云话题，输出写入 FIFO
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml \
  -r /livox/lidar:=/unilidar/cloud_filtered \
  -r /livox/imu:=/unilidar/imu > "$FIFO" 2>&1 &
MAPPING_PID=$!

# 从 FIFO 读取日志
while IFS= read -r line; do
    echo "$line"
    if echo "$line" | grep -q "publish_map:.*total="; then
        TOTAL=$(echo "$line" | sed 's/.*total=//' | awk '{print $1}')
        if [ -n "$TOTAL" ] && [ "$TOTAL" -ge "$TARGET" ] 2>/dev/null; then
            echo ""
            echo "============================================"
            echo " 累计点数 $TOTAL >= $TARGET，保存地图..."
            echo "============================================"
            ros2 service call /map_save std_srvs/srv/Trigger
            echo "============================================"
            echo " 地图已保存，停止建图。"
            echo "============================================"
            kill $MAPPING_PID 2>/dev/null
            wait $MAPPING_PID 2>/dev/null
            break
        fi
    fi
done < "$FIFO"

rm -f "$FIFO"

# 清理过滤器
kill $FILTER_PID 2>/dev/null
wait $FILTER_PID 2>/dev/null

echo "建图已退出。"
