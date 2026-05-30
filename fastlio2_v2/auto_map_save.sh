#!/bin/bash
# 自动建图保存：监控 publish_map 日志，累计点数达目标时自动保存并退出
# 用法：bash auto_map_save.sh [目标点数，默认 1000000]

TARGET=${1:-1000000}
FIFO=$(mktemp -u)
mkfifo "$FIFO"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "自动建图已启动，目标点数: $TARGET，达到后自动保存并退出。"

# 后台启动建图，输出写入 FIFO
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml \
  -r /livox/lidar:=/unilidar/cloud \
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
echo "建图已退出。"
