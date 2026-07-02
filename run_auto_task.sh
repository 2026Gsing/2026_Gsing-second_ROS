#!/bin/bash
# run_auto_task.sh — 任务赛视觉全自动执行 一键启动
#
# 使用方式：
#   cd Gsing/2026_Gsing-second_ROS
#   bash run_auto_task.sh
#
# 前提（请先在别的终端启动）：
#   1. LiDAR 驱动： ros2 launch unitree_lidar_ros2 launch.py
#   2. FAST-LIO2：  ros2 run fast_lio fastlio_mapping --ros-args -p ...
#   3. 全局定位：    ros2 launch fast_lio_localization 1.launch.py ...
#
# 本脚本启动：
#   - Nav2 静态地图导航
#   - 串口桥（底盘速度 0x10 + 自动任务 0x15）
#   - 视觉自动任务状态机

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$SCRIPT_DIR/nav2_ws1"
PY_DIR="$SCRIPT_DIR/py"

echo "=============================="
echo "  任务赛视觉全自动执行 启动"
echo "=============================="

# ============ 环境 ============
source /opt/ros/jazzy/setup.bash
source "$NAV2_DIR/install/setup.bash" 2>/dev/null || echo "⚠️  nav2_ws1 未 build，仅启动 Python 节点"

# ============ 参数 ============
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
BAUD="${BAUD:-115200}"
MAP_FILE="${MAP_FILE:-$NAV2_DIR/src/dog_nav2_bringup/maps/task_field_map.yaml}"

# ============ 1. 串口桥 ============
echo ""
echo "【1/3】启动串口桥..."
ros2 run dog_nav2_bringup cmd_vel_chassis_serial \
  --ros-args \
  -p serial_port:="$SERIAL_PORT" \
  -p baud_rate:="$BAUD" \
  -p cmd_vel_topic:=/cmd_vel \
  -p send_rate_hz:=50.0 &

SERIAL_PID=$!
sleep 2

# ============ 2. Nav2 ============
echo ""
echo "【2/3】启动 Nav2 导航..."
if [ -f "$MAP_FILE" ]; then
  ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
    map:="$MAP_FILE" &
  NAV_PID=$!
  sleep 3
else
  echo "⚠️  地图文件 $MAP_FILE 不存在，跳过 Nav2 启动"
  echo "   请先用 task_field_competition.sh 生成地图"
fi

# ============ 3. 视觉自动任务节点 ============
echo ""
echo "【3/3】启动视觉自动任务节点..."
python3 "$PY_DIR/vision_auto_task_node.py" &
VISION_PID=$!

echo ""
echo "=============================="
echo "  所有组件已启动"
echo "  在 vision 终端输入 start 开始"
echo "=============================="
echo "  PID: 串口桥=$SERIAL_PID"
echo "  PID: 视觉=$VISION_PID"
echo ""
echo "  按 Ctrl+C 停止所有"

# ============ 清理 ============
cleanup() {
  echo ""
  echo "正在停止..."
  kill $VISION_PID 2>/dev/null
  kill $NAV_PID 2>/dev/null
  kill $SERIAL_PID 2>/dev/null
  wait
  echo "已停止"
}
trap cleanup EXIT INT TERM

# 等待所有后台进程
wait
