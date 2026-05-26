#!/bin/bash
# start_nav2_full.sh — 完整启动脚本：地图服务器 + Nav2 核心
#
# 功能：
#   1. 启动 map_server 加载任务场地地图
#   2. 激活 map_server 生命周期
#   3. 启动 Nav2 核心（planner、controller、bt_navigator 等）
#
# 使用方式：
#   ./start_nav2_full.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash

# 步骤1：启动地图服务器（后台运行）
echo "启动地图服务器..."
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$NAV2_DIR/maps/task_field_map.yaml" &
sleep 3

# 步骤2：激活地图服务器生命周期
echo "激活地图服务器..."
ros2 lifecycle set /map_server activate &
sleep 2

# 步骤3：启动Nav2核心
echo "启动Nav2核心..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  autostart:=True \
  params_file:=$NAV2_DIR/params/nav2_task_params.yaml \
  map:=$NAV2_DIR/maps/task_field_map.yaml
