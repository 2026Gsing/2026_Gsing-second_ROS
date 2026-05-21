#!/bin/bash
# 启动 Nav2（无仿真）+ 加载任务场地地图
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash

# 启动 Nav2 核心
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  autostart:=True \
  params_file:=$NAV2_DIR/params/nav2_task_params.yaml \
  map:=$NAV2_DIR/maps/task_field_map.yaml

# 新开终端启动 RViz2（手动执行）
# ros2 launch nav2_bringup rviz_launch.py
