#!/bin/bash
# ros-run.sh — 用 system Python 3.12 运行 ROS2 脚本
# 用法: ./ros-run.sh py/control/box_pick_node.py
#
# 自动 source ROS2 + workspaces + 设置 RMW

HERE="$(cd "$(dirname "$0")" && pwd)"

# Source everything
source /opt/ros/jazzy/setup.bash
source "$HERE/fastlio2_v2/install/setup.bash" 2>/dev/null
source "$HERE/nav2_ws1/install/setup.bash" 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Run with system python
exec /usr/bin/python3 "$@"
