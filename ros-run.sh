#!/bin/bash
# ros-run.sh — 用 system Python 3.10 运行 ROS2 脚本
# 用法: ./ros-run.sh py/control/box_pick_node.py
#
# 自动 source ROS2 + workspaces + 设置 RMW

HERE="$(cd "$(dirname "$0")" && pwd)"

# 自动检测可用 ROS2 发行版 (支持 jazzy / humble)
_ROS_DISTRO=""
for _d in jazzy humble; do
    if [ -f "/opt/ros/$_d/setup.bash" ]; then
        _ROS_DISTRO="$_d"
        break
    fi
done
if [ -z "$_ROS_DISTRO" ]; then
    echo "❌ 未检测到 ROS2 (尝试过 jazzy, humble)" >&2
    exit 1
fi
source "/opt/ros/$_ROS_DISTRO/setup.bash"
source "$HERE/fastlio2_v2/install/setup.bash" 2>/dev/null
source "$HERE/nav2_ws1/install/setup.bash" 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Run with system python
exec /usr/bin/python3 "$@"
