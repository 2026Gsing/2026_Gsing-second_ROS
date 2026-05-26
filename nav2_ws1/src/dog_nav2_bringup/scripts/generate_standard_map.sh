#!/bin/bash
# generate_standard_map.sh — 竞赛标准地图生成脚本
#
# 功能：
#   使用 ImageMagick 生成 6m×4m （400×600px）竞赛场地栅格地图，
#   包含围墙、物资箱存放区（8个）、归位区（4个彩色）、
#   智力题显示区、启动区。然后启动并激活 map_server。
#
# 地图布局：
#   - 围墙：20px 黑色边框
#   - 存放区：8个 25×25px 黑色方块（两排四列）
#   - 归位区：4个 40×40px 彩色方块（绿/灰/蓝/红）
#   - 智力题区：160×90px 浅灰色区域
#   - 启动区：80×80px 浅蓝色区域
#
# 使用方式：
#   ./generate_standard_map.sh

# ====================== 环境初始化 ======================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 终止旧进程
pkill -f "map_server" > /dev/null 2>&1
sleep 2

# 配置ROS环境（避免conda冲突）
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash

# 创建目录（确保存在）
mkdir -p $NAV2_DIR/maps/
cd $NAV2_DIR/maps/
rm -f task_field_map.pgm task_field_map.yaml

# ====================== 生成标准地图 ======================
# 1. 生成400×600全白基础图（可通行区域）
if ! convert -size 400x600 xc:white task_field_map.pgm; then
    echo -e "\033[31m❌ 安装imagemagick：sudo apt install -y imagemagick\033[0m"
    exit 1
fi

# 2. 绘制封闭围墙（黑色，20像素宽，无缺口）
convert task_field_map.pgm -fill black \
-draw "rectangle 0,0 399,19" \
-draw "rectangle 0,580 399,599" \
-draw "rectangle 0,0 19,599" \
-draw "rectangle 380,0 399,599" \
task_field_map.pgm

# 3. 绘制物资箱存放区（8个，25×25像素，黑色）
convert task_field_map.pgm -fill black \
-draw "rectangle 37,87 62,112" \
-draw "rectangle 97,87 122,112" \
-draw "rectangle 157,87 182,112" \
-draw "rectangle 217,87 242,112" \
-draw "rectangle 37,147 62,172" \
-draw "rectangle 97,147 122,172" \
-draw "rectangle 157,147 182,172" \
-draw "rectangle 217,147 242,172" \
task_field_map.pgm

# 4. 绘制物资箱归位区（4个，40×40像素，按规则配色）
convert task_field_map.pgm \
# 0号：绿色（食品箱）
-fill "#00FF00" -draw "rectangle 60,500 99,539" \
# 1号：灰色（工具箱）
-fill "#888888" -draw "rectangle 140,500 179,539" \
# 2号：蓝色（仪器箱）
-fill "#0000FF" -draw "rectangle 220,500 259,539" \
# 3号：红色（药品箱）
-fill "#FF0000" -draw "rectangle 300,500 339,539" \
task_field_map.pgm

# 5. 绘制智力题显示区（浅灰色，160×90像素）
convert task_field_map.pgm -fill "#DDDDDD" \
-draw "rectangle 20,460 179,549" \
task_field_map.pgm

# 6. 绘制启动区（浅蓝色，80×80像素）
convert task_field_map.pgm -fill "#ADD8E6" \
-draw "rectangle 160,20 239,99" \
task_field_map.pgm

# ====================== 生成YAML配置 ======================
cat > task_field_map.yaml << EOF
image: task_field_map.pgm
resolution: 0.01
origin: [0.0, 6.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF

# ====================== 启动并激活map_server ======================
# 启动地图服务器（后台运行）
ros2 run nav2_map_server map_server --ros-args \
-p yaml_filename:="$NAV2_DIR/maps/task_field_map.yaml" &
MAP_PID=$!
sleep 8  # 延长等待，确保初始化完成

# 分步激活（按ROS2生命周期规则）
echo -e "\033[32m激活地图服务器...\033[0m"
ros2 lifecycle set /map_server configure > /dev/null 2>&1
sleep 3
ros2 lifecycle set /map_server activate > /dev/null 2>&1

# 验证激活状态
if ros2 lifecycle get /map_server | grep -q "active"; then
    echo -e "\033[32m✅ 标准地图生成+激活成功！\033[0m"
    echo -e "\033[33m👉 启动RViz2：ros2 launch nav2_bringup rviz_launch.py\033[0m"
else
    echo -e "\033[31m❌ map_server激活失败，手动执行：\033[0m"
    echo "ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate"
fi
