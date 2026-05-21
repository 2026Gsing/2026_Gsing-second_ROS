#!/bin/bash
# 修正版：任务赛单场地地图一键生成+Nav2启动脚本
# 解决convert命令换行和依赖安装问题
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ====================== 第一步：环境准备 ======================
echo -e "\033[32m【1/6】环境准备中...\033[0m"
# 手动确认imagemagick已安装
if ! command -v convert &> /dev/null; then
    echo -e "\033[31m❌ 未安装imagemagick，手动安装中...\033[0m"
    sudo apt update && sudo apt install -y imagemagick ros-jazzy-nav2-map-server ros-jazzy-nav2-bringup ros-jazzy-rviz2
fi

# 终止旧进程
pkill -f "nav2" > /dev/null 2>&1
pkill -f "map_server" > /dev/null 2>&1

# 创建目录
mkdir -p "$NAV2_DIR"/{maps,config,scripts}

# ====================== 第二步：生成精准栅格地图 ======================
echo -e "\033[32m【2/6】生成6m×4m单场地栅格地图...\033[0m"
cd "$NAV2_DIR/maps/"
rm -f task_field_map.pgm task_field_map.yaml

# 生成基础图（400×600像素）
convert -size 400x600 xc:white task_field_map.pgm

# 修正convert命令格式（单行完成，避免换行错误）
convert task_field_map.pgm -fill black \
-draw "rectangle 0,0 399,19" \
-draw "rectangle 0,580 399,599" \
-draw "rectangle 0,0 19,599" \
-draw "rectangle 380,0 399,599" \
-draw "rectangle 0,240 399,260" \
-draw "rectangle 37,87 62,112" \
-draw "rectangle 137,87 162,112" \
-draw "rectangle 237,87 262,112" \
-draw "rectangle 337,87 362,112" \
-draw "rectangle 37,37 62,62" \
-draw "rectangle 137,37 162,62" \
-draw "rectangle 237,37 262,62" \
-draw "rectangle 337,37 362,62" \
task_field_map.pgm

# 生成yaml文件
cat > task_field_map.yaml << EOF_MAP
image: task_field_map.pgm
resolution: 0.01
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF_MAP

# 验证地图文件
if [ -s task_field_map.pgm ] && [ -s task_field_map.yaml ]; then
  echo -e "\033[32m✅ 地图文件生成成功！\033[0m"
else
  echo -e "\033[31m❌ 地图文件生成失败！\033[0m"
  exit 1
fi

# ====================== 第三步：生成Nav2配置 ======================
echo -e "\033[32m【3/6】生成Nav2适配配置文件...\033[0m"
cd "$NAV2_DIR/config/"

cat > nav2_task_params.yaml << EOF_CONFIG
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom"
    laser_model_type: "likelihood_field"
    max_particles: 2000
    min_particles: 500
    tf_broadcast: true

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_footprint
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_wait_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.3
      min_linear_vel: 0.1
      max_linear_accel: 0.2
      lookahead_dist: 0.6

local_costmap:
  ros__parameters:
    use_sim_time: False
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_footprint
    robot_radius: 0.35
    inflation_radius: 0.6
    rolling_window: true
    width: 3.0
    height: 3.0
    resolution: 0.05
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

global_costmap:
  ros__parameters:
    use_sim_time: False
    update_frequency: 1.0
    publish_frequency: 1.0
    global_frame: map
    robot_base_frame: base_footprint
    robot_radius: 0.35
    inflation_radius: 0.6
    static_map: true
    resolution: 0.05
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

planner_server:
  ros__parameters:
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      use_astar: true
      allow_unknown: true

lifecycle_manager:
  ros__parameters:
    use_sim_time: False
    autostart: true
    node_names:
      - controller_server
      - planner_server
      - bt_navigator
      - map_server
EOF_CONFIG

# ====================== 第四步：启动地图服务器 ======================
echo -e "\033[32m【4/6】启动地图服务器...\033[0m"
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash

# 先等待前一个map_server退出
sleep 2
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$NAV2_DIR/maps/task_field_map.yaml" &
sleep 4

# 重试激活地图服务器
MAX_RETRY=3
RETRY=0
while [ $RETRY -lt $MAX_RETRY ]; do
  if ros2 lifecycle set /map_server activate > /dev/null 2>&1; then
    echo -e "\033[32m✅ 地图服务器激活成功！\033[0m"
    break
  fi
  RETRY=$((RETRY+1))
  echo -e "\033[33m⚠️  地图服务器激活失败，重试第 $RETRY 次...\033[0m"
  sleep 2
done

if [ $RETRY -eq $MAX_RETRY ]; then
  echo -e "\033[31m❌ 地图服务器激活失败！\033[0m"
  exit 1
fi

# ====================== 第五步：启动Nav2 ======================
echo -e "\033[32m【5/6】启动Nav2核心...\033[0m"
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  autostart:=True \
  params_file:="$NAV2_DIR/config/nav2_task_params.yaml" \
  map:="$NAV2_DIR/maps/task_field_map.yaml" &
sleep 5

# ====================== 第六步：验证 ======================
echo -e "\033[32m【6/6】验证地图加载状态...\033[0m"
if ros2 topic list | grep -q "/map"; then
  echo -e "\033[32m✅ /map话题已发布！\033[0m"
  echo -e "\033[32m🎉 任务赛单场地地图生成+Nav2启动完成！\033[0m"
  echo -e "\033[33m👉 启动RViz2验证：ros2 launch nav2_bringup rviz_launch.py\033[0m"
else
  echo -e "\033[31m❌ /map话题未发布，地图加载失败！\033[0m"
  exit 1
fi
