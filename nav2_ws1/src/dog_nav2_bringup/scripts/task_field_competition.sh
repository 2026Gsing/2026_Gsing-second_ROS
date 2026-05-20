#!/bin/bash
# 文件名：task_field_competition.sh
# 功能：一键生成符合任务赛规则的6m×4m单场地栅格地图 + 启动Nav2
# 适配规则：物资箱存放/归位区、减速带可通行、智力题显示区、启动区
# 分辨率：0.01m/像素（1像素=10mm）

# ====================== 第一步：环境准备 ======================
echo -e "\033[32m【1/7】环境准备中...\033[0m"
# 安装必备依赖
if ! command -v convert &> /dev/null; then
    echo -e "\033[33m⚠️  未安装imagemagick，开始安装...\033[0m"
    sudo apt update && sudo apt install -y imagemagick ros-jazzy-nav2-map-server ros-jazzy-nav2-bringup ros-jazzy-rviz2 > /dev/null 2>&1
fi

# 终止旧进程
pkill -f "nav2" > /dev/null 2>&1
pkill -f "map_server" > /dev/null 2>&1

# 创建标准化目录
mkdir -p ~/program/ROS/nav2_ws1/src/dog_nav2_bringup/{maps,config,scripts}

# ====================== 第二步：参数定义（贴合赛事规则） ======================
# 物理尺寸（mm）→ 像素（1像素=10mm）
MAP_WIDTH_PX=400    # 4000mm → 400像素
MAP_HEIGHT_PX=600   # 6000mm → 600像素
RESOLUTION=0.01     # 1像素=0.01m=10mm

# 归位区参数（400mm×400mm=40×40像素，间距400mm=40像素，离墙≥600mm=60像素）
RETURN_ZONE_SIZE=40
RETURN_ZONE_GAP=40
RETURN_ZONE_OFFSET=60  # 离墙60像素（600mm）

# 存放区参数（250mm×250mm=25×25像素，间距600mm=60像素）
STORE_ZONE_SIZE=25
STORE_ZONE_GAP=60

# ====================== 第三步：生成精准栅格地图 (.pgm) ======================
echo -e "\033[32m【2/7】生成符合赛事规则的6m×4m单场地地图...\033[0m"
cd ~/program/ROS/nav2_ws1/src/dog_nav2_bringup/maps/
rm -f task_field_map.pgm task_field_map.yaml

# 1. 生成400×600全白基础图（可通行区域）
convert -size ${MAP_WIDTH_PX}x${MAP_HEIGHT_PX} xc:white task_field_map.pgm

# 2. 标记外围围墙（黑色，墙宽200mm=20像素）
convert task_field_map.pgm -fill black \
-draw "rectangle 0,0 $((MAP_WIDTH_PX-1)),19" \
-draw "rectangle 0,$((MAP_HEIGHT_PX-20)) $((MAP_WIDTH_PX-1)),$((MAP_HEIGHT_PX-1))" \
-draw "rectangle 0,0 19,$((MAP_HEIGHT_PX-1))" \
-draw "rectangle $((MAP_WIDTH_PX-20)),0 $((MAP_WIDTH_PX-1)),$((MAP_HEIGHT_PX-1))" \
task_field_map.pgm

# 3. 标记物资箱存放区（8个，25×25像素，黑色障碍物）
convert task_field_map.pgm -fill black \
-draw "rectangle $((50-STORE_ZONE_SIZE/2)),$((100-STORE_ZONE_SIZE/2)) $((50+STORE_ZONE_SIZE/2-1)),$((100+STORE_ZONE_SIZE/2-1))" \
-draw "rectangle $((110-STORE_ZONE_SIZE/2)),$((100-STORE_ZONE_SIZE/2)) $((110+STORE_ZONE_SIZE/2-1)),$((100+STORE_ZONE_SIZE/2-1))" \
-draw "rectangle $((170-STORE_ZONE_SIZE/2)),$((100-STORE_ZONE_SIZE/2)) $((170+STORE_ZONE_SIZE/2-1)),$((100+STORE_ZONE_SIZE/2-1))" \
-draw "rectangle $((230-STORE_ZONE_SIZE/2)),$((100-STORE_ZONE_SIZE/2)) $((230+STORE_ZONE_SIZE/2-1)),$((100+STORE_ZONE_SIZE/2-1))" \
-draw "rectangle $((50-STORE_ZONE_SIZE/2)),$((160-STORE_ZONE_SIZE/2)) $((50+STORE_ZONE_SIZE/2-1)),$((160+STORE_ZONE_SIZE/2-1))" \
-draw "rectangle $((110-STORE_ZONE_SIZE/2)),$((160-STORE_ZONE_SIZE/2)) $((110+STORE_ZONE_SIZE/2-1)),$((160+STORE_ZONE_SIZE/2-1))" \
-draw "rectangle $((170-STORE_ZONE_SIZE/2)),$((160-STORE_ZONE_SIZE/2)) $((170+STORE_ZONE_SIZE/2-1)),$((160+STORE_ZONE_SIZE/2-1))" \
-draw "rectangle $((230-STORE_ZONE_SIZE/2)),$((160-STORE_ZONE_SIZE/2)) $((230+STORE_ZONE_SIZE/2-1)),$((160+STORE_ZONE_SIZE/2-1))" \
task_field_map.pgm

# 4. 标记物资箱归位区（4个，40×40像素，灰色标注）
convert task_field_map.pgm -fill "#888888" \
-draw "rectangle $RETURN_ZONE_OFFSET,$((MAP_HEIGHT_PX-100)) $((RETURN_ZONE_OFFSET+RETURN_ZONE_SIZE-1)),$((MAP_HEIGHT_PX-100+RETURN_ZONE_SIZE-1))" \
-draw "rectangle $((RETURN_ZONE_OFFSET+RETURN_ZONE_SIZE+RETURN_ZONE_GAP)),$((MAP_HEIGHT_PX-100)) $((RETURN_ZONE_OFFSET+RETURN_ZONE_SIZE+RETURN_ZONE_GAP+RETURN_ZONE_SIZE-1)),$((MAP_HEIGHT_PX-100+RETURN_ZONE_SIZE-1))" \
-draw "rectangle $((RETURN_ZONE_OFFSET+2*(RETURN_ZONE_SIZE+RETURN_ZONE_GAP))),$((MAP_HEIGHT_PX-100)) $((RETURN_ZONE_OFFSET+2*(RETURN_ZONE_SIZE+RETURN_ZONE_GAP)+RETURN_ZONE_SIZE-1)),$((MAP_HEIGHT_PX-100+RETURN_ZONE_SIZE-1))" \
-draw "rectangle $((RETURN_ZONE_OFFSET+3*(RETURN_ZONE_SIZE+RETURN_ZONE_GAP))),$((MAP_HEIGHT_PX-100)) $((RETURN_ZONE_OFFSET+3*(RETURN_ZONE_SIZE+RETURN_ZONE_GAP)+RETURN_ZONE_SIZE-1)),$((MAP_HEIGHT_PX-100+RETURN_ZONE_SIZE-1))" \
task_field_map.pgm

# 5. 标记智力题显示区（浅灰色）
convert task_field_map.pgm -fill "#DDDDDD" \
-draw "rectangle $((MAP_WIDTH_PX/2-80)),$((MAP_HEIGHT_PX-80)) $((MAP_WIDTH_PX/2+79)),$((MAP_HEIGHT_PX-80+89))" \
task_field_map.pgm

# 6. 标记启动区（浅蓝色）
convert task_field_map.pgm -fill "#ADD8E6" \
-draw "rectangle $((MAP_WIDTH_PX/2-40)),5 $((MAP_WIDTH_PX/2+39)),84" \
task_field_map.pgm

# ====================== 第四步：生成YAML配置文件 ======================
echo -e "\033[32m【3/7】生成地图配置文件...\033[0m"
cat > task_field_map.yaml << EOF_MAP
image: task_field_map.pgm
resolution: ${RESOLUTION}
origin: [0.0, 6.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF_MAP

# 验证地图文件
if [ -s task_field_map.pgm ] && [ -s task_field_map.yaml ]; then
  echo -e "\033[32m✅ 赛事地图文件生成成功！\033[0m"
else
  echo -e "\033[31m❌ 地图文件生成失败，请检查imagemagick是否安装！\033[0m"
  exit 1
fi

# ====================== 第五步：生成Nav2配置 ======================
echo -e "\033[32m【4/7】生成Nav2赛事适配配置...\033[0m"
cd ~/program/ROS/nav2_ws1/src/dog_nav2_bringup/config/

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
      desired_linear_vel: 0.2
      min_linear_vel: 0.1
      max_linear_accel: 0.1
      lookahead_dist: 0.3

local_costmap:
  ros__parameters:
    use_sim_time: False
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_footprint
    robot_radius: 0.2
    inflation_radius: 0.3
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
    robot_radius: 0.2
    inflation_radius: 0.3
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

# ====================== 第六步：启动地图服务器 ======================
echo -e "\033[32m【5/7】启动地图服务器...\033[0m"
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash

# 启动地图服务器
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="~/program/ROS/nav2_ws1/src/dog_nav2_bringup/maps/task_field_map.yaml" &
sleep 5

# 激活地图服务器
echo -e "\033[32m【6/7】激活地图服务器...\033[0m"
ros2 lifecycle set /map_server configure > /dev/null 2>&1
sleep 2
if ros2 lifecycle set /map_server activate > /dev/null 2>&1; then
  echo -e "\033[32m✅ 地图服务器激活成功！\033[0m"
else
  echo -e "\033[31m❌ 地图服务器激活失败，请手动执行：\033[0m"
  echo -e "\033[33m   ros2 lifecycle set /map_server configure\033[0m"
  echo -e "\033[33m   ros2 lifecycle set /map_server activate\033[0m"
fi

# ====================== 第七步：启动Nav2并验证 ======================
echo -e "\033[32m【7/7】启动Nav2赛事核心...\033[0m"
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  autostart:=True \
  params_file:=~/program/ROS/nav2_ws1/src/dog_nav2_bringup/config/nav2_task_params.yaml \
  map:=~/program/ROS/nav2_ws1/src/dog_nav2_bringup/maps/task_field_map.yaml &
sleep 5

# 验证地图加载
if ros2 topic list | grep -q "/map"; then
  echo -e "\033[32m✅ /map话题已发布！\033[0m"
  echo -e "\033[32m🎉 任务赛地图生成+Nav2启动完成！\033[0m"
  echo -e "\033[33m👉 启动RViz2验证：ros2 launch nav2_bringup rviz_launch.py\033[0m"
else
  echo -e "\033[31m❌ /map话题未发布，地图加载失败！\033[0m"
  exit 1
fi
