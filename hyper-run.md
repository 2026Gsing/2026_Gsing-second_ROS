# Hyper Run — 仿生足式机器人竞赛部署手册

整合建图与比赛定位导航全流程，适用于 ROBOCON 2026 仿生足式机器人挑战赛（任务赛）。

---

## 首次准备

```bash
# 安装 CycloneDDS（仅一次）
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
ros2 daemon stop && ros2 daemon start

# 构建 FAST-LIO2 全部组件
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh
source install/setup.bash

# 构建 Nav2
cd /home/hyper/program/2026_Gsing-second_ROS/nav2_ws1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
source install/setup.bash

# ⚠️ 重要：Nav2 参数文件（install/ 下是副本非 symlink）
# colcon build 后需手动同步参数文件：
cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
   install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

---

## 网卡配置（首次或重启后）

```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

---

# 第一部分：建图流程

在场地首次部署或地图过期时执行。

## 终端 1 — LiDAR 驱动

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## 终端 2 — FAST-LIO2 SLAM 建图（回车保存并退出）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
bash auto_map_save.sh
```

## 终端 2b（可选）— RViz 可视化

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/install/setup.bash
rviz2 -d /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/src/fast_lio_config.rviz
```

## 终端 3 — PCD → PGM 栅格地图转换

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch pcd2pgm pcd2pgm_launch.py \
  pcd_file:=/home/hyper/program/2026_Gsing-second_ROS/map/PCD13.pcd
# 输出: map/map.yaml + map/map.pgm（文件名与 PCD 相同，目录与 PCD 相同）
```

## 工具

```bash
# 查看 PCD 点云
pcl_viewer /home/hyper/program/2026_Gsing-second_ROS/map/PCD13.pcd
```

---

# 第二部分：比赛导航定位

建图完成后，每次比赛按以下流程启动。

## 终端 1 — LiDAR 驱动

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## 终端 2 — 全局定位 ICP（内置 FAST-LIO2 + transform_fusion）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH"
export PYTHONPATH=$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages
ros2 launch fast_lio_localization 1.launch.py \
  map:=/home/hyper/program/2026_Gsing-second_ROS/map/map.pcd \
  config_file:=unilidar_l2.yaml rviz:=true \
  map_voxel_size:=0.01 scan_voxel_size:=0.03 \
  freq_localization:=2.0 localization_threshold:=0.9
```

在打开的 RViz 中点击 **"2D Pose Estimate"** 初始化 ICP 定位。

## 终端 3 — Nav2 导航

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/nav2_ws1
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=/home/hyper/program/2026_Gsing-second_ROS/map/pgm_map.yaml
```

## 终端 4 — 串口桥（/cmd_vel → STM32）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/nav2_ws1
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 \
  active_state:=1 idle_state:=0
```

## 终端 5（可选）— 视觉自动任务

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/hyper/program/2026_Gsing-second_ROS/nav2_ws1/install/setup.bash
python3 /home/hyper/program/2026_Gsing-second_ROS/py/control/auto_task.py
```

---