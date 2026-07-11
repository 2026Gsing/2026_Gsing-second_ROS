# Hyper Run — 仿生足式机器人竞赛部署手册

整合建图与比赛定位导航全流程，适用于 ROBOCON 2026 仿生足式机器人挑战赛（任务赛）。

---

## 系统概述

```
map_scan.md（建图）            competition.md（比赛导航）
     │                                │
     ├─ FAST-LIO2 SLAM 建图           ├─ FAST-LIO2 + ICP 全局定位
     ├─ PCD → PGM 栅格转换            ├─ Nav2 路径规划
     └─ 点云可视化                     ├─ 串口桥 → STM32
                                       └─ 视觉自动任务（YOLO）
```

**流程时序：** 首次部署或场地变化时，先执行**建图流程**生成地图，之后每次比赛运行**导航定位流程**。

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
ros2 launch pcd2pgm pcd2pgm_launch.py
```

## 工具

```bash
# 查看 PCD 点云
pcl_viewer /home/hyper/program/2026_Gsing-second_ROS/map/scans.pcd
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
  map:=/home/hyper/program/2026_Gsing-second_ROS/map/scans.pcd \
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
python3 /home/hyper/program/2026_Gsing-second_ROS/py/vision_auto_task_node.py
```

## 终端 6（可选）— YOLO 检测

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/vision
source /opt/ros/jazzy/setup.bash
python3 src/predict.py --weights weights/task3.pt --source 1 --draw-roi
```

---

# 改动记录（2026-07-05）

## TF 架构重构：统一使用 camera_init

**删除冗余的 `odom` 帧**，改为 `camera_init` 作为导航参考帧。

```
旧 TF 树（4 跳，含冗余 odom）：
  map → camera_init → odom → base_link → body

新 TF 树（2 跳，camera_init 为中心）：
  map ──(ICP)──→ camera_init ──(里程计)──→ base_link → body
  └──(tf_static odom→map 用于控制器坐标转换)
```

| 改动文件 | 修改内容 |
|---|---|
| `odometry_to_tf.cpp:26` | `frame_id: "odom"` → `"camera_init"`（需 rebuild） |
| `transform_fusion.py` | 所有 `odom` → `camera_init`；`odom→map` 静态 TF 保留用于坐标转换 |
| `nav2_fastlio_static_map_params.yaml` | `odom_frame: odom` → `camera_init`（3 处） |
| `nav2_fastlio_static_map.launch.py` | 删除冗余的 `map→camera_init` 静态 TF（防 TF 环路） |

## 控制器更换：DWB → RegulatedPurePursuit

DWB 轨迹评分机制始终不输出 `vx>0`（经诊断不是权重或参数问题）。

**已更换为** `nav2_regulated_pure_pursuit_controller`，使用纯追踪算法（前视点 + 圆弧跟踪），配置：

| 参数 | 值 | 说明 |
|---|---|---|
| `desired_linear_vel` | 0.3 m/s | 目标前进速度 |
| `min_linear_vel` | 0.05 m/s | 最低前进速度（防止 stuck） |
| `lookahead_dist` | 0.3 m | 前视距离 |
| `use_rotate_to_heading` | false | 不先旋转，边走边转 |
| `use_collision_detection` | false | 暂不启用碰撞检测 |

## 定位位姿：纯 ICP（无里程计漂移）

`/localization` 话题的位姿现在**直接用 ICP 定位结果**（`map→camera_init`），不再乘以 FAST-LIO2 里程计。速度（twist）仍来自 FAST-LIO2 保证平滑控制。

## 修复：机器人足迹超出地图左边界

- **根因**: 地图宽仅 1.7m（34px×0.05m），`robot_radius=0.20` 使机器人左侧足迹超出地图边界
- **修改**: `robot_radius` 0.20 → **0.15**（global + local costmap）
- **验证**: `python3 -c "print((0.11-(-0.072)) >= 0.15)"` → 应输出 `True`

## 修复：`/localization.child_frame_id`

- **文件**: `transform_fusion.py:207`
- **修改**: `"body"` → `"base_link"`（Nav2 期望 `base_link`）
- **注意**: Python 文件 symlink-install，重启进程即可生效
