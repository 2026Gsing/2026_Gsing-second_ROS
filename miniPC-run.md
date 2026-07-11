# 运行指南（本机适配版）

> 基于 `competition.md` + `map_scan.md`，路径适配 `gsing@gsing:~/2026Gsing`，网口 `enp4s0`。

---

## 1. 首次环境准备

```bash
# ROS2 基础
source /opt/ros/jazzy/setup.bash

# CycloneDDS（必须，所有终端都要）
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
ros2 daemon stop && ros2 daemon start

# 可视化工具
sudo apt install -y ros-jazzy-rviz2 pcl-tools

# 串口权限
sudo usermod -aG dialout $USER
# 重新登录生效，或临时：sudo chmod 666 /dev/ttyACM0
```

---

## 2. 构建

### 2.1 FAST-LIO2 工作空间

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh
source install/setup.bash
```

### 2.2 Nav2 工作空间

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/nav2_ws1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

⚠️ **colcon build 后需同步 Nav2 参数文件**（install/ 下是副本非 symlink）：

```bash
cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
   install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

---

## 3. 网卡配置（首次或重启后）

LiDAR 需通过以太网口连接。本机网口名为 **`enp4s0`**（取代码中的 `enp129s0`）：

```bash
sudo nmcli device set enp4s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp4s0
```

验证：
```bash
ip addr show enp4s0 | grep 192.168.1.2
```

---

## 4. 扫描建图流程

### 终端 1 — LiDAR 驱动

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

### 终端 2 — FAST-LIO2 SLAM 建图（回车保存并退出）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
bash auto_map_save.sh
```

### 终端 2b（可选）— RViz 可视化

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2/install/setup.bash
rviz2 -d /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2/src/fast_lio_config.rviz
```

### 终端 3 — PCD → PGM 栅格地图转换

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

### 工具

```bash
# 查看 PCD 点云
pcl_viewer /home/gsing/2026Gsing/2026_Gsing-second_ROS/map/map.pcd
```

> ⚠️ **注意：** 当前 `map/` 目录下缺少 `map.pcd`，只有栅格地图（`pgm_map.pgm/.yaml`）。需先运行建图流程生成。也可以先直接用已有栅格地图跑定位。

---

## 5. 比赛定位导航流程

> 前提：已有建好的 `map/map.pcd`（点云地图）和 `map/pgm_map.yaml`（栅格地图）。

### 终端 1 — LiDAR 驱动

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

### 终端 2 — 全局定位 ICP（内置 FAST-LIO2 + transform_fusion）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH"
export PYTHONPATH=$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages
ros2 launch fast_lio_localization 1.launch.py \
  map:=/home/gsing/2026Gsing/2026_Gsing-second_ROS/map/map.pcd \
  config_file:=unilidar_l2.yaml rviz:=true \
  map_voxel_size:=0.01 scan_voxel_size:=0.03 \
  freq_localization:=2.0 localization_threshold:=0.9
```

在打开的 RViz 中点击 **"2D Pose Estimate"** 初始化 ICP 定位。

### 终端 3 — Nav2 导航

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/nav2_ws1
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=/home/gsing/2026Gsing/2026_Gsing-second_ROS/map/pgm_map.yaml
```

### 终端 4 — 串口桥（/cmd_vel → STM32）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/nav2_ws1
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 \
  active_state:=1 idle_state:=0
```

### 终端 5（可选）— 视觉自动任务

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/gsing/2026Gsing/2026_Gsing-second_ROS/nav2_ws1/install/setup.bash
python3 /home/gsing/2026Gsing/2026_Gsing-second_ROS/py/vision_auto_task_node.py
```

### 终端 6（可选）— YOLO 检测

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS/vision
source /opt/ros/jazzy/setup.bash
python3 src/predict.py --weights weights/task3.pt --source 1 --draw-roi
```

---

## 6. 一键启动（任务赛）

```bash
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS
bash run_auto_task.sh
```

---

## 7. 常用工具

```bash
# 底盘运动测试
source /opt/ros/jazzy/setup.bash
python3 /home/gsing/2026Gsing/2026_Gsing-second_ROS/py/test_move.py

# 串口监听
python3 /home/gsing/2026Gsing/2026_Gsing-second_ROS/py/listen_serial.py

# 实时位姿
python3 /home/gsing/2026Gsing/2026_Gsing-second_ROS/py/fastlio_pose.py

# LiDAR 3D 立方体检测
python3 /home/gsing/2026Gsing/2026_Gsing-second_ROS/py/cube_detector.py

# 到达检测（单测）
python3 /home/gsing/2026Gsing/2026_Gsing-second_ROS/py/arrival_detector.py
```

---

## 8. 系统架构

### TF 树

```
map ──(ICP)──→ camera_init ──(里程计)──→ base_link → body
└──(tf_static odom→map 用于控制器坐标转换)
```

### 控制器

使用 `nav2_regulated_pure_pursuit_controller`（纯追踪算法），关键参数：

| 参数 | 值 | 说明 |
|---|---|---|
| `desired_linear_vel` | 0.3 m/s | 目标前进速度 |
| `min_linear_vel` | 0.05 m/s | 最低前进速度 |
| `lookahead_dist` | 0.3 m | 前视距离 |
| `robot_radius` | 0.15 | 机器人半径 |

### 速度仲裁

`cmd_vel_chassis_serial.py` 底盘速度优先级：
1. `/vision_cmd_vel` 500ms 内有数据 → 视觉速度（精细对位）
2. 否则 `/cmd_vel` (Nav2) 80ms 内有数据 → Nav2 速度（长距导航）
3. 否则 → 停止
