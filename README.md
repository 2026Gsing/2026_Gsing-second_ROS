# 2026 Gsing 二队 — ROS2 Jazzy 导航 + 视觉自动任务系统

基于 **ROS2 Jazzy** 的机器人导航与视觉自动任务系统，集成 **FAST-LIO2 SLAM**（建图与定位）、**Nav2**（路径规划与运动控制）、**YOLO 视觉**（物资识别 + 数学题识别）和 **STM32 串口通信**（底盘 + 机械臂），用于竞赛任务场景。

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        FAST-LIO2 工作空间                         │
│  ┌──────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │ Unitree LiDAR │  │  FAST-LIO2       │  │ fast_lio_         │  │
│  │ Driver        │ → │  Mapping / LIO   │ → │ localization     │  │
│  │ (点云)        │  │  (建图/里程计)    │  │  (全局重定位)     │  │
│  └──────────────┘  └──────┬───────────┘  └────────┬─────────┘  │
│                           │ /Odometry              │ /map_to_odom│
│                           ▼                        ▼             │
│                    ┌──────────────────┐  ┌──────────────────┐  │
│                    │ transform_fusion  │  │ publish_initial  │  │
│                    │ (TF 变换融合)     │  │ _pose (初始化)    │  │
│                    └────────┬─────────┘  └──────────────────┘  │
│                             │ /localization                     │
└─────────────────────────────┼───────────────────────────────────┘
                              │
┌─────────────────────────────┼───────────────────────────────────┐
│                  Nav2 工作空间                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  map_server → global_costmap (static_layer)              │   │
│  │       ┌──────────────────────────────────────┐          │   │
│  │       │ planner_server (Navfn/A* 全局规划)   │          │   │
│  │       │ controller_server (DWB 局部规划)     │          │   │
│  │       │ bt_navigator (行为树导航)             │          │   │
│  │       └──────────────┬───────────────────────┘          │   │
│  └──────────────────────┼───────────────────────────────────┘   │
│                         │ /cmd_vel                                │
│  ┌──────────────────────┼───────────────────────────────────┐   │
│  │  chassis_serial_bridge (cmd_vel → STM32 串口 0x10)       │   │
│  │  + 新增: /vision_cmd_vel 优先, /vision/auto_cmd → 0x15  │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────┼───────────────────────────────────┐
│              视觉自动任务模块 (新)                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  vision_auto_task_node.py  (状态机)                       │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────────┐ │   │
│  │  │ YOLO 检测│ │ Nav2 Goal│ │到达检测  │ │ 串口 0x15   │ │   │
│  │  │ task.pt  │ │ Navigator│ │/localizat│ │ AUTO_CMD    │ │   │
│  │  │ math.pt  │ │          │ │          │ │             │ │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └─────────────┘ │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 目录结构

```
2026_Gsing-second_ROS/
├── fastlio2_v2/                          # 主工作空间：FAST-LIO2 SLAM + 定位
│   └── src/
│       ├── unilidar_fastlio_ros2-ros2/   # FAST-LIO2 核心（建图 + 里程计）
│       ├── fast_lio_localization/         # 全局重定位（ICP 配准）
│       │   ├── global_localization.py     #   ICP 配准节点
│       │   ├── transform_fusion.py        #   TF 变换融合节点
│       │   └── publish_initial_pose.py    #   初始位姿发布工具
│       ├── unitree_lidar_ros2/            # 宇树科技激光雷达驱动
│       ├── my_odom_tf_pkg/                # 里程计 → TF 桥接
│       └── pcd2pgm/                       # PCD 点云 → PGM 栅格地图转换
│
├── nav2_ws1/                              # Nav2 导航工作空间
│   └── src/dog_nav2_bringup/
│       ├── launch/
│       │   ├── nav2_fastlio_bringup.launch.py         # 动态建图模式（无地图）
│       │   ├── nav2_fastlio_static_map.launch.py      # 静态地图模式（竞赛主用）
│       │   └── chassis_serial_bridge.launch.py        # 底盘串口桥接
│       ├── params/
│       │   ├── nav2_fastlio_params.yaml               # 动态建图参数
│       │   └── nav2_fastlio_static_map_params.yaml    # 静态地图参数
│       ├── scripts/
│       │   ├── cmd_vel_chassis_serial.py      # /cmd_vel → 串口转发
│       │   ├── goal_pose_to_nav2.py           # RViz 目标 → Nav2 Action
│       │   ├── costmap_to_grid.py             # Costmap 转发（RViz 显示）
│       │   ├── nav2_task_launch.py            # 竞赛任务启动（AMCL 版）
│       │   ├── send_chassis_test_serial.py    # 串口通信测试工具
│       │   ├── task_field_competition.sh      # 竞赛一站式启动脚本
│       │   ├── task_field_map_full.sh         # 完整地图生成脚本
│       │   ├── generate_standard_map.sh       # 标准地图生成
│       │   ├── generate_custom_map.sh         # 自定义地图生成
│       │   ├── start_nav2.sh                  # Nav2 启动（精简）
│       │   └── start_nav2_full.sh             # Nav2 启动（完整）
│       ├── maps/                              # 预存栅格地图
│       └── rviz/                              # RViz 配置
│
├── 2026_Gsing_Dog_simulation-main/        # Gazebo 仿真（四足狗）
│   └── simulation/src/scripts/
│       └── path_planner.py                 # A* 路径规划器（仿真用）
│
├── vision/                                 # YOLO 视觉模块（整合自 second-YOLO-tmp）
│   ├── src/
│   │   ├── predict.py                      # YOLO 检测主脚本（任务模型+槽位分配）
│   │   ├── slot_roi.py                     # ROI 槽位分配模块
│   │   └── MATH.PY                         # 数学符号识别
│   ├── config/
│   │   ├── slots_roi.json                  # ROI 槽位标定
│   │   ├── decision_state.json             # 数学决策结果 (IPC)
│   │   └── nav_target.json                 # 导航目标输出 (IPC)
│   └── weights/
│       ├── task3.pt                        # 4 类物资检测 (tool/device/food/remedy)
│       └── math12.pt                       # 数学符号检测
│
├── py/                                     # 独立 Python 脚本
│   ├── vision_auto_task_node.py            # 视觉自动任务状态机（核心新增）
│   ├── arrival_detector.py                 # 到达检测工具
│   ├── cube_detector.py                    # 3D OBB 立方体检测（雷达点云→DBSCAN→PCA）
│   ├── catch.py                            # 机械臂抓取控制（串口→STM32）
│   ├── camera_four_colors_config.json      # 摄像头四色 HSV 范围配置
│   ├── test_dog.py                         # OpenCV 颜色检测调参工具
│   ├── move.py                             # 底盘指令测试（前进/后退/转向）
│   ├── listen_serial.py                    # 串口监听（十六进制显示）
│   ├── pointcloud_saver.py                 # 点云保存（空格触发）
│   ├── folder_summary.py                   # 项目文件内容提取工具
│   ├── test_interactive.py                 # 交互测试脚本
│   └── config/competition_poses.yaml       # 物资箱/归位区预置坐标
│
├── run_auto_task.sh                        # 视觉自动任务一键启动（新增）
├── run.md                                  # 详细运行流程
└── README.md                               # 本文件
```

---

## 环境要求

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 24.04 |
| ROS2 | Jazzy |
| Python | 3.12 |
| 激光雷达 | Unitree LiDAR (L2) |
| 底盘 | STM32 串口控制 |
| 建图 | FAST-LIO2 |
| 导航 | Nav2 (Planner + Controller + BT) |
| 依赖包 | nav2-bringup, nav2-msgs |
| Python 依赖 | open3d, tf_transformations, ultralytics, pyserial |
| 视觉 | YOLOv8/YOLO11 (ultralytics), OpenCV |

---

## 完整工作流程

### 导航模式选择

本项目支持 **两种导航模式**，分别对应不同的定位方案：

| 模式 | 定位方案 | 地图来源 | 启动文件 | 适用场景 |
|------|---------|---------|---------|---------|
| **静态地图模式** ⭐ | FAST-LIO2 ICP 全局重定位 | 预存 PGM 地图 | `nav2_fastlio_static_map.launch.py` | 竞赛主用 |
| **动态建图模式** | FAST-LIO2 实时里程计 | 实时构建 | `nav2_fastlio_bringup.launch.py` | 探索建图 |

### 步骤 0：配置网卡（雷达通信）

```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

### 步骤 1：编译工作空间

```bash
# 安装系统依赖（首次）
sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs
pip install open3d tf_transformations

# 克隆 pcd2pgm（首次）
cd fastlio2_v2/src
git clone https://github.com/liuscn/pcd2pgm.git
cd ../..

# 设置脚本执行权限（首次）
find ../nav2_ws1/src/dog_nav2_bringup/scripts -name "*.py" -exec chmod +x {} +

# 编译 FAST-LIO2 工作空间
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization

# 修复 am ent_python 包的索引
bash src/fast_lio_localization/scripts/hook_fix.sh

source install/setup.bash

# 编译 Nav2 工作空间
cd ../nav2_ws1
rm -rf build/ install/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 步骤 2：启动雷达驱动

```bash
cd fastlio2_v2
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

### 步骤 3：启动 FAST-LIO2 建图（带过滤）

使用 `auto_map_save.sh` 一键启动，自动过滤狗身点云（距 LiDAR < 0.8m 和 x < 0.5 的点），按 Enter 保存 PCD 并退出。

```bash
cd fastlio2_v2
source install/setup.bash
bash auto_map_save.sh
```

### 步骤 4：生成 2D 栅格地图（首次建图时需要）

先确认 `src/pcd2pgm/config/pcd2pgm.yaml` 中 `pcd_file` 指向你的 PCD 文件。

```bash
cd fastlio2_v2
source install/setup.bash

ros2 launch pcd2pgm pcd2pgm_launch.py
```

生成结果（默认在 `/home/hyper/program/2026_Gsing-second_ROS/map/`）：
- `pgm_map.pgm`
- `pgm_map.yaml`

### 步骤 5：启动全局定位（fast_lio_localization）

```bash
cd fastlio2_v2
source install/setup.bash
ros2 launch fast_lio_localization 1.launch.py \
  map:=/home/hyper/program/2026_Gsing-second_ROS/map/map.pcd \
  config_file:=unilidar_l2.yaml \
  rviz:=true
```

在 RViz 中使用 **"2D Pose Estimate"** 工具点击地图上的大致位置初始化定位。

### 步骤 6：启动 Nav2 导航

```bash
cd nav2_ws1
source install/setup.bash

ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=src/dog_nav2_bringup/maps/scans_2d.yaml
```

### 步骤 7：启动底盘串口桥接

```bash
cd nav2_ws1
source install/setup.bash

ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel \
  send_rate_hz:=50.0 \
  active_state:=1 \
  idle_state:=0
```

---

## 数据流说明

### TF 坐标树

```
map → camera_init ── (static) ──→ odom ← FAST-LIO2 ──→ base_link
  ↓                                                      ↓
  (global_localization 发布)                     (static) ↓
  map→camera_init TF                            base_link → unilidar_lidar
```

关键点：
1. **map→camera_init**：由 `fast_lio_localization` 的 ICP 配准计算，通过 `transform_fusion.py` 广播
2. **camera_init→odom**：静态恒等变换（launch 文件中定义），用于满足 Nav2 的 TF 树要求
3. **odom→base_link**：由 FAST-LIO2 里程计发布
4. Nav2 的规划器使用 **map** 作为全局框架，控制器使用 **odom** 作为局部框架

### 话题数据流

```
=== 导航链路 ===
雷达驱动 (unitree_lidar_ros2)
  │ /unilidar/cloud (原始点云)
  ▼
FAST-LIO2 Mapping (fastlio_mapping)
  │ /Odometry (里程计: odom→base_link)
  │ /cloud_registered (配准后点云)
  ▼
fast_lio_localization (global_localization.py)
  │ /map_to_odom (ICP 配准: map→odom)
  ▼
transform_fusion (transform_fusion.py)
  │ TF: map→camera_init
  │ /localization (map 坐标系下的完整位姿)
  ▼
Nav2 (planner_server, controller_server, bt_navigator)
  │ /cmd_vel (速度指令)
  ▼
cmd_vel_chassis_serial (速度仲裁: vision_cmd_vel > cmd_vel)
  │ 串口 [0x55][0xAA][0x10][0x09][vx][wz][state][checksum]
  ▼
STM32 底盘 → 电机运动

=== 自动任务链路（新增） ===
YOLO (vision/src/predict.py)
  │ /vision/detected_objects (检测结果)
  │ /vision/math_result (数学题结果)
  ▼
vision_auto_task_node.py (视觉状态机)
  │ → Nav2 NavigateToPose (长距导航)
  │ → /vision_cmd_vel (精细对位，高于 Nav2 优先级)
  │ → /vision/auto_cmd (JSON) → cmd_vel_chassis_serial → 串口 0x15
  │ ← /localization (到达判断)
  ▼
STM32 auto_task 状态机 → arm_task 抓放
```

---

## 串口协议（ROS → STM32）

### 0x10 — 底盘速度控制 (FUNC_CHASSIS_MOVE)

帧格式（Nav2 或视觉 → STM32）：

| 偏移 | 字节 | 说明 |
|------|------|------|
| 0 | 0x55 | 帧头 1 |
| 1 | 0xAA | 帧头 2 |
| 2 | 0x10 | 功能码（底盘速度控制） |
| 3 | 0x0D | 载荷长度（13 字节：vx+vy+wz+state） |
| 4-7 | vx (float32 LE) | 线速度 (m/s)，正=前进 |
| 8-11 | vy (float32 LE) | 横向速度（差速/轮足暂不使用，发 0） |
| 12-15 | wz (float32 LE) | 角速度 (rad/s)，正=左转 |
| 16 | state (uint8) | 0=IDLE, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT |
| 17 | checksum (uint8) | 前面所有字节和 & 0xFF |

发送频率 ≥20Hz，STM32 100ms 未收到新帧自动停车。

### 0x15 — 自动任务事件 (FUNC_AUTO_TASK)

视觉状态机在关键节点发送：

| 偏移 | 字节 | 说明 |
|------|------|------|
| 0 | 0x55 | 帧头 1 |
| 1 | 0xAA | 帧头 2 |
| 2 | 0x15 | 功能码（自动任务事件） |
| 3 | 0x03 | 载荷长度（固定 3 字节） |
| 4 | cmd (uint8) | 命令：1=START, 2=ARRIVED_BOX, 3=PICK_DONE, 4=ARRIVED_ZONE, 5=PLACE_DONE, 6=NEXT, 7=FINISH, 8=ESTOP |
| 5 | target_id (uint8) | 物资箱编号 |
| 6 | zone_id (uint8) | 归位区编号 |
| 7 | checksum (uint8) | 前面所有字节和 & 0xFF |

由 `cmd_vel_chassis_serial.py` 订阅 `/vision/auto_cmd` (JSON) 自动转发。

---

## 立方体检测 + 机械臂抓取

完成导航到达目标位置后，系统使用雷达视觉进行立方体检测和机械臂抓取。

### 数据流

```
FAST-LIO2 雷达点云 (/unilidar/cloud)
  │
  ▼
cube_detector.py ──── DBSCAN 聚类 ──── 3D PCA ──── OBB 包围盒
  │                      │                │
  │  空间裁剪+去地面   聚类分离      主轴分析+边长计算
  │
  ▼  /detected_cube (Marker: 位置+边长+朝向)
  │
catch.py ──── 滑动窗口标准差（稳定检测）──── 串口 ──── STM32 ──── 机械臂
  │                                                     │
  │  坐标系变换 (雷达→机械臂)                     控制抓取动作
```

### cube_detector.py — 3D OBB 立方体检测

| 步骤 | 方法 | 参数 |
|------|------|------|
| 1. 空间裁剪 | 保留雷达前方 0~0.8m | `x_range=(−0.2, 0.8)` |
| 2. 去地面 | 移除 z 最低 5% 分位以下的点 | `z_keep_percent=2` |
| 3. 半径滤波 | BallTree 邻域点数过滤 | `radius=0.04, min_neighbors=5` |
| 4. 体素下采样 | 重心下采样 | `voxel_size=0.008` (8mm) |
| 5. DBSCAN 聚类 | 密度聚类分离不同物体 | `eps=0.04, min_samples=20` |
| 6. 3D PCA | 协方差矩阵特征分解 → OBB 主轴 | — |
| 7. 边长校验 | 均值边长在 25cm ± 5cm 内接受 | `edge_target=0.25, edge_tol=0.05` |
| 8. 发布 Marker | 选边长最接近 25cm 的立方体发布 | `/detected_cube` |

检测结果会以绿色半透明立方体 Marker 在 RViz 中可视化。

### catch.py — 机械臂抓取控制

**坐标变换**（雷达系 → 机械臂系）：
```
final_x = -radar_z + 0.06
final_y =  radar_y + 0.0
final_z = -radar_x - 0.15
```

**稳定检测**：滑动窗口标准差法
- 缓存最近 N 帧的立方体位置
- 计算 XY 方向的标准差
- 标准差 < 阈值视作稳定 → 发送坐标给机械臂
- 1Hz 心跳发送确保下位机不会丢包

**串口协议**（机械臂控制）：`[0x55][0xAA][0x12][12][x][y][z][checksum]`

### test_dog.py — 颜色检测调参工具

基于 OpenCV 的 HSV 颜色分类调参工具：
- 支持四种颜色分类：食品(绿)、工具(灰)、药品(红)、仪器(蓝)
- 滑条实时调节 HSV 阈值
- Top-1 占比直判算法
- 自动保存配置到 `camera_four_colors_config.json`

### 使用方式

```bash
source /opt/ros/jazzy/setup.bash

# 视觉自动任务
python3 py/vision_auto_task_node.py      # 状态机（终端输入 start 开始）

# YOLO 检测（独立运行）
cd vision && python3 src/predict.py --weights weights/task3.pt --source 1 --draw-roi

# 立方体检测（需要 FAST-LIO2 雷达点云）
python3 py/cube_detector.py

# 机械臂控制（需要 cube_detector 正在运行）
python3 py/catch.py

# 颜色检测调参（需要 USB 摄像头）
python3 py/test_dog.py

# 底盘运动测试
python3 py/move.py 1   # 0=待机 1=前进 2=后退 3=左转 4=右转 5=蹲下

# 监听串口数据
python3 py/listen_serial.py

# 串口通信测试
python3 nav2_ws1/src/dog_nav2_bringup/scripts/send_chassis_test_serial.py
```

---

## 视觉自动任务系统（新增）

### 视觉状态机

由 `py/vision_auto_task_node.py` 驱动，完整流程：

```
IDLE ──(输入 start)──→ SOLVE_TASK
  │                     │ YOLO 识别智力题 → mod4 → zone_sequence
  ▼                     ▼
SOLVE_TASK ──────────→ FIND_BOX
  │                     │ YOLO 检测物资箱类别
  ▼                     ▼
FIND_BOX ────────────→ NAV_BOX
  │                     │ 发 Nav2 goal → 监听 /localization → 到达
  ▼                     ▼
NAV_BOX ────(发 0x15 ARRIVED_BOX)──→ WAIT_PICK
  │                     │ 等待 ~5s → 发 PICK_DONE
  ▼                     ▼
WAIT_PICK ───(发 0x15 PICK_DONE)──→ NAV_ZONE
  │                     │ 发 Nav2 goal → 到归位区
  ▼                     ▼
NAV_ZONE ───(发 0x15 ARRIVED_ZONE)──→ WAIT_PLACE
  │                     │ 等待 ~5s → 发 PLACE_DONE
  ▼                     ▼
WAIT_PLACE ──(发 0x15 PLACE_DONE)──→ NEXT_OR_FINISH
                                       │
                          ┌────────────┴────────────┐
                          ▼                         ▼
                     还有箱 → FIND_BOX         完成 → IDLE
```

### 速度仲裁

底盘速度优先级：`/vision_cmd_vel` (500ms 超时) > `/cmd_vel` Nav2 (80ms 超时) > 停止

### 到达检测

订阅 `/localization` (Odometry, map 坐标系)，判断是否到达目标点。
配置参数：`arrival_pos_threshold=0.25m`, `arrival_angle_threshold=0.30rad`, `settle_frames=5`

### YOLO 检测

| 模型 | 用途 | 类别 |
|------|------|------|
| `vision/weights/task3.pt` | 物资分类 | tool, device, food, remedy |
| `vision/weights/math12.pt` | 数学符号 | 0-9, +, -, ×, ÷, (, ) |

## 竞赛任务工作流

### 手动启动（分步调试）

```
1. 上电 → 机器人启动
2. 配置网卡 → 雷达通信
3. 启动雷达驱动
4. 启动 FAST-LIO2 建图
5. 发布初始位姿（RViz "2D Pose Estimate"）
6. 启动 Nav2（静态地图模式）
7. 启动串口桥接（底盘控制）
8. 启动视觉状态机：python3 py/vision_auto_task_node.py
9. 在终端输入 start → 全自动执行
```

### 一键启动（竞赛模式）
```bash
# 启动 Nav2 + 串口桥 + 视觉状态机
bash run_auto_task.sh

# 另开终端，启动 YOLO 检测
cd vision && python3 src/predict.py --weights weights/task3.pt --source 1 --draw-roi
```

### 竞赛场地地图
```bash
# 生成 6m×4m 竞赛地图 + 启动 Nav2
./nav2_ws1/src/dog_nav2_bringup/scripts/task_field_competition.sh
```

### 竞赛场地地图说明

地图生成脚本在 `nav2_ws1/src/dog_nav2_bringup/scripts/` 下：

| 脚本 | 地图尺寸 | 特点 |
|------|---------|------|
| `task_field_competition.sh` | 6m × 4m (400×600px) | 完整竞赛流程 |
| `generate_standard_map.sh` | 6m × 4m (400×600px) | 仅生成地图+启动 |
| `generate_custom_map.sh` | 8m × 10m (800×1000px) | 可定制区域 |

---

## 常见问题

### 串口权限不足
```bash
sudo usermod -aG dialout $USER
# 重新登录生效；临时方案：
sudo chmod 666 /dev/ttyACM0
```

### pyserial 未安装
```bash
sudo apt install python3-serial
```

### ImageMagick 未安装（地图生成需要）
```bash
sudo apt install imagemagick
```

### `package 'fast_lio_localization' not found`

`fast_lio_localization` 是纯 Python 包，colcon 不会自动注册到 `AMENT_PREFIX_PATH`。编译后需运行：

```bash
cd fastlio2_v2
bash src/fast_lio_localization/scripts/hook_fix.sh
source install/setup.bash
```

### `executable not found on the libexec directory`

Python 脚本缺少执行权限。`--symlink-install` 模式下软链接继承源文件权限：

```bash
find . -name "*.py" -path "*/scripts/*" -exec chmod +x {} +
```

### RViz2 没有显示地图
- 确认 map_server 已成功激活（`ros2 lifecycle get /map_server` 应显示 `active`）
- 确认 TF 树完整（`ros2 run tf2_tools view_frames.py`）

---

## 引用

- [FAST-LIO2](https://github.com/hku-mars/FAST-LIO2)
- [Navigation2](https://docs.nav2.org/)
- [Unitree LiDAR ROS2](https://github.com/unitreerobotics/unitree_lidar_ros2)
