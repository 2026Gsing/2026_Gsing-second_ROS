# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Jazzy 机器人导航+视觉自动任务系统，用于仿生足式机器人竞赛。集成 **FAST-LIO2**（LiDAR SLAM + 全局定位）、**Nav2**（路径规划控制）、**YOLO 视觉**（物资识别 + 数学题）和 **STM32 串口通信**（底盘 + 机械臂）。

三个独立 colcon 工作空间 + 两个非构建目录协同工作。

## Repository Structure

```
├── fastlio2_v2/           # Colcon 工作空间：FAST-LIO2 SLAM + 定位
│   └── src/
│       ├── unilidar_fastlio_ros2-ros2/  # FAST-LIO2 核心 (建图+里程计)
│       ├── fast_lio_localization/       # ICP 全局重定位 (ament_python)
│       ├── unitree_lidar_sdk/           # 宇树 LiDAR L2 驱动
│       └── pcd2pgm/                     # PCD → PGM 转换
├── nav2_ws1/              # Colcon 工作空间：Nav2 导航 + 串口桥
│   └── src/dog_nav2_bringup/
│       ├── launch/                      # 启动文件 (static_map, dynamic, serial_bridge)
│       ├── params/                      # Nav2 YAML 参数
│       ├── scripts/                     # Python 桥接节点 + shell 脚本
│       │   └── cmd_vel_chassis_serial.py  # 扩展：0x10 + 0x15 串口发送
│       ├── maps/                        # 预建 PGM 栅格地图
│       └── rviz/                        # RViz 配置
├── py/                    # 独立 Python 工具 (不需 colcon build)
│   ├── vision_auto_task_node.py   # 视觉自动任务状态机 (核心节点)
│   ├── arrival_detector.py        # 到达检测工具
│   ├── cube_detector.py           # 3D OBB 立方体检测 (LiDAR 点云)
│   ├── catch.py                   # 机械臂抓取串口控制
│   ├── listen_serial.py           # 串口监听 (hex 显示)
│   └── config/competition_poses.yaml  # 物资箱/归位区坐标
├── vision/                # 视觉模块 (YOLO) — 整合自 second-YOLO-tmp
│   ├── src/
│   │   ├── predict.py     # YOLO 检测主脚本 (任务模型 + 槽位分配)
│   │   ├── slot_roi.py    # ROI 槽位分配模块
│   │   └── MATH.PY        # 数学符号识别 (YOLO)
│   ├── config/
│   │   ├── slots_roi.json        # ROI 槽位标定
│   │   ├── decision_state.json   # 数学决策结果 (IPC)
│   │   └── nav_target.json       # 导航目标输出 (IPC)
│   └── weights/                   # YOLO 模型权重
│       ├── task3.pt              # 4 类物资检测 (tool/device/food/remedy)
│       └── math12.pt             # 数学符号检测 (0-9, ±×÷())
├── run_auto_task.sh       # 任务赛一键启动脚本
├── run.md                 # 详细启动步骤
└── README.md              # 完整系统架构文档
```

## 数据流

### 导航数据流
```
Unitree LiDAR → /unilidar/cloud
  → FAST-LIO2 → /Odometry + /cloud_registered
    → global_localization.py (ICP vs PCD 地图) → /map_to_odom
      → transform_fusion.py → TF map→camera_init + /localization
        → Nav2 (planner + controller) → /cmd_vel
          → cmd_vel_chassis_serial.py → serial 0x10 → STM32
```

### 自动任务数据流（新增）
```
YOLO 视觉检测 (vision/src/predict.py)
  → 检测结果 (JSON 文件或 ROS topic)
    → vision_auto_task_node.py (状态机)
      ← 读取 /localization (导航到达判断)
      → 发布 Nav2 NavigateToPose goal (长距导航)
      → 发布 /vision_cmd_vel (精细对位)
      → 发布 /vision/auto_cmd (JSON → serial 0x15)
        → cmd_vel_chassis_serial.py → serial 0x15 → STM32
```

## 串口协议 (ROS → STM32)

### 0x10 — FUNC_CHASSIS_MOVE (底盘速度)
```
[0x55][0xAA][0x10][0x09][vx(f32)][wz(f32)][state(u8)][checksum]
```
- vx>0 前进, vx<0 后退, wz>0 左转, wz<0 右转
- state: 0=IDLE, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT
- 发送频率 ≥20Hz, 100ms 超时自动停车

### 0x15 — FUNC_AUTO_TASK (自动任务事件)
```
[0x55][0xAA][0x15][0x03][cmd(u8)][target(u8)][zone(u8)][checksum]
```
- cmd: 1=START, 2=ARRIVED_BOX, 3=PICK_DONE, 4=ARRIVED_ZONE, 5=PLACE_DONE, 6=NEXT, 7=FINISH, 8=ESTOP
- target: 物资箱编号, zone: 归位区编号

由 `cmd_vel_chassis_serial.py` 订阅 `/vision/auto_cmd` (JSON) 自动转发。

## 自动任务状态机 (py/vision_auto_task_node.py)

```
IDLE → (输入 start)
  → SOLVE_TASK   等待 YOLO 数学题结果 → mod4 → zone_sequence
  → FIND_BOX     YOLO 检测物资箱类别
  → NAV_BOX      发送 Nav2 goal → 监听 /localization → 到达
  → WAIT_PICK    等待 ~5s → 发 PICK_DONE (0x15)
  → NAV_ZONE     发送 Nav2 goal → 到归位区
  → WAIT_PLACE   等待 ~5s → 发 PLACE_DONE (0x15)
  → NEXT_OR_FINISH → 下一箱 / 全部完成
```

### Nav2 速度 vs 视觉速度优先级
`cmd_vel_chassis_serial.py` 的底盘速度仲裁：
1. `/vision_cmd_vel` 在 500ms 内有数据 → 使用视觉速度（精细对位）
2. 否则 `/cmd_vel` (Nav2) 在 80ms 内有数据 → 使用 Nav2 速度（长距导航）
3. 否则 → 停止

## 启动命令

### 环境与构建
```bash
source /opt/ros/jazzy/setup.bash
pip install open3d tf_transformations ultralytics pyserial
sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs python3-serial imagemagick

# FAST-LIO2 选择性构建
cd fastlio2_v2
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh

# Nav2 完整构建
cd nav2_ws1
colcon build --symlink-install
```

### 竞赛启动流程
```bash
# 终端 A: LiDAR 驱动
cd fastlio2_v2 && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# 终端 B: FAST-LIO2 建图
ros2 run fast_lio fastlio_mapping --ros-args --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml

# 终端 C: 全局定位
ros2 launch fast_lio_localization 1.launch.py map:=path/to/scans.pcd config_file:=unilidar_l2.yaml rviz:=true
# → 在 RViz 中点击 "2D Pose Estimate" 初始化

# 终端 D: Nav2 导航 + 串口桥 + 视觉任务 (一键)
bash run_auto_task.sh

# 或分别启动:
# Nav2 导航
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py map:=nav2_ws1/src/dog_nav2_bringup/maps/task_field_map.yaml

# 串口桥
ros2 run dog_nav2_bringup cmd_vel_chassis_serial --ros-args -p serial_port:=/dev/ttyACM0

# 视觉状态机
python3 py/vision_auto_task_node.py

# YOLO 检测 (可选，另开终端)
cd vision && python3 src/predict.py --weights weights/task3.pt --source 1 --draw-roi
```

### 工具命令 (不需构建)
```bash
source /opt/ros/jazzy/setup.bash
python3 py/cube_detector.py          # LiDAR 3D 立方体检测
python3 py/catch.py                  # 机械臂抓取控制
python3 py/listen_serial.py          # 串口监听
python3 py/fastlio_pose.py           # 实时位姿
python3 py/arrival_detector.py       # 到达检测 (单测)
```

### 地图生成
```bash
# 保存 PCD 地图 (建图运行中)
ros2 service call /map_save std_srvs/srv/Trigger

# PCD → PGM 转换
ros2 launch pcd2pgm pcd2pgm_launch.py

# 竞赛场地生成
bash nav2_ws1/src/dog_nav2_bringup/scripts/task_field_competition.sh
```

### LiDAR 网络
```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

## 关键文件

- `py/vision_auto_task_node.py` — 视觉状态机核心，协调 YOLO + Nav2 + 串口
- `cmd_vel_chassis_serial.py` — 串口协议桥 (0x10 底盘 + 0x15 任务事件)
- `vision/src/predict.py` — YOLO 检测 + 槽位分配 + 文件 IPC
- `vision/src/slot_roi.py` — ROI 槽位管理（检测框 → 编号映射）
- `vision/src/MATH.PY` — 数学符号 YOLO 识别 → 方程求值
- `py/cube_detector.py` — DBSCAN + PCA 立方体检测 (LiDAR)
- `py/config/competition_poses.yaml` — 物资箱/归位区预置坐标
- `fast_lio_localization/global_localization.py` — ICP 全局重定位

## 常见问题

- **fast_lio_localization 找不到包**: 运行 `hook_fix.sh` 后重新 `source install/setup.bash`
- **串口权限**: `sudo usermod -aG dialout $USER` 后重新登录；临时 `sudo chmod 666 /dev/ttyACM0`
- **RViz 空白地图**: 检查 `ros2 lifecycle get /map_server` 是否为 active，用 `ros2 run tf2_tools view_frames.py` 检查 TF 树
- **YOLO 导入报错**: 确保在 `vision/` 目录下运行，或 `pip install ultralytics`
