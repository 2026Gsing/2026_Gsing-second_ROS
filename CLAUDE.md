# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Jazzy robot navigation + vision-based auto-task system for **ROBOCON 2026** 仿生足式机器人挑战赛（任务赛）。

Robot: Unitree Go2/B2 wheel-leg quadruped with STM32H723VGTX bare-metal firmware.

**RMW must be CycloneDDS** — every ROS2 command needs `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## ⚠️ Python 环境陷阱

**默认 `python3` 解析到 conda 3.14.4，无法 import rclpy**（ROS2 的 C 扩展编译为 Python 3.12 ABI）。

| 方式 | 用什么 Python | 能不能跑 ROS2 |
|------|:---:|:---:|
| `python3 script.py` | conda 3.14.4 | ❌ |
| `./script.py` | shebang → `/usr/bin/python3` (3.12) | ✅ |
| `./ros-run.sh script.py` | `/usr/bin/python3` + 自动 source | ✅ |

**所有脚本的 shebang 已改为 `#!/usr/bin/python3`**，因此 `./` 开头可正确使用系统 Python。

`ros-run.sh` 是项目根目录的 helper，自动执行 source + RMW 设置：
```bash
cd /home/gsing/2026Gsing/2026_Gsing-second_ROS
./ros-run.sh py/control/box_pick_node.py
```

---

## Quick Start

```bash
# 1. Source ROS2 + workspaces（每次新终端）
source /opt/ros/jazzy/setup.bash
source fastlio2_v2/install/setup.bash
source nav2_ws1/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2. 一键启动
./py/control/box_pick_node.py              # 物资箱检测抓取
./py/control/auto_task.py field_id:=1       # 全场自动任务
./py/tools/map_scan.py                      # 建图
./py/tools/test_move.py                     # 底盘测试
```

---

## Build Commands

```bash
# FAST-LIO2 workspace (C++: SLAM, PCD→PGM, LiDAR driver)
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh

# Nav2 workspace (launch files + Python scripts only, no C++)
cd nav2_ws1
colcon build --symlink-install
# ⚠️ Manual sync after build:
cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
   install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

**注意**: 如果从别处 clone，`install/` 下的 symlink 会断。先 `rm -rf build/ install/` 再重新编译。

---

## Hardware Setup

```bash
# LiDAR 网卡（Unitree L2 LiDAR，每次重启后执行）
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0

# 串口权限（STM32 下位机）
sudo chmod 666 /dev/ttyACM0              # 临时
sudo usermod -aG dialout $USER           # 永久（需重登录）
```

验证 LiDAR 连接：
```bash
source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /unilidar/cloud --once   # 有数据则正常
```

---

## Architecture

```
2026_Gsing-second_ROS/
├── config/competition.yaml       ← 比赛配置（场地布局、超时、坐标）
├── py/
│   ├── control/                  ← ROS2 节点（直接运行，自动启动前置）
│   │   ├── launch_utils.py       ← 一键启动 LiDAR + ICP + Nav2 + 串口桥
│   │   ├── auto_task.py          ← 全场自动状态机（8 状态循环）
│   │   ├── box_pick_node.py      ← 到达→检测→抓取/重规划
│   │   ├── catch.py              ← 机械臂坐标变换 + STM32 可达性验证
│   │   ├── cube_detector.py      ← DBSCAN+PCA 3D OBB 检测
│   │   ├── arrival_detector.py   ← 位置/角度到达判断
│   │   └── init_pose.py          ← 发布 /initialpose 初始化 ICP
│   └── tools/
│       ├── map_scan.py           ← 一键建图（SLAM→PCD→PGM）
│       ├── test_move.py          ← 底盘运动测试
│       ├── listen_serial.py      ← 串口十六进制监听
│       ├── test_interactive.py   ← 机械臂协议交互测试
│       └── pointcloud_x_filter.py ← 点云 x>0 过滤
├── fastlio2_v2/                  ← FAST-LIO2 SLAM + pcd2pgm 工作空间
│   └── src/
│       ├── fast_lio/             ← C++: FAST-LIO2 核心
│       ├── fast_lio_localization/ ← Python: ICP 全局定位 + transform_fusion
│       ├── pcd2pgm/              ← C++: PCD → 栅格地图
│       └── unitree_lidar_ros2/   ← C++: Unitree L2 LiDAR 驱动
├── nav2_ws1/                     ← Nav2 工作空间
│   └── src/dog_nav2_bringup/
│       ├── launch/               ← launch 文件
│       ├── scripts/              ← cmd_vel → STM32 串口桥等
│       ├── params/               ← Nav2 调参 YAML
│       └── maps/                 ← 预存栅格地图
├── vision/                       ← YOLO 权重 (task3.pt, math12.pt)
└── map/                          ← 建图输出 (PCD + PGM + YAML)
```

### 关键数据流

```
LiDAR → /unilidar/cloud → pointcloud_x_filter(x>0) → /unilidar/cloud_filtered
                                                           ↓
                                                    FAST-LIO2 SLAM
                                                           ↓
                                            ┌────────────────┴────────────────┐
                                            ↓                                ↓
                              fast_lio_localization (ICP)            pcd2pgm (PCD→栅格)
                                            ↓                                ↓
                                   /localization (Odometry)           map/*.pgm
                                            ↓
                              ┌─────────────┴─────────────┐
                              ↓                           ↓
                    arrival_detector                  Nav2 (规划+控制)
                    (到达判断)                              ↓
                                                      /cmd_vel
                                                         ↓
                                              chassis_serial_bridge
                                              [0x55][0xAA][0x10]... → STM32
```

---

## Launch Utilities (`py/control/launch_utils.py`)

`auto_task.py` 和 `box_pick_node.py` 共用 `start_prerequisites()` 启动全部前置节点：

```
顺序: LiDAR → ICP定位 → TF桥 → Nav2导航 → 串口桥
```

顶部开关：
```python
ENABLE_RVIZ = True      # 是否打开 RViz
USE_TERMINAL = True     # 每个节点开独立 gnome-terminal 窗口
SERIAL_PORT = "/dev/ttyACM0"
MAP_NAME = "map/PCD17"  # 默认地图名（不含扩展名）
```

所有子进程 stdout+stderr → `logs/{category}/YYYY-MM-DD_HHMMSS_name.log`。

---

## 各脚本行为

### `box_pick_node.py` — 物资箱到达→检测→抓取

```
start_prerequisites() → sleep 8s → init_icp_pose(0,0 X正向)
  → rclpy.spin()
  → 等待 /goal_pose（RViz 2D Goal Pose）→ 记录目标坐标
  → 自动到达检测（距离 < 0.15m）→ on_arrived()
    → 启动 cube_detector 子进程
    → 收到 /detected_cube (Marker) → transform_and_offset()
    → validate_arm_target() + stm32_will_accept()
      → 可达 → 启动 catch.py 抓取（发送 0x12 坐标）
      → 不可达 → Nav2 导航到立方体前方 0.3m
    → catch.py 收到 ARM_EVENT pick_done → 退出 → 回到 IDLE
```

交互命令: `arrived` `status` `stop` `quit`

### `auto_task.py` — 全场自动状态机

```
IDLE → SOLVE_TASK(数学题) → FIND_BOX → NAV_BOX
  → WAIT_PICK → NAV_ZONE → WAIT_PLACE → NEXT_OR_FINISH (loop)
```

- 从 `config/competition.yaml` 读取场地配置
- 取货顺序：高分类型 → 外排 → 左到右
- 比赛超时 180s 自动结束

### `map_scan.py` — 一键建图

```
LiDAR → FAST-LIO2 SLAM → Enter 保存 PCD → 启动 pcd2pgm → PGM + YAML
```

---

## Serial Protocol（ROS ↔ STM32）

固定帧格式: `[0x55][0xAA][func_id][len][payload...][checksum]`

| Code | 名称 | 载荷 | 方向 | 发送者 |
|------|------|------|------|--------|
| `0x10` | CHASSIS_MOVE | `vx(f32)+wz(f32)+state(u8)` = 9B | ROS→STM32 | 串口桥 50Hz |
| `0x11` | GAIT_SWITCH | `gait_id(u8)` = 1B | ROS→STM32 | test_move.py |
| `0x12` | ARM_CONTROL | `x(f32)+y(f32)+z(f32)` = 12B | ROS→STM32 | catch.py / auto_task |
| `0x13` | SUCTION | 1B | ROS→STM32 | - |
| `0x14` | ARM_MISSION | `mode(u8)+flags(u8)+pick/back/place` | ROS→STM32 | auto_task |
| `0x15` | AUTO_TASK | `cmd(u8)+target(u8)+zone(u8)` = 3B | ROS→STM32 | auto_task |
| `0x22` | ARM_EVENT | `event+mode+slot+side+xyz` = 16B | STM32→ROS | 串口桥 RX 线程 → `/vision/arm_event` |
| `0x31` | LEG_DEBUG | 腿部调试数据 | STM32→ROS | 串口桥 RX 线程 |

AUTO_CMD: `START(1)` → `ARRIVED_BOX(2)` → `PICK_DONE(3)` → `ARRIVED_ZONE(4)` → `PLACE_DONE(5)` → `NEXT(6)` → `FINISH(7)` → `ESTOP(8)`

ARM_EVENT: `pick_done(1)`, `place_done(2)` — JSON 格式发布

### Velocity Arbitration（串口桥 `cmd_vel_chassis_serial.py`）

1. `/vision_cmd_vel` < 500ms 且非零 → 视觉精细控速（覆盖 Nav2）
2. `/cmd_vel` (Nav2) < 80ms → Nav2 导航速度
3. 否则 → 停止 (ROBOT_STATE_IDLE)

---

## Coordinate Transform & Arm Reachability

`catch.py` 中 LiDAR 坐标系 → 机械臂坐标系：
```python
arm_x = -radar_z - OFFSET_X + HALF_BOX_HEIGHT   # 高度 (up)
arm_y =  radar_y                                  # 横向 (right)
arm_z = -radar_x - OFFSET_Z                      # 前向 (forward)
```

验证函数（与 STM32 `arm_task.c` + `arm.c` 严格一致）：

| 检查 | 函数 | 范围（补偿前） |
|------|------|----------------|
| 坐标轴边界 | `validate_arm_target()` | X∈[-0.23,0.42], Y∈[-0.50,0.50], Z∈[-0.75,0.55] |
| IK 可达性 | `validate_arm_target()` | `sqrt(x²+y²+z²)` ∈ [0.02, 0.62] |
| 肩部禁区 | `validate_arm_target()` | Z≥0 ⇒ X≥0（肩部以下够不到前方） |
| STM32 后补偿 | `stm32_will_accept()` | X 加 0.03 后检查 STM32 原始范围 |

---

## Nav2 Speed Tuning

```bash
# 运行时调参（无需重启）
ros2 param set /controller_server FollowPath.desired_linear_vel 1.0
ros2 param set /controller_server FollowPath.max_linear_vel 1.2
ros2 param set /controller_server FollowPath.lookahead_dist 0.8
```

配置文件: `nav2_ws1/src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml`

---

## 各脚本依赖的系统 Python 包

| 包 | 用途 | 安装方式 |
|----|------|----------|
| `rclpy` + ROS2 msgs | ROS2 Python | `apt install ros-jazzy-*` (系统 Python 3.12) |
| `ultralytics` | YOLO 检测 | `pip install ultralytics` → `~/.local/lib/python3.12/` |
| `cv2` (opencv) | 摄像头 | `apt install python3-opencv` |
| `numpy` | 数值计算 | 系统预装 |
| `scikit-learn` | DBSCAN 聚类 | `apt install python3-sklearn` |
| `pyserial` | 串口通信 | `apt install python3-serial` |
| `pyyaml` | 配置文件 | `apt install python3-yaml` |
| `sensor_msgs_py` | point_cloud2 | `apt install ros-jazzy-sensor-msgs-py` |
| `tf-transformations` | TF 工具 | `apt install ros-jazzy-tf-transformations` |

---

## 常见问题

### 编译/构建
- **symlink 断链**（`install/` 指向 `/home/hyper/...`）：`rm -rf build/ install/` 后重新编译
- **fast_lio_localization 找不到**：编译后运行 `hook_fix.sh`
- **Nav2 参数未同步**：编译后手动 cp params YAML 到 `install/`
- **`ModuleNotFoundError: No module named 'lark'`**：`pip install lark`（conda Python 需要）
- **`ModuleNotFoundError: No module named 'catkin_pkg'`**：`pip install catkin_pkg empy`（conda Python 需要）

### 运行
- **`rclpy` import 失败**：用了 `python3` 而非 `./` 运行脚本
- **ICP 未初始化**：`box_pick_node.py` 自动等 8s 后发 `/initialpose`，或在 RViz 中点 "2D Pose Estimate"
- **TF 树不更新**：`ros2 run tf2_tools view_frames.py` 检查 map→camera_init→odom→base_link 链
- **pcd_to_pointcloud crash (exit -6)**：CPU 无 AVX，`pcl_ros` 节点 SIGABRT，不影响主流程
- **ICP 说 "Skipping: only N pts, need >= 5000"**：检查 LiDAR 连接和 x>0 滤波器
- **`python3: can't open file`**：确保在项目根目录运行

### 硬件
- **LiDAR 没数据**：检查 `nmcli` 网卡设置和 `ip addr`
- **串口打不开**：`sudo chmod 666 /dev/ttyACM0` 或确认 STM32 已连接
- **`/dev/ttyACM0` 不存在**：STM32 未连接或被其他程序占用（`ls /dev/tty*` 查看）
