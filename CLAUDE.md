# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Humble robot navigation + vision-based auto-task system for **ROBOCON 2026** 仿生足式机器人挑战赛（任务赛）。

Robot: Unitree Go2/B2 wheel-leg quadruped with STM32H723VGTX bare-metal firmware.

**RMW must be CycloneDDS** — every ROS2 command needs `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## ⚠️ Python 环境陷阱

**默认 `python3` 解析到 conda 3.14.x，无法 import rclpy**（ROS2 的 C 扩展编译为 Python 3.10 ABI）。

| 方式 | 用什么 Python | 能否跑 ROS2 |
|------|:---:|:---:|
| `python3 script.py` | conda 3.14 | ❌ |
| `./ros-run.sh script.py` | `/usr/bin/python3` + 自动 source | ✅ |
| `/usr/bin/python3 script.py` | system 3.10 | ✅（需手动 source） |

`ros-run.sh` 已包含自动 source + RMW 设置，推荐使用：
```bash
./ros-run.sh py/control/box_pick_node.py
```

## 双机配置（主机 + miniPC）

代码通过 Git 在开发主机（`hyper-Ubuntu`）和机载 miniPC（`gsing`）之间同步。

| 方面 | 主机 (hyper-Ubuntu) | miniPC (gsing) |
|------|---------------------|----------------|
| hostname | `hyper-Ubuntu` | `gsing` |
| 项目路径 | `/home/hyper/program/...` | `/home/gsing/2026Gsing/...` |
| 网卡（LiDAR） | `enp129s0` | 需 `ip a` 确认 |
| 默认 RViz | ✅ 开 | ❌ 关（`GSING_RVIZ=1` 临时开） |

**路径自动适配**：`Path(__file__).resolve()` 在任何机器上都能找到正确路径。C++ 各编各的（`build/` `install/` 在 `.gitignore` 中）。

## Quick Start

```bash
# 1. Source ROS2 + workspaces（每个新终端）
source /opt/ros/humble/setup.bash
source fastlio2_v2/install/setup.bash
source nav2_ws1/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2. 一键启动（推荐用 ros-run.sh）
./ros-run.sh py/control/box_pick_node.py         # 物资箱检测抓取
./ros-run.sh py/control/auto_task.py field_id:=1  # 全场自动任务
./ros-run.sh py/tools/map_scan.py                 # 建图
./ros-run.sh py/tools/test_move.py                # 底盘测试
```

## Build Commands

```bash
# FAST-LIO2 workspace (C++: SLAM, PCD→PGM, LiDAR driver)
cd fastlio2_v2
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh

# Nav2 workspace (launch files + Python scripts only, no C++)
cd nav2_ws1
colcon build --symlink-install
# ⚠️ Manual sync after build (install/ 下是副本)：
cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
   install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

**注意**: 从别处 clone 后 `install/` 的 symlink 会断，先 `rm -rf build/ install/` 再重新编译。

## Hardware Setup

```bash
# LiDAR 网卡（每次重启后执行）
sudo nmcli device set enp129s0 managed no     # miniPC 网卡名可能不同
sudo ip addr add 192.168.1.2/24 dev enp129s0

# 验证 LiDAR 连接
source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /unilidar/cloud --once

# 串口权限
sudo chmod 666 /dev/ttyACM0              # 临时
sudo usermod -aG dialout $USER           # 永久（需重登录）
```

## RViz 控制

LiDAR 和 ICP 强制不开 RViz（`launch_utils.py` 中硬编码 `start_rviz:=false` / `rviz:=false`），只有 Nav2 受 `GSING_RVIZ` 控制。

```bash
GSING_RVIZ=1 ./ros-run.sh py/control/auto_task.py     # 强制开 Nav2 RViz
GSING_RVIZ=0 ./ros-run.sh py/control/box_pick_node.py # 强制关 Nav2 RViz
```

默认行为：主机开（`hyper-Ubuntu`），miniPC 关（`gsing`）。

## Architecture

```
2026_Gsing-second_ROS/
├── config/competition.yaml       ← 比赛配置（场地布局、超时、坐标）
├── py/                           ← ROS2 Python 节点
│   ├── control/                  ← 自动任务、机械臂控制、检测
│   │   ├── launch_utils.py       ← 一键启动 LiDAR + ICP + Nav2 + 串口桥
│   │   ├── auto_task.py          ← 全场自动状态机
│   │   ├── box_pick_node.py      ← 到达→检测→抓取/重规划
│   │   ├── catch.py              ← 机械臂坐标变换 + STM32 可达性验证
│   │   ├── cube_detector.py      ← DBSCAN+PCA 3D OBB 检测
│   │   ├── arrival_detector.py   ← /localization → 到达判断
│   │   └── init_pose.py          ← 发布 /initialpose（ICP 初始化）
│   └── tools/                    ← 独立工具
│       ├── map_scan.py           ← 一键建图（SLAM→PCD→PGM）
│       ├── test_move.py          ← 底盘运动测试（支持 --auto 序列）
│       ├── test_interactive.py   ← 交互式测试
│       ├── listen_serial.py      ← 串口十六进制监听
│       └── pointcloud_x_filter.py ← LiDAR 点云 x>0 过滤
├── fastlio2_v2/                  ← FAST-LIO2 SLAM 工作空间（C++）
│   └── src/
│       ├── unilidar_fastlio_ros2-ros2/ ← FAST-LIO2 核心（建图+里程计）
│       ├── fast_lio_localization/    ← ICP 全局定位 + transform_fusion
│       │   ├── global_localization.py ← numpy/scipy ICP 配准
│       │   ├── transform_fusion.py    ← TF 融合
│       │   └── publish_initial_pose.py ← 初始位姿发布
│       ├── pcd2pgm/              ← PCD → 栅格地图
│       ├── unitree_lidar_ros2/   ← LiDAR 驱动
│       └── my_odom_tf_pkg/       ← odometry→TF 桥接
├── nav2_ws1/                     ← Nav2 工作空间
│   └── src/dog_nav2_bringup/
│       ├── launch/
│       │   ├── nav2_fastlio_static_map.launch.py  ← 静态地图（竞赛主用）
│       │   ├── nav2_fastlio_bringup.launch.py      ← 动态建图
│       │   └── chassis_serial_bridge.launch.py     ← 串口桥
│       ├── scripts/
│       │   ├── cmd_vel_chassis_serial.py  ← /cmd_vel → STM32 串口
│       │   ├── nav2_task_launch.py        ← AMCL 版 Nav2 启动（备选）
│       │   ├── goal_pose_to_nav2.py       ←（已停用，保留供参考）
│       │   ├── costmap_to_grid.py         ← costmap → RViz 显示
│       │   ├── task_field_competition.sh  ← 竞赛一键启动
│       │   ├── generate_standard_map.sh   ← 6m×4m 标准地图
│       │   └── start_nav2.sh / start_nav2_full.sh
│       ├── params/
│       │   ├── nav2_fastlio_static_map_params.yaml  ← 竞赛导航参数
│       │   └── nav2_fastlio_params.yaml             ← 建图导航参数
│       └── maps/                    ← 预存 2D 栅格地图
├── vision/                       ← YOLO 视觉模块
│   ├── src/
│   │   ├── predict.py             ← YOLO 检测主脚本
│   │   ├── slot_roi.py            ← ROI 槽位分配
│   │   └── MATH.PY                ← 数学符号识别
│   ├── config/                    ← IPC JSON（slots_roi, decision_state, nav_target）
│   └── weights/
│       ├── task3.pt               ← 4 类物资检测 (tool/device/food/remedy)
│       └── math12.pt              ← 数学符号检测
├── config/competition.yaml       ← 比赛配置
├── map/                          ← 建图输出（PCD + PGM + YAML）
├── logs/                         ← 运行日志（gitignored）
└── ros-run.sh                    ← helper: source + system Python 启动
```

### Key Data Flows

**Navigation pipeline:**
```
LiDAR L2 → /unilidar/cloud → pointcloud_x_filter → /unilidar/cloud_filtered
  → FAST-LIO2 SLAM → /Odometry → odometry_to_tf → TF camera_init→body
  → ICP (global_localization.py) → /map_to_odom → transform_fusion → /localization + TF map→camera_init
  → Nav2 (planner + controller) → /cmd_vel
  → cmd_vel_chassis_serial.py (0x10 frame with state) → STM32
```

**TF tree:**
```
map ← (ICP / transform_fusion) ← camera_init ← (odometry_to_tf, dynamic) ← body ← (static) ← base_link
                                   └── (static identity) ← odom  (local costmap 用)
```
- `camera_init→body` 由 FAST-LIO2 的 `odometry_to_tf` 动态发布
- `body→base_link` 是静态恒等变换
- **不发布 `camera_init→base_link` 静态恒等** — 会与动态链冲突
- `map→camera_init` 由 `transform_fusion.py` 以 ICP 结果发布

**Vision auto task:**
```
YOLO detection → JSON files (IPC) → auto_task.py (state machine)
  → START/ARRIVED_BOX/ARRIVED_ZONE/FINISH (0x15) → STM32
  → NavigateToPose action (Nav2, long-range)
  → /vision_cmd_vel (fine alignment, overrides Nav2)
```

## Launch Utilities (`py/control/launch_utils.py`)

Both `auto_task.py` and `box_pick_node.py` share `start_prerequisites()`.

**Startup order:** LiDAR → wait 10s → ICP localization → TF bridge → Nav2 → Serial bridge

**Startup cleanup:** Before launching, automatically kills stale ROS processes (`pkill -f`) and cleans CycloneDDS shared memory (`/dev/shm/*cyclone*`).

**Exit cleanup:** `cleanup_all()` kills background processes + terminal ROS nodes + cleans DDS shm.

Top-level switches:
```python
ENABLE_RVIZ = True      # GSING_RVIZ env var overrides; hostname-based default
USE_TERMINAL = False    # False = 后台静默运行（日志写文件），True = gnome-terminal
SERIAL_PORT = "/dev/ttyACM0"
MAP_NAME = "map/map"    # 地图文件名，同时用于 PCD 和 YAML
```

### ICP 定位参数 (`launch_utils.py:244-247`)

```python
map_voxel_size:=0.08 scan_voxel_size:=0.08
freq_localization:=2.0 localization_threshold:=0.85 scale_x:=1.0
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `map_voxel_size` | 0.08 m | 地图下采样体素，~4K 点 |
| `scan_voxel_size` | 0.08 m | 扫描下采样体素，与地图一致 |
| `freq_localization` | 2.0 Hz | ICP 定位频率（每 0.5s） |
| `localization_threshold` | 0.85 | ICP 内点率阈值（fitness > 0.85 接受） |
| `scale_x` | 1.0 | X 方向增量缩放系数（0.6 试调中）。仅影响发布值，不影响 ICP 内部初值。运行时改：`ros2 param set /global_localization scale_x 0.6`，或启动前设环境变量：`SCALE_X=0.6 ./ros-run.sh py/control/auto_task.py` |

注意：ICP 的 fitness 虽高（0.94~1.0），但在矩形场地上 fitness 对 x 方向滑移不敏感。**高 fitness ≠ 高 x 精度**，需靠 scale_x 修正比例尺误差。

ICP 采用多尺度策略：coarse (scale=5, voxel×5, max_dist=2.5m) → fine (scale=1, voxel×1, max_dist=0.5m)。
fitness = 内点数 / 源点总数（内点率 0~1），coarse 因搜索半径大因此 fitness 自然更高。

### 底盘状态码推导 (`cmd_vel_chassis_serial.py:derive_robot_state()`)

Serial bridge 从 `(vx, wz)` 速度推导 `state` 字节（0x10 帧的第9字节）：

```python
# vx 非零 → FORWARD(1) 或 BACKWARD(2)
# vx 为零且 wz 非零 → LEFT(3) 或 RIGHT(4)（纯旋转）
# 全部零 → IDLE(0)
```

2026-07-15 修正：之前当 `abs(wz) > abs(vx)` 时返回 LEFT/RIGHT，导致 Nav2 导航时（vx=0.1, wz=0.15→0.29）发送 state=LEFT，STM32 优先转向步态，前进速度降至指令的 1/8。

## Serial Protocol (ROS ↔ STM32)

Frame: `[0x55][0xAA][func_id][len][payload...][checksum]`

| Code | Name | Payload | Direction | Sent by |
|------|------|---------|-----------|---------|
| `0x10` | CHASSIS_MOVE | `vx(f32)+wz(f32)+state(u8)` = 9B | ROS→STM32 | serial bridge 50Hz |
| `0x11` | GAIT_SWITCH | `gait_id(u8)` = 1B | ROS→STM32 | test_move.py |
| `0x12` | ARM_CONTROL | `x(f32)+y(f32)+z(f32)` = 12B | ROS→STM32 | catch.py |
| `0x14` | ARM_MISSION | `mode(u8)+flags(u8)+pick/back/place` | ROS→STM32 | auto_task.py |
| `0x15` | AUTO_TASK | `cmd(u8)+target(u8)+zone(u8)` = 3B | ROS→STM32 | auto_task.py |
| `0x22` | ARM_EVENT | `event+mode+slot+side+xyz` = 16B | STM32→ROS | serial bridge |
| `0x31` | LEG_DEBUG | 72B 腿部调试帧 | STM32→ROS | serial bridge |

AUTO_CMD: 1=START, 2=ARRIVED_BOX, 3=PICK_DONE, 4=ARRIVED_ZONE, 5=PLACE_DONE, 6=NEXT, 7=FINISH, 8=ESTOP

**注意 (2026-07-15)**: `0x10` 帧的 `wz` 在 `cmd_vel_chassis_serial.py:_build_packet()` 中取反后发送。原因是 STM32 端转向方向与 ROS 约定相反（ROS 左转为正，STM32 右转为正），取反后双方行为一致。

## Navigation: best practices

- 使用 **GoalTool**（`nav2_rviz_plugins/GoalTool`）发导航目标，**不要用** SetGoal（`rviz_default_plugins/SetGoal`）— 后者发 `/goal_pose` 已无订阅者
- 目标点必须在地图的 free 区域内，否则 planner 会失败
- `box_pick_node.py` 支持 CLI 命令：`goto <x> <y> [yaw]` / `nav <x> <y> [yaw]` / `pos`

## cube_detector.py — 3D OBB 立方体检测

| 步骤 | 方法 | 参数 |
|------|------|------|
| 空间裁剪 | 雷达前方 0~0.8m | `x_range=(−0.2, 0.8)` |
| 去地面 | 移除 z 最低 5% 分位 | `z_keep_percent=2` |
| 半径滤波 | BallTree 邻域点数过滤 | `radius=0.04, min_neighbors=5` |
| 体素下采样 | 重心下采样 | `voxel_size=0.008` (8mm) |
| DBSCAN 聚类 | 密度聚类分离物体 | `eps=0.04, min_samples=20` |
| 3D PCA | 协方差 → OBB 主轴 + 边长 | — |
| 边长校验 | 25cm ± 5cm | `edge_target=0.25, edge_tol=0.05` |

## 速度仲裁

底盘速度优先级：`/vision_cmd_vel` (500ms 超时) > `/cmd_vel` Nav2 (80ms 超时) > 停止

到达检测：订阅 `/localization`，参数 `arrival_pos_threshold=0.25m`, `arrival_angle_threshold=0.30rad`, `settle_frames=5`。

## Common Pitfalls

- **LiDAR network**: `sudo nmcli device set enp4s0 managed no && sudo ip addr add 192.168.1.2/24 dev enp4s0`（笔记本用 `enp129s0`）
- **LiDAR no data**: Check with `ping 192.168.1.1` and `timeout 5 tcpdump -i enp4s0 port 6201 -X`
- **RMW mismatch**: All terminals MUST set `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- **Nav2 params out of sync**: After `colcon build`, manually `cp` params YAML to install/
- **DDS participant exhaustion**: Script auto-cleans `/dev/shm/*cyclone*` on startup and exit
- **conda Python**: Always use `./ros-run.sh` or explicit `/usr/bin/python3`, never bare `python3`
- **Serial port**: `sudo chmod 666 /dev/ttyACM0`
- **ICP not initialized**: When LiDAR data doesn't reach ICP (`cur_scan=✗`), TF 树断裂导致 Nav2 无法启动。通常在快速重启场景出现。确认 LiDAR 网络连通后重试。
- **Robot tilting severely (pitch > 15°)**：导航速度过快会导致机器人前倾，使 LiDAR 倾斜 → ICP 匹配不稳定 → 定位漂移 → Nav2 路径规划失败。建议降低 `max_linear_vel` 或检查场地平整度。
