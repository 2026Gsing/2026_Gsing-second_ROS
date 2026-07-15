# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Jazzy robot navigation + vision-based auto-task system for **ROBOCON 2026** 仿生足式机器人挑战赛（任务赛）。

Robot: Unitree Go2/B2 wheel-leg quadruped with STM32H723VGTX bare-metal firmware.

**RMW must be CycloneDDS** — every ROS2 command needs `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## ⚠️ Python 环境陷阱

**默认 `python3` 解析到 conda 3.14.x，无法 import rclpy**（ROS2 的 C 扩展编译为 Python 3.12 ABI）。

| 方式 | 用什么 Python | 能否跑 ROS2 |
|------|:---:|:---:|
| `python3 script.py` | conda 3.14 | ❌ |
| `./ros-run.sh script.py` | `/usr/bin/python3` + 自动 source | ✅ |
| `/usr/bin/python3 script.py` | system 3.12 | ✅（需手动 source） |

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
source /opt/ros/jazzy/setup.bash
source fastlio2_v2/install/setup.bash
source nav2_ws1/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2. 一键启动
./ros-run.sh py/control/box_pick_node.py         # 物资箱检测抓取
./ros-run.sh py/control/auto_task.py field_id:=1  # 全场自动任务
./ros-run.sh py/tools/map_scan.py                 # 建图
./ros-run.sh py/tools/test_move.py                # 底盘测试
```

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
source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
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
│   │   └── cube_detector.py      ← DBSCAN+PCA 3D OBB 检测
│   └── tools/                    ← 工具脚本
│       ├── map_scan.py           ← 一键建图（SLAM→PCD→PGM）
│       ├── test_move.py          ← 底盘运动测试
│       ├── listen_serial.py      ← 串口十六进制监听
│       └── pointcloud_x_filter.py ← LiDAR 点云 x>0 过滤
├── fastlio2_v2/                  ← FAST-LIO2 SLAM 工作空间（C++）
│   └── src/
│       ├── fast_lio/             ← FAST-LIO2 核心
│       ├── fast_lio_localization/ ← ICP 全局定位 + transform_fusion
│       ├── pcd2pgm/              ← PCD → 栅格地图
│       └── unitree_lidar_ros2/   ← LiDAR 驱动（launch.py 已改支持 start_rviz 参数）
├── nav2_ws1/                     ← Nav2 工作空间
│   └── src/dog_nav2_bringup/
│       ├── launch/               ← launch 文件
│       ├── scripts/              ← cmd_vel → STM32 串口桥
│       ├── params/               ← Nav2 调参 YAML
│       └── maps/                 ← 预存栅格地图
├── vision/                       ← YOLO 权重 (task3.pt, math12.pt)
├── map/                          ← 建图输出 (PCD + PGM + YAML)
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

**TF tree (2026-07-15 修正):**
```
map ← (ICP / transform_fusion) ← camera_init ← (odometry_to_tf, dynamic) ← body ← (static) ← base_link
                                   └── (static identity) ← odom  (local costmap 用)
```
- `camera_init→body` 由 FAST-LIO2 的 `odometry_to_tf` 动态发布（反映机器人运动）
- `body→base_link` 是静态恒等变换
- **不发布 `camera_init→base_link` 静态恒等** — 这会与上述动态链冲突，导致 Nav2 的 SimpleProgressChecker 读取到 base_link 始终在 (0,0)
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

**Startup order:** LiDAR → ICP localization → TF bridge → Nav2 → Serial bridge

**Startup cleanup:** Before launching, automatically kills stale ROS processes (`pkill -f`) and cleans CycloneDDS shared memory (`/dev/shm/*cyclone*`).

**Exit cleanup:** `cleanup_all()` kills background processes + terminal ROS nodes + cleans DDS shm.

Top-level switches:
```python
ENABLE_RVIZ = True      # GSING_RVIZ env var overrides; hostname-based default
USE_TERMINAL = True     # gnome-terminal per node (vs background)
SERIAL_PORT = "/dev/ttyACM0"
MAP_NAME = "map/PCD21"  # 地图文件名（不含扩展名），同时用于 PCD 和 YAML。标准比赛地图
```

### ICP 定位参数 (`launch_utils.py:207-210`)

```python
map_voxel_size:=0.08 scan_voxel_size:=0.08
freq_localization:=2.0 localization_threshold:=0.85
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `map_voxel_size` | 0.08 m | 地图下采样体素（2026-07-15 从 0.008 放宽到 0.08） |
| `scan_voxel_size` | 0.08 m | 扫描下采样体素（2026-07-15 从 0.02 放宽到 0.08） |
| `freq_localization` | 2.0 Hz | ICP 定位频率 |
| `localization_threshold` | 0.85 | ICP 配准 fitness 阈值（2026-07-15 从 0.9 降低） |

### 底盘状态码推导 (`cmd_vel_chassis_serial.py:derive_robot_state()`)

Serial bridge 从 `(vx, wz)` 速度推导 `state` 字节（0x10 帧的第9字节）：

```python
# vx 非零 → FORWARD(1) 或 BACKWARD(2)，即使 wz > vx（前进中转弯）
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

## Navigation: ComputePathToPose Timeout — duplicate goal preemption

**Problem**: bt_navigator logs "Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose", navigation fails within ~22ms.

**Root Cause**: RViz Nav2 plugin (nav2_rviz_plugins/GoalTool) sends a NavigateToPose action goal directly to bt_navigator. Simultaneously, rviz_default_plugins/SetGoal (also in the RViz toolbar) publishes to /goal_pose, which was subscribed by goal_pose_to_nav2.py bridge — causing a **second** NavigateToPose goal that preempts the first.

The preemption cancels the first BT execution while ComputePathToPose's action client is mid-DDS-discovery (wait_for_action_server). The second BT creates a fresh client, but on this slow CPU (Celeron N2940) discovery hasn't completed in ~22ms → wait_for_action_server returns "server not found" → entire navigation fails.

Note: `server_timeout="5.0"` on the ComputePathToPose BT node does NOT fix this — it controls the timeout for the action **result**, but the failure is at the **goal acknowledgment** stage before the server processes the request.

**Fix** (2026-07-15):
- **Removed** `goal_pose_to_nav2.py` bridge from `nav2_fastlio_static_map.launch.py` — GoalTool already sends NavigateToPose directly, the bridge was redundant
- Custom BT XML (`custom_navigate_to_pose_w_replanning_and_recovery.xml` with `server_timeout="5.0"`) retained as safety net

**RViz Tools** in `nav2_fastlio_static_map.rviz`:
```yaml
Tools:
  - Class: nav2_rviz_plugins/GoalTool       # ← sends NavigateToPose action (USE THIS)
  - Class: rviz_default_plugins/SetGoal     # ← publishes /goal_pose only, no subscriber anymore
    Topic: goal_pose
```
SetGoal still appears in the toolbar but has no subscriber — harmless. Always select GoalTool for navigation.

## Common Pitfalls

- **LiDAR network**: `sudo nmcli device set enp4s0 managed no && sudo ip addr add 192.168.1.2/24 dev enp4s0`（笔记本用 `enp129s0`）
- **LiDAR no data**: Check with `ping 192.168.1.1` and `timeout 5 tcpdump -i enp4s0 port 6201 -X`
- **RMW mismatch**: All terminals MUST set `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- **Nav2 params out of sync**: After `colcon build`, manually `cp` params YAML to install/
- **DDS participant exhaustion**: Script auto-cleans `/dev/shm/*cyclone*` on startup and exit
- **conda Python**: Always use `./ros-run.sh` or explicit `/usr/bin/python3`, never bare `python3`
- **Serial port**: `sudo chmod 666 /dev/ttyACM0`
- **ICP not initialized**: `box_pick_node.py` publishes `/initialpose` after 8s, but ICP takes ~15s to load map — message is often dropped. ICP auto-initializes once LiDAR data flows, so this is usually non-blocking.

## 2026-07-15 修改记录

### 1. box_pick_node.py — CLI 导航 + 周期定位输出

**问题**: 导航依赖 RViz 的 GoalTool，但工具栏有 GoalTool/SetGoal 两个按钮，容易选错导致 SetGoal 发布到无人订阅的 `/goal_pose`（桥已移除），导航不启动。

**修复**:
- 新增 `goto <x> <y> [yaw]` CLI 命令，通过 `navigate_to_pose` action 直接发给 bt_navigator，绕过 RViz 工具选择问题
- 新增 `nav <x> <y> [yaw]` 作为 `goto` 的别名
- 新增 `pos` 命令，立即打印当前定位
- 新增每 0.5 秒定时器自动输出定位坐标（x, y, z, yaw, state）
- `_nav_result_cb` 补全导航失败日志（之前只有成功才打印）
- 启动 banner 改为提示 `goto` 命令

### 2. launch_utils.py — `USE_TERMINAL = False`

**问题**: `USE_TERMINAL = True` 每次启动弹 5 个 gnome-terminal 窗口，miniPC (Celeron N2940, 7.6G RAM) 上吃资源。

**修复**: 改为 `False`，后台静默运行，日志仍写入 `logs/` 各子目录。想看实时输出用 `tail -f` 对应日志文件。

---

## 2026-07-15 修复记录

### 1. Nav2 "Failed to make progress" — TF 树冲突

**问题**: `nav2_fastlio_static_map.launch.py` 发布了 `camera_init→base_link` 静态恒等变换，与 FAST-LIO2 `odometry_to_tf` 发布的动态 `camera_init→body→base_link` 链冲突。Nav2 的 SimpleProgressChecker 查找 `odom→base_link` 时解析到静态恒等，始终读到 (0,0)，10 秒后触发失败。

**修复**: 删除了 `static_tf_camerainit_baselink` 节点。动态 TF 链已提供正确的 `camera_init→base_link`。

### 2. Nav2 "Failed to make progress" — 底盘状态码错误

**问题**: `cmd_vel_chassis_serial.py` 的 `derive_robot_state()` 中，当 `abs(wz) > abs(vx)` 时返回 `LEFT(3)`。导航时 Nav2 发出 `vx=0.10, wz=0.15~0.29`，状态码为 LEFT，STM32 优先执行转向步态，实际前进速度仅 0.013 m/s（指令的 1/8）。

**修复**: 改为先判 vx，非零则直接返回 FORWARD(1)/BACKWARD(2)。wz 仅用于纯旋转。

### 3. Nav2 参数调整

| 参数 | 文件 | 旧值 | 新值 |
|------|------|------|------|
| `required_movement_radius` | `nav2_fastlio_static_map_params.yaml` | 0.5 | 0.3 |
| `movement_time_allowance` | ↑ | 10.0 | 30.0 |
| `lookahead_dist` | ↑ | 0.5 | 0.4 |
| `max_lookahead_dist` | ↑ | 0.8 | 0.6 |
| `min_approach_linear_velocity` | ↑ | 0.35 | 0.10 |
| `local_costmap.rolling_window` | ↑ | false | true |

### 4. ICP 定位参数优化

`launch_utils.py` 中 ICP 启动参数：

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `map_voxel_size` | 0.008 | 0.08 | 10x 放宽，地图点从 11万→约 3000-5000，首次匹配 18.5s→<2s |
| `scan_voxel_size` | 0.02 | 0.08 | 与地图一致 |
| `localization_threshold` | 0.9 | 0.85 | 体素变粗后 fitness 自然稍低 |
