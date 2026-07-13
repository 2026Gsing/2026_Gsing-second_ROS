# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Jazzy robot navigation + vision-based auto-task system for **ROBOCON 2026** 仿生足式机器人挑战赛（任务赛）。Robot: Unitree Go2/B2 wheel-leg quadruped with STM32H723VGTX bare-metal firmware + ROS2 Jazzy (Ubuntu 24.04).

**RMW must be CycloneDDS** — `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` before every ROS2 command.

## Build Commands

```bash
# FAST-LIO2 workspace
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh

# Nav2 workspace
cd nav2_ws1
colcon build --symlink-install
# ⚠️ Manual sync after build:
cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
   install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

## One-Click Launch

```bash
# Competition (auto-starts LiDAR, ICP, Nav2, serial bridge, auto_task)
python3 py/control/auto_task.py field_id:=1

# Mapping (LiDAR + FAST-LIO2 SLAM, Enter to save + auto PCD→PGM)
python3 py/tools/map_scan.py
python3 py/tools/map_scan.py --no-rviz    # without RViz

# Box pick (auto-starts all nodes, auto-detect arrival, pick/renav)
python3 py/control/box_pick_node.py

# Chassis test (auto-starts serial bridge only)
python3 py/tools/test_move.py             # interactive mode
python3 py/tools/test_move.py --auto      # auto test sequence
```

## Launch Utilities (`py/control/launch_utils.py`)

Shared module for auto-starting all ROS prerequisite nodes. Used by both `auto_task.py` and `box_pick_node.py`.

```python
# Switches at top of file — change before launch:
ENABLE_RVIZ = True     # Whether ICP and Nav2 launch RViz windows
USE_TERMINAL = True    # Whether to open gnome-terminal per node (vs background)
```

**`start_prerequisites(map_pcd, map_yaml)`** starts (in order):
1. LiDAR driver (`unitree_lidar_ros2`)
2. ICP localization (FAST-LIO2 + `transform_fusion.py`)
3. Odometry→TF bridge (`odometry_to_tf`, skipped if binary missing)
4. Nav2 (`nav2_fastlio_static_map.launch.py`)
5. Serial bridge (`chassis_serial_bridge.launch.py`)

Default maps: `map/map.pcd` (ICP) and `map/map.yaml` (Nav2). Pass custom paths to override.

## File Structure

```
2026_Gsing-second_ROS/
├── config/competition.yaml          ← 赛前配置（场地布局、超时、坐标）
├── py/
│   ├── control/
│   │   ├── launch_utils.py          ← 共享：一键启动所有前置 ROS 节点
│   │   ├── auto_task.py             ← 比赛主状态机 + 拉起全流程
│   │   ├── box_pick_node.py         ← 物资箱自动检测+抓取（含重规划）
│   │   ├── catch.py                 ← 机械臂坐标变换 + 可达性验证
│   │   ├── cube_detector.py         ← DBSCAN+PCA 3D OBB 检测
│   │   ├── arrival_detector.py      ← 到达检测工具
│   │   └── init_pose.py             ← /initialpose 发布(ICP初始化)
│   └── tools/
│       ├── map_scan.py              ← 一键建图(LiDAR+SLAM+PCD→PGM)
│       ├── test_move.py             ← 底盘运动测试(交互/自动序列)
│       ├── listen_serial.py         ← 串口监听(hex显示)
│       ├── test_interactive.py      ← 交互测试
│       └── pointcloud_x_filter.py   ← 点云x>0过滤节点
├── fastlio2_v2/                     ← FAST-LIO2 SLAM + pcd2pgm workspace
├── nav2_ws1/                        ← Nav2 workspace
├── vision/                          ← YOLO detection (task3.pt, math12.pt)
└── map/                             ← 建图输出 (map.pcd, map.pgm+map.yaml)
```

## Coordinate Transform & Reachability

```python
# catch.py transform_and_offset():
arm_x = -radar_z - OFFSET_X + HALF_BOX_HEIGHT   # height (up)
arm_y =  radar_y                                  # lateral (right)
arm_z = -radar_x - OFFSET_Z                      # forward
```

### Validation (matches STM32 arm_task.c + arm.c):

| Check | Function | Bounds (pre-compensation) |
|-------|----------|--------------------------|
| Axis bounds | `validate_arm_target()` | X∈[-0.23,0.42], Y∈[-0.50,0.50], Z∈[-0.75,0.55] |
| IK reachability | `validate_arm_target()` | `sqrt(x²+y²+z²)` ∈ [0.02, 0.62] |
| Shoulder forbidden zone | `validate_arm_target()` | Z≥0 ⇒ X≥0 (can't reach forward below shoulder) |
| STM32 post-compensation | `stm32_will_accept()` | Adds 0.03 to X, checks raw STM32 bounds |

**On validation failure**: `catch.py` clears its position cache and re-acquires. `box_pick_node.py` triggers re-navigation 0.3m closer.

## Auto-Task State Machine (`auto_task.py`)

```
SOLVE_TASK → NAV_BOX → WAIT_PICK → NAV_ZONE → WAIT_PLACE → NEXT_OR_FINISH (loop)
```
- Pickup sequence by `_generate_pickup_sequence()`: high-score type → outer row → left to right
- Uses `cube_detector` + `find_stable_points()` + `validate_arm_target()` from catch module
- Competition timeout 180s auto-finishes

## Box Pick Node (`box_pick_node.py`)

```
启动 → start_prerequisites() → 等8s → init_icp_pose() → 等待 /localization 速度归零
  → 启动 cube_detector → 检测到立方体
  → transform_and_offset() → validate_arm_target() [+ stm32_will_accept()]
    → 可达 → 启动 catch.py 抓取
    → 不可达 → Nav2导航到立方体前方0.3m处 → 到达再检测
```

Commands: `arrived` (manual trigger), `status`, `stop`, `quit`

## Nav2 Speed Tuning

File: `nav2_ws1/src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml`

Live tuning (no restart):
```bash
ros2 param set /controller_server FollowPath.desired_linear_vel 1.0
ros2 param set /controller_server FollowPath.max_linear_vel 1.2
ros2 param set /controller_server FollowPath.lookahead_dist 0.8
```

## Serial Protocol

Frame: `[0x55][0xAA][func_id][len][payload...][checksum]`

| Code | Name | Payload | Sent by |
|------|------|---------|---------|
| `0x10` | CHASSIS_MOVE | `vx(f32)+wz(f32)+state(u8)` = 9B | 50Hz tick via serial bridge |
| `0x12` | ARM_CONTROL | `x(f32)+y(f32)+z(f32)` = 12B | auto_task.py / catch.py |
| `0x14` | ARM_MISSION | `mode(u8)+flags(u8)+pick/back/place` | auto_task.py |
| `0x15` | AUTO_TASK | `cmd(u8)+target(u8)+zone(u8)` = 3B | auto_task.py |

AUTO_CMD: 1=START, 2=ARRIVED_BOX, 3=PICK_DONE, 4=ARRIVED_ZONE, 5=PLACE_DONE, 6=NEXT, 7=FINISH, 8=ESTOP.

## Velocity Arbitration (serial bridge `cmd_vel_chassis_serial.py`)

1. `/vision_cmd_vel` fresh < 500ms → vision (fine alignment)
2. `/cmd_vel` (Nav2) fresh < 80ms → Nav2 (long-range)
3. Otherwise → stop (ROBOT_STATE_IDLE)

## Mapping Pipeline

```
map_scan.py:
  LiDAR driver → /unilidar/cloud → pointcloud_x_filter(x>0) → /unilidar/cloud_filtered
    → FAST-LIO2 SLAM → PCD file
      → pcd2pgm → PGM grid map + YAML (用于 Nav2)
```

Run with LiDAR network connected (`enp129s0: 192.168.1.2/24`) and LiDAR powered.

## Common Pitfalls

- **LiDAR network**: `sudo nmcli device set enp129s0 managed no && sudo ip addr add 192.168.1.2/24 dev enp129s0`
- **Serial port**: `sudo chmod 666 /dev/ttyACM0`
- **Nav2 params out of sync**: After `colcon build`, manually `cp` params YAML to install/
- **fast_lio_localization not found**: Run `hook_fix.sh` after build
- **TF tree not updating**: Check with `ros2 run tf2_tools view_frames.py`; ensure map→camera_init→odom→base_link chain
- **YOLO import errors**: Run from `vision/` directory or `pip install ultralytics`
- **RMW mismatch**: All terminals MUST set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- **ICP not initialized**: After starting, click "2D Pose Estimate" in RViz, or restart so `init_pose.py` auto-publishes
