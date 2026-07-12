# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Jazzy robot navigation + vision-based auto-task system for ROBOCON 2026. Integrates **FAST-LIO2** (LiDAR SLAM + ICP global localization), **Nav2** (path planning with RegulatedPurePursuit controller), **YOLO vision** (box classification + math symbol recognition), and **STM32 serial protocol** (chassis + arm control over USB CDC).

**RMW must be CycloneDDS** — `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` before every ROS2 command.

## Coordinate Systems (Critical)

| Frame | +X | +Y | +Z | Notes |
|-------|----|----|----|-------|
| `unilidar_lidar` | forward | **right** | up | LiDAR is inverted; ground at z≈0.39 |
| Arm (STM32) | height (up) | **left** | forward | FK in arm.c:382-384 |

**Transform** (catch.py `transform_and_offset`):
```
arm_x = -radar_z - OFFSET_X     + HALF_BOX_HEIGHT
arm_y = radar_y
arm_z = -radar_x - OFFSET_Z
```

**Arm reachability** (`sqrt(x²+y²+z²)` must be in [0.02, 0.62] — `hu=0.30, hl=0.32`).

## Launch Flow (5 Terminals)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# T1: LiDAR driver
cd fastlio2_v2 && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# T2: FAST-LIO2 + ICP localization
cd fastlio2_v2 && source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH"
export PYTHONPATH=$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages
ros2 launch fast_lio_localization 1.launch.py \
  map:=/home/hyper/program/2026_Gsing-second_ROS/map/map.pcd \
  config_file:=unilidar_l2.yaml rviz:=true
# → Click "2D Pose Estimate" in RViz

# T2b: odometry→TF bridge (if needed)
./build/fast_lio/odometry_to_tf

# T3: Nav2
cd nav2_ws1 && source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=/home/hyper/program/2026_Gsing-second_ROS/map/pgm_map.yaml

# T4: Serial bridge (ROS→STM32)
cd nav2_ws1 && source install/setup.bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 baud_rate:=115200 cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0

# T5: Vision auto task (optional)
python3 py/cube_detector.py     # LiDAR cube detection
python3 py/catch.py             # Arm control via serial bridge
python3 py/vision_auto_task_node.py  # Full auto-task state machine
```

## Build Commands

```bash
# FAST-LIO2
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh

# Nav2
cd nav2_ws1
colcon build --symlink-install
# ⚠️ Manually sync params after build:
cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
   install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

## Key Data Flow

### Navigation
```
LiDAR → /unilidar/cloud → FAST-LIO2 → /Odometry
  → ICP (vs PCD map) → /map_to_odom
    → transform_fusion.py → /localization + TF map→camera_init
      → Nav2 → /cmd_vel → serial 0x10 → STM32
```

### Vision Auto-Task (catch.py + cube_detector)
```
cube_detector.py (DBSCAN+PCA on accumulated point cloud)
  → /detected_cube (Marker) → catch.py
    → transform to arm frame + stability check
      → /vision/auto_cmd (0x15) → STM32 state machine
      → /vision/arm_control (0x12) → STM32 arm IK
```

### Velocity Arbitration (cmd_vel_chassis_serial.py)
1. `/vision_cmd_vel` fresh < 500ms → use vision (fine alignment)
2. `/cmd_vel` (Nav2) fresh < 80ms → use Nav2 (long-range)
3. Otherwise → stop

## Serial Protocol

Frame: `[0x55][0xAA][func_id][len][payload...][checksum]`

| Code | Name | Payload | Sent by |
|------|------|---------|---------|
| `0x10` | CHASSIS_MOVE | `vx(f32)+wz(f32)+state(u8)` = 9B | 50Hz tick |
| `0x12` | ARM_CONTROL | `x(f32)+y(f32)+z(f32)` = 12B | catch.py |
| `0x14` | ARM_MISSION | `mode(u8)+flags(u8)+pick/back/place` | vision_auto_task |
| `0x15` | AUTO_TASK | `cmd(u8)+target(u8)+zone(u8)` = 3B | catch.py / vision |

AUTO_CMD values: 1=START, 2=ARRIVED_BOX, 3=PICK_DONE, 4=ARRIVED_ZONE, 5=PLACE_DONE, 6=NEXT, 7=FINISH, 8=ESTOP.

## Auto-Task State Machine

### ROS side (vision_auto_task_node.py)
```
IDLE → SOLVE_TASK → FIND_BOX → NAV_BOX → WAIT_PICK
  → NAV_ZONE → WAIT_PLACE → NEXT_OR_FINISH → (loop)
```

### catch.py arm flow (open-loop)
```
stable detected → AUTO_CMD_START → 0.1s → AUTO_CMD_ARRIVED_BOX
  → 0.6s delay → 0x12 coordinates → 2Hz heartbeat resend
```

### STM32 auto_task state machine
```
BOOT_SAFE_DELAY(2s) → BOOT_STAND_UP → IDLE
  → START → NAV_TO_BOX → ARRIVED_BOX(400ms) → PICK
    → NAV_TO_ZONE → ARRIVED_ZONE(400ms) → PLACE
      → RETURN_NEXT(300ms) → NAV_TO_BOX (loop)
```
Arm accepts 0x12 only during PICK/PLACE (`Auto_Task_ArmAcceptsNewTarget()`).

## Key Files

### Python tools (no colcon build, run with `python3 py/xxx.py`)
| File | Role |
|------|------|
| `py/cube_detector.py` | DBSCAN + PCA 3D OBB detection, 10-frame accumulation |
| `py/catch.py` | Arm coordinate transform, stability check, serial bridge |
| `py/vision_auto_task_node.py` | Full auto-task state machine |
| `py/cube_detector.py` analyzer | PCA → dims → adaptive center (1-face push / 3-face no-push) |
| `py/config/competition_poses.yaml` | Pre-configured box/zone waypoints |

### ROS serial bridge (colcon package)
| File | Role |
|------|------|
| `nav2_ws1/src/dog_nav2_bringup/scripts/cmd_vel_chassis_serial.py` | Central serial bridge, velocity arbitration, all func codes |
| `nav2_ws1/src/dog_nav2_bringup/launch/chassis_serial_bridge.launch.py` | Launch file |

### SLAM & Localization
| File | Role |
|------|------|
| `fast_lio_localization/global_localization.py` | ICP matching vs PCD map |
| `fast_lio_localization/transform_fusion.py` | TF fusion + /localization |

### STM32 firmware (separate repo at `second-DM4340/`)
| File | Role |
|------|------|
| `Task/arm_task.c` | 13-state arm vision state machine (WAIT_TARGET → HOVER → DOWN → ... → RETREAT) |
| `Task/auto_task.c` | Competition state machine (BOOT → IDLE → NAV → PICK → PLACE → FINISH) |
| `Task/protocol_handler.c` | Serial protocol parser |
| `Algorithm/kinematic/arm.c` | Arm IK/FK solver, workspace limits |

## Common Pitfalls

- **LiDAR network**: `sudo nmcli device set enp129s0 managed no && sudo ip addr add 192.168.1.2/24 dev enp129s0`
- **Serial port**: `sudo chmod 666 /dev/ttyACM0`
- **Nav2 params out of sync**: After `colcon build`, manually `cp` params YAML to install/
- **YOLO import errors**: Run from `vision/` directory
- **STM32 compilation**: Only works on Windows with STM32CubeIDE
- **ICP not updating**: Check TF tree with `ros2 run tf2_tools view_frames.py`
- **`fast_lio_localization` not found**: Run `hook_fix.sh` after build
