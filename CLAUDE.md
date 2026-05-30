# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Jazzy-based robot navigation system for a competition robot (四足狗/quadruped chassis). Integrates **FAST-LIO2** (LiDAR SLAM + global relocalization), **Nav2** (path planning + control), and **STM32 serial communication** (chassis + robotic arm). Two independent colcon workspaces work together.

## Repository Structure

```
├── fastlio2_v2/           # Colcon workspace: FAST-LIO2 SLAM + localization
│   └── src/
│       ├── unilidar_fastlio_ros2-ros2/  # FAST-LIO2 core (mapping + odometry)
│       ├── fast_lio_localization/       # Global relocalization via ICP (ament_python)
│       ├── unitree_lidar_sdk/           # Unitree LiDAR L2 driver
│       └── pcd2pgm/                     # PCD → PGM grid map converter
├── nav2_ws1/              # Colcon workspace: Nav2 navigation
│   └── src/dog_nav2_bringup/
│       ├── launch/                      # 3 launch files (static_map, dynamic, serial_bridge)
│       ├── params/                      # Nav2 YAML parameters
│       ├── scripts/                     # Shell scripts + Python bridge nodes
│       ├── maps/                        # Pre-built PGM grid maps
│       └── rviz/                        # RViz configs
├── py/                    # Standalone utility scripts (no colcon needed)
│   ├── cube_detector.py   # 3D OBB cube detection (DBSCAN+PCA on LiDAR point cloud)
│   ├── catch.py           # Robotic arm control via serial
│   ├── pointcloud_saver.py # Spacebar-triggered PCD saving
│   ├── test_dog.py        # OpenCV HSV color calibration GUI
│   ├── move.py            # Chassis command test tool
│   └── listen_serial.py   # Serial monitor (hex display)
├── run.md                 # Detailed step-by-step launch instructions
└── README.md              # Full system architecture + data flow docs
```

## Key Architecture

### Two Navigation Modes

| Mode | Localization | Map Source | Launch File |
|------|-------------|------------|-------------|
| **Static Map** (competition) | FAST-LIO2 ICP global relocalization | Pre-built PGM | `nav2_fastlio_static_map.launch.py` |
| **Dynamic** (exploration) | FAST-LIO2 real-time odometry | Built live | `nav2_fastlio_bringup.launch.py` |

Also an AMCL-based alternative: `nav2_task_launch.py`

### TF Tree

```
map → camera_init → (static) → odom ← FAST-LIO2 → base_link → unilidar_lidar
  ↑
  (fast_lio_localization maps ICP result via transform_fusion.py)
```

### Data Pipeline

1. **Unitree LiDAR** → `/unilidar/cloud` raw point cloud
2. **FAST-LIO2** → `/Odometry` (odom→base_link) + `/cloud_registered`
3. **global_localization.py** → ICP match vs pre-built PCD → `/map_to_odom`
4. **transform_fusion.py** → broadcast TF `map→camera_init` + `/localization`
5. **Nav2** → plan + control → `/cmd_vel`
6. **cmd_vel_chassis_serial.py** → serial frame `[0x55][0xAA][0x10][0x09][vx][wz][state][checksum]` → STM32

### Cube Detection + Arm Grasping Pipeline

FAST-LIO2 point cloud → `cube_detector.py` (DBSCAN → PCA → OBB) → `/detected_cube` Marker → `catch.py` (coordinate transform + stability filter) → serial → STM32 arm

## Essential Commands

### Environment & Dependencies
```bash
source /opt/ros/jazzy/setup.bash
pip install open3d tf_transformations
sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs python3-serial imagemagick
```

### Build Workspaces
```bash
# FAST-LIO2 workspace (selective build)
cd fastlio2_v2
rm -rf build/ install/
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh  # fix ament_python indexing
source install/setup.bash

# Nav2 workspace (full build)
cd nav2_ws1
rm -rf build/ install/
colcon build --symlink-install
source install/setup.bash
```

### Launch Sequence (Competition)
```bash
# Terminal A: LiDAR driver
cd fastlio2_v2 && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# Terminal B: FAST-LIO2 mapping
ros2 run fast_lio fastlio_mapping --ros-args --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml

# Terminal C: Global relocalization
ros2 launch fast_lio_localization 1.launch.py map:=path/to/scans.pcd config_file:=unilidar_l2.yaml rviz:=true
# Then click "2D Pose Estimate" in RViz

# Terminal D: Nav2 navigation
cd nav2_ws1 && source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py map:=src/dog_nav2_bringup/maps/scans_2d.yaml

# Terminal E: Chassis serial bridge
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py serial_port:=/dev/ttyACM0 baud_rate:=115200 cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 active_state:=1 idle_state:=0
```

### Tools (no colcon needed, run from workspace root)
```bash
source /opt/ros/jazzy/setup.bash
python3 py/cube_detector.py         # 3D OBB cube detection
python3 py/catch.py                 # Robotic arm grasp control
python3 py/pointcloud_saver.py      # Spacebar-triggered PCD save
python3 py/test_dog.py              # HSV color calibration
python3 py/move.py 1                # Chassis test (0=idle 1=fwd 2=rev 3=left 4=right 5=crouch)
python3 py/listen_serial.py         # Serial monitor
python3 py/fastlio_pose.py          # Print real-time pose from /Odometry
```

### Map Generation Pipeline
```bash
# Save PCD map (after mapping runs)
ros2 service call /map_save std_srvs/srv/Trigger

# Convert PCD → PGM grid map
ros2 launch pcd2pgm pcd2pgm_launch.py

# Or use competition scripts
bash nav2_ws1/src/dog_nav2_bringup/scripts/task_field_competition.sh
```

### Networking (LiDAR communication)
```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

### Common Pitfalls
- **Python package not found** (`fast_lio_localization`): Run `hook_fix.sh` after build then `source install/setup.bash`
- **Permission denied**: Run `find . -name "*.py" -path "*/scripts/*" -exec chmod +x {} +`
- **Serial access**: `sudo usermod -aG dialout $USER` then re-login; temp fix `sudo chmod 666 /dev/ttyACM0`
- **RViz blank map**: Check `ros2 lifecycle get /map_server` is `active`, verify TF tree with `ros2 run tf2_tools view_frames.py`

## Notable Files

- `fast_lio_localization/global_localization.py` — ICP-based relocalization node (core competition feature)
- `fast_lio_localization/transform_fusion.py` — TF bridge: merges odometry + ICP result
- `dog_nav2_bringup/scripts/cmd_vel_chassis_serial.py` — Serial protocol encoder (timeout safety, exit-safe)
- `py/cube_detector.py` — DBSCAN + PCA pipeline for 25cm cube detection in LiDAR point cloud
- `py/catch.py` — Sliding-window stability filter + coordinate transform for arm grasping
