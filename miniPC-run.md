# 运行指南（机载 MiniPC 版）

基于 `README.md`，路径适配本机，统一相对路径。

---

## 1. 环境依赖

```bash
# ROS2
source /opt/ros/jazzy/setup.bash

# CycloneDDS（必须，所有终端都要）
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 每个终端都要设

# 串口权限
sudo usermod -aG dialout $USER
# 重新登录生效，或临时：sudo chmod 666 /dev/ttyACM0
```

---

## 2. 构建

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# FAST-LIO2 工作空间
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh
source install/setup.bash

# Nav2 工作空间
cd ../nav2_ws1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# ⚠️ 编译后手动同步参数文件（install/ 下是副本）：
cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
   install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

---

## 3. 网卡配置（LiDAR 通信）

```bash
# 查网口名称
ip a | grep enp

# 配置 LiDAR 网口
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

---

## 4. 一键启动

所有节点由 `launch_utils.py` 自动拉起，自动输出日志到 `logs/` 目录。

### 物资箱自动抓取

```bash
cd 2026_Gsing-second_ROS
./ros-run.sh py/control/box_pick_node.py
```

功能：
- 自动拉起 LiDAR、ICP、TF桥、Nav2、串口桥
- 8s 后自动初始化 ICP（0,0,0 X正向）
- 订阅 `/localization` 检测到达（速度归零 20 帧 ≈ 2s）
- 到达后启动 `cube_detector` 检测物资箱
- 可达则启动 `catch.py` 抓取，不可达则 Nav2 导航靠近

### 竞赛全自动

```bash
./ros-run.sh py/control/auto_task.py field_id:=1
```

### 建图

```bash
./ros-run.sh py/tools/map_scan.py
./ros-run.sh py/tools/map_scan.py --no-rviz
```

按 Enter 保存 PCD + 自动 PCD→PGM 转换，输出 `logs/` 有点云数量监控。

### 底盘测试

```bash
./ros-run.sh py/tools/test_move.py           # 交互模式
./ros-run.sh py/tools/test_move.py --auto    # 自动序列
```

自动启动串口桥（使用 `launch_utils`，与 box_pick_node 一致），显示定位和串口反馈。

---

## 5. 地图配置

在 `py/control/launch_utils.py` 开头修改：

```python
MAP_NAME = "map/map"       # → map/map.pcd + map/map.yaml（默认）
MAP_NAME = "map/PCD13"     # → map/PCD13.pcd + map/PCD13.yaml
```

---

## 6. 开关

`py/control/launch_utils.py` 开头：

| 参数 | 默认 | 说明 |
|------|------|------|
| `ENABLE_RVIZ` | `True` | 是否打开 RViz 可视化 |
| `USE_TERMINAL` | `True` | 是否用独立终端窗口显示每个节点输出 |
| `SERIAL_PORT` | `/dev/ttyACM0` | STM32 串口设备路径 |
| `MAP_NAME` | `map/map` | 地图文件名（不含扩展名） |

---

## 7. 实时调参

```bash
# Nav2 速度
ros2 param set /controller_server FollowPath.desired_linear_vel 1.0
ros2 param set /controller_server FollowPath.max_linear_vel 1.2
ros2 param set /controller_server FollowPath.lookahead_dist 0.8

# ICP 定位阈值
ros2 param set /global_localization localization_threshold 0.8
```

---

## 8. 日志

所有进程输出 → `logs/YYYY-MM-DD_HHMMSS_节点名.log`

```bash
# 查看日志目录
ls -lh logs/

# 实时看某个节点
tail -f logs/*_LiDAR.log

# 找报错
grep -l "ERROR\|Traceback" logs/*.log | xargs tail -20
```

---

## 9. 手动分步启动

如果不用一键启动，每步单独终端：

```bash
# 终端 1: LiDAR 驱动
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd fastlio2_v2 && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# 终端 2: ICP 定位
cd fastlio2_v2 && source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH"
export PYTHONPATH=$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages
ros2 launch fast_lio_localization 1.launch.py \
  map:=map/map.pcd config_file:=unilidar_l2.yaml rviz:=true \
  map_voxel_size:=0.01 scan_voxel_size:=0.03 \
  freq_localization:=2.0 localization_threshold:=0.9

# 终端 2b: odometry→TF 桥
cd fastlio2_v2 && source install/setup.bash
./build/fast_lio/odometry_to_tf

# 终端 3: Nav2
cd nav2_ws1 && source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=map/map.yaml

# 终端 4: 串口桥
cd nav2_ws1 && source install/setup.bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 \
  active_state:=1 idle_state:=0 \
  leg_debug_csv_path:=logs/leg_debug.csv \
  leg_debug_log_period_sec:=0.5
```

---

## 10. 常用排查

```bash
# TF 树
ros2 run tf2_tools view_frames.py

# 话题列表
ros2 topic list

# 定位数据
ros2 topic echo /localization --once

# 串口桥日志（腿部调试）
tail -f logs/*_串口桥.log | grep LEG_DEBUG
```
