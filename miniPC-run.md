# 运行指南（机载 MiniPC 版）

基于 `README.md`，路径适配本机，统一相对路径。
**本机主机名 `gsing`** — `launch_utils.py` 根据主机名自动控制 RViz 默认开关。
**终端提示符**: `(base) gsing@gsing:~/2026Gsing$` — 以下命令均在此工作目录执行。

**最后更新**: 2026-07-14

---

## 1. 本机环境确认

| 项目 | 值 |
|------|-----|
| 主机名 | `gsing` |
| OS | Ubuntu 24.04 Noble |
| ROS2 | Jazzy (apt 源 `mirrors.tuna.tsinghua.edu.cn`) |
| LiDAR 网口 | **`enp4s0`** |
| STM32 串口 | `/dev/ttyACM0`（未连接时跳过串口桥） |
| 终端 | `gnome-terminal` ✓ |
| Workspaces | `fastlio2_v2` + `nav2_ws1` 均已构建 |

### 已安装 Python 包（系统 Python 3.12）

| 包 | 版本 | 方式 |
|----|------|------|
| numpy | 1.26.4 | 系统预装 |
| opencv | 4.6.0 | `apt install python3-opencv` |
| pyserial | 3.5 | `apt install python3-serial` |
| pyyaml | 6.0.1 | `apt install python3-yaml` |
| ultralytics | 8.4.84 | `pip install ultralytics` → `~/.local/lib/python3.12/` |

### ⚠️ 缺失依赖（首次需安装）

```bash
sudo apt install python3-sklearn
```

---

## 2. 环境准备（每个新终端执行）

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
sudo chmod 666 /dev/ttyACM0    # STM32 串口权限，设备存在时执行
```

---

## 3. 构建

两个 workspace 已预构建，换机器 clone 后需重新编译：

```bash
cd 2026_Gsing-second_ROS

# FAST-LIO2
cd fastlio2_v2 && source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install \
    --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization && \
  bash src/fast_lio_localization/scripts/hook_fix.sh

# Nav2（回到 2026_Gsing-second_ROS 下）
cd ../nav2_ws1 && source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install && \
  cp src/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml \
     install/dog_nav2_bringup/share/dog_nav2_bringup/params/nav2_fastlio_static_map_params.yaml
```

---

## 4. LiDAR 网卡配置

每次重启后执行：

```bash
sudo nmcli device set enp4s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp4s0
```

验证：

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /unilidar/cloud --once
```

---

## 5. 一键启动

所有节点由 `launch_utils.py` 自动拉起，日志输出到 `2026_Gsing-second_ROS/logs/{category}/`。

```bash
cd 2026_Gsing-second_ROS   # ← 先进入项目目录
```

### 5.1 竞赛全自动

```bash
./ros-run.sh py/control/auto_task.py
```

启动流程：
1. 交互输入**场地号**（1 或 2）和**内排顺序**（4 个物资类型编号，空格分隔）
2. 自动拉起 LiDAR → ICP → TF桥 → Nav2 → 串口桥
3. 8s 后自动初始化 ICP（0,0,0 X正向）
4. 输入 `start` 开始比赛，`estop` 急停，`quit` 退出
5. 状态机：`IDLE → SOLVE_TASK → NAV_BOX → WAIT_PICK → NAV_ZONE → WAIT_PLACE → NEXT_OR_FINISH`

### 5.2 物资箱手动抓取

```bash
./ros-run.sh py/control/box_pick_node.py
```

在 RViz 中用 2D Nav Goal 导航 → 自动到达检测（距离 < 0.15m）→ 检测+抓取。  
输入 `arrived` 强制触发到达。

### 5.3 建图

```bash
./ros-run.sh py/tools/map_scan.py
./ros-run.sh py/tools/map_scan.py --no-rviz
```

按 Enter 保存 PCD + 自动 PCD→PGM 转换。

### 5.4 底盘测试

```bash
./ros-run.sh py/tools/test_move.py              # 交互模式
./ros-run.sh py/tools/test_move.py --auto       # 自动序列
```

自动启动串口桥 + 发送 `AUTO_CMD_START`。

---

## 6. RViz 控制

**本机默认不启动 RViz**（主机名 `gsing` 自动识别），可通过环境变量临时覆盖：

```bash
GSING_RVIZ=1 ./ros-run.sh py/control/auto_task.py     # 强制打开
GSING_RVIZ=0 ./ros-run.sh py/control/box_pick_node.py  # 强制关闭
```

---

## 7. 赛前配置

### 地图选择

`2026_Gsing-second_ROS/py/control/launch_utils.py` 顶部：

```python
MAP_NAME = "map/map"       # 标准比赛地图
```

可用地图（`ls map/*.yaml` 查看完整列表）：

| 文件 | 说明 |
|------|------|
| `map/map` | 标准比赛地图 |
| `map/PCD17` | 最近建图记录（编号递增） |

### 比赛参数

`2026_Gsing-second_ROS/config/competition.yaml` — 修改场地布局、超时、机械臂参数。

---

## 8. 日志

```
2026_Gsing-second_ROS/logs/
├── lidar/    → unitree_lidar_ros2
├── slam/     → FAST-LIO2, ICP, TF桥
├── nav2/     → Nav2 各节点, RViz
├── serial/   → 串口桥, 腿部调试 CSV
└── arm/      → catch.py 抓取日志
```

```bash
# 在 2026_Gsing-second_ROS 目录下
cd 2026_Gsing-second_ROS
tail -f logs/lidar/*_LiDAR.log              # LiDAR 实时
grep -l "ERROR\|Traceback" logs/*/*.log     # 查报错
```

---

## 9. 实时调参

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 param set /controller_server FollowPath.desired_linear_vel 0.8
ros2 param set /global_localization localization_threshold 0.8
```

---

## 10. 分步启动（排查用）

每步一个独立终端，全部 `start_rviz:=false`：

```bash
# ===== 终端 1: LiDAR =====
cd ~/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py start_rviz:=false

# ===== 终端 2: ICP 定位 =====
cd ~/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH"
export PYTHONPATH=$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages
ros2 launch fast_lio_localization 1.launch.py \
  map:=map/map.pcd config_file:=unilidar_l2.yaml rviz:=false \
  map_voxel_size:=0.01 scan_voxel_size:=0.03 \
  freq_localization:=2.0 localization_threshold:=0.9

# ===== 终端 2b: TF桥 =====
cd ~/2026Gsing/2026_Gsing-second_ROS/fastlio2_v2
source install/setup.bash
./build/fast_lio/odometry_to_tf

# ===== 终端 3: Nav2 =====
cd ~/2026Gsing/2026_Gsing-second_ROS/nav2_ws1
source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=map/map.yaml start_rviz:=false

# ===== 终端 4: 串口桥（STM32 已连接时）=====
cd ~/2026Gsing/2026_Gsing-second_ROS/nav2_ws1
source install/setup.bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 baud_rate:=115200
```

---

## 11. 常用排查

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run tf2_tools view_frames.py                  # TF 树
ros2 topic echo /localization --once               # 定位数据
ros2 topic echo /vision/auto_task_state            # 比赛状态

cd ~/2026Gsing/2026_Gsing-second_ROS
tail -f logs/serial/*_串口桥.log | grep LEG_DEBUG  # 腿部调试

# CycloneDDS 共享内存清理（启动报 participant 满时）
rm -f /dev/shm/*cyclone* /dev/shm/*dds*
```
