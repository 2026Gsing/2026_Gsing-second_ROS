# 运行流程（按顺序）

## 前置准备（仅首次）

### 安装系统依赖

```bash
sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs
```

### 克隆 pcd2pgm（点云转栅格地图工具）

```bash
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/src
git clone https://github.com/liuscn/pcd2pgm.git
```

### 安装 Python 依赖

```bash
pip install open3d tf_transformations
```

### 设置脚本执行权限

```bash
find /home/hyper/program/2026_Gsing-second_ROS -name "*.py" -path "*/scripts/*" -exec chmod +x {} +
```

---

## 0) 进入仓库根目录

```bash
cd /home/hyper/program/2026_Gsing-second_ROS
```

## 1) 配置网卡（雷达通信）

```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

## 2) 编译 FAST-LIO 工作空间

首次或有缓存冲突时，先清理再编译：

```bash
cd fastlio2_v2

# 首次 / 路径变更时清理旧缓存
rm -rf build/ install/

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization

# 修复 ament_python 包的索引（colcon 不自动注册纯 Python 包）
bash src/fast_lio_localization/scripts/hook_fix.sh

source install/setup.bash
```

## 3) 启动雷达驱动（终端A）

```bash
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## 4) 启动 FAST-LIO 建图（终端B）

```bash
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml
```

### 可视化（终端 B2，可选）

```bash
source /opt/ros/jazzy/setup.bash
source /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/install/setup.bash
rviz2 -d /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/src/fast_lio_config.rviz
```

## 5) 生成 2D 栅格地图（pcd2pgm，终端C）

先确认 `src/pcd2pgm/config/pcd2pgm.yaml` 中 `pcd_file` 指向你的 PCD 文件。

```bash
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch pcd2pgm pcd2pgm_launch.py
```

生成结果（默认与 PCD 同目录）：
- `pgm_map.pgm`
- `pgm_map.yaml`

## 6) 启动全局定位（fast_lio_localization，终端D）

在已建好的 PCD 地图中定位机器人位姿。

```bash
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fast_lio_localization 1.launch.py \
  map:=src/unilidar_fastlio_ros2-ros2/PCD/scans.pcd \
  config_file:=unilidar_l2.yaml \
  rviz:=true
```

## 7) 启动 Nav2 导航（终端E）

首次编译：

```bash
cd nav2_ws1
rm -rf build/ install/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

之后直接启动：

```bash
cd nav2_ws1
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=src/dog_nav2_bringup/maps/scans_2d.yaml
```

如果你想继续用已有任务地图，把 `map:=.../scans_2d.yaml` 换成 `map:=src/dog_nav2_bringup/maps/task_field_map.yaml`。

## 8) 底盘串口桥接（cmd_vel → STM32，终端F）

```bash
cd nav2_ws1
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 串口桥：把 /cmd_vel 按协议发给 STM32（0x55 0xAA 0x10）
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel \
  send_rate_hz:=50.0 \
  active_state:=1 \
  idle_state:=0
```

如果串口权限不足：

```bash
sudo usermod -aG dialout $USER
# 重新登录生效；临时可用 sudo chmod 666 /dev/ttyACM0
```

## 9) 不启动 Nav2 时，直接发送测试串口帧（可选）

```bash
python3 nav2_ws1/src/dog_nav2_bringup/scripts/send_chassis_test_serial.py \
  --port /dev/ttyACM0 \
  --baud 115200 \
  --vx 0.10 \
  --wz 0.00 \
  --state 1 \
  --rate 50 \
  --duration 2 \
  --send-stop-on-exit
```

## 10) 立方体检测 + 机械臂抓取

导航到达目标后，使用雷达点云检测立方体，控制机械臂抓取。

### 启动立方体检测节点

```bash
# 终端 G：启动 3D OBB 立方体检测
source /opt/ros/jazzy/setup.bash
python3 py/cube_detector.py
```

- 订阅 `/unilidar/cloud`（FAST-LIO2 配准点云）
- 发布 `/detected_cube` (Marker) 
- 算法：DBSCAN 聚类 → 3D PCA → OBB 包围盒 → 边长校验（25cm ± 5cm）
- 在 RViz 中添加 Marker 话题即可看到绿色半透明立方体

### 启动机械臂抓取控制

```bash
# 终端 H：启动机械臂控制节点
source /opt/ros/jazzy/setup.bash
python3 py/catch.py
```

- 订阅 `/detected_cube` 获取立方体位置
- 自动进行坐标变换（雷达系 → 机械臂系）和稳定检测
- 稳定后通过串口发送坐标给 STM32，控制机械臂抓取
- 1Hz 心跳重发机制确保下位机收到指令

### 启动点云保存工具（可选，用于数据采集）

```bash
# 终端 I：启动点云保存
source /opt/ros/jazzy/setup.bash
python3 py/pointcloud_saver.py
# 按空格键保存当前帧点云，按 Q 退出
```

### 颜色检测调参（可选，用于摄像头颜色分类）

```bash
# 终端 J：启动颜色检测调参
source /opt/ros/jazzy/setup.bash
python3 py/test_dog.py
# 按 1-4 切换颜色类别，s 保存配置，q 退出
```

### 串口通信测试

```bash
# 底盘运动测试
source /opt/ros/jazzy/setup.bash
python3 py/move.py 1   # 0=待机 1=前进 2=后退 3=左转 4=右转 5=蹲下

# 监听 STM32 串口返回数据
python3 py/listen_serial.py
```

### py/ 目录文件说明

| 文件 | 功能 | 依赖 |
|------|------|------|
| `cube_detector.py` | 3D OBB 立方体检测（雷达点云） | FAST-LIO2 点云 |
| `catch.py` | 机械臂抓取控制（串口→STM32） | cube_detector, 串口 |
| `pointcloud_saver.py` | 点云保存（空格键触发） | FAST-LIO2 点云 |
| `test_dog.py` | OpenCV 颜色检测 HSV 调参 | USB 摄像头 |
| `move.py` | 底盘指令测试工具 | 串口 |
| `listen_serial.py` | 串口数据监听 | 串口 |
| `check_gz_services.py` | Gazebo 服务检查 | Gazebo |
| `folder_summary.py` | 项目文件内容提取（GUI） | tkinter |
| `test_ros2.py` | ROS2 初始化测试 | — |
| `camera_four_colors_config.json` | 四色 HSV 范围配置 | test_dog.py 自动生成 |
