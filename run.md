# 运行流程（按顺序）

## 0) 进入仓库根目录

```bash
cd .
```

## 1) 配置网卡（雷达通信）

```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

## 2) 进入 FAST-LIO 工作空间并编译必要包

```bash
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
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

## 5) 生成 2D 栅格地图（pcd2pgm，终端C）

先确认 `src/pcd2pgm/config/pcd.yaml` 中 `file_directory` 与 `file_name` 指向你的 PCD（默认 `scans.pcd`）。

```bash
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 把生成的地图直接保存到 nav2_ws1 的 maps 目录
ros2 launch pcd2pgm pcd2pgm.launch.py \
  save_prefix:=../nav2_ws1/src/dog_nav2_bringup/maps/scans_2d \
  save_delay:=8.0
```

生成结果会在：
- `../nav2_ws1/src/dog_nav2_bringup/maps/scans_2d.pgm`
- `../nav2_ws1/src/dog_nav2_bringup/maps/scans_2d.yaml`

## 6) 启动 fast_lio_localization 的 `1.launch.py`（终端D）

```bash
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fast_lio_localization 1.launch.py \
  map:=src/unilidar_fastlio_ros2-ros2/PCD/scans.pcd \
  config_file:=unilidar_l2.yaml \
  rviz:=true
```

## 7) 启动 nav2_ws1 导航（终端E）

```bash
cd nav2_ws1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=src/dog_nav2_bringup/maps/scans_2d.yaml
```

如果你想继续用已有任务地图，把 `map:=.../scans_2d.yaml` 换成 `map:=src/dog_nav2_bringup/maps/task_field_map.yaml`。

## 8) 底盘数据传输指令（cmd_vel -> 串口，终端F）

```bash
cd nav2_ws1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
sudo apt install -y python3-serial

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
cd nav2_ws1
python3 src/dog_nav2_bringup/scripts/send_chassis_test_serial.py \
  --port /dev/ttyACM0 \
  --baud 115200 \
  --vx 0.10 \
  --wz 0.00 \
  --state 1 \
  --rate 50 \
  --duration 2 \
  --send-stop-on-exit
```

## 10) 立方体检测以及通信

```bash
python py\cube_detector.py

python py\catch.py
```