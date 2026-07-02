# 运行流程

## 首次准备

```bash
# 系统依赖
sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs
pip install -r requirements.txt

# 点云转栅格地图工具
cd fastlio2_v2/src
git clone https://github.com/liuscn/pcd2pgm.git
cd ../..

# 执行权限
find . -name "*.py" -path "*/scripts/*" -exec chmod +x {} +
```

## 构建工作空间

```bash
# FAST-LIO2
cd fastlio2_v2
rm -rf build/ install/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh
source install/setup.bash

# Nav2
cd ../nav2_ws1
rm -rf build/ install/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 竞赛启动（终端 A~F 按顺序）

```bash
# ===== 终端 A：LiDAR 通信网卡 =====
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0

# ===== 终端 A：雷达驱动 =====
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# ===== 终端 B：FAST-LIO2 建图 =====
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml

# ===== 终端 B（可选）：RViz 可视化 =====
source /opt/ros/jazzy/setup.bash
source fastlio2_v2/install/setup.bash
rviz2 -d fastlio2_v2/src/fast_lio_config.rviz

# ===== 终端 B3（建图后）：保存 PCD =====
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 service call /map_save std_srvs/srv/Trigger

# ===== 终端 C：PCD → PGM 栅格地图 =====
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# 先确认 fastlio2_v2/src/pcd2pgm/config/pcd2pgm.yaml 中 pcd_file 指向你的 PCD
ros2 launch pcd2pgm pcd2pgm_launch.py

# ===== 终端 D：全局定位（ICP） =====
cd fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH"
export PYTHONPATH=$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages
ros2 launch fast_lio_localization 1.launch.py \
  map:=src/unilidar_fastlio_ros2-ros2/PCD/scans.pcd \
  config_file:=unilidar_l2.yaml rviz:=true \
  map_voxel_size:=0.01 scan_voxel_size:=0.03 \
  freq_localization:=2.0 localization_threshold:=0.9
# → 在 RViz 中点击 "2D Pose Estimate" 初始化

# ===== 终端 E：Nav2 导航 =====
cd nav2_ws1
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
  map:=src/dog_nav2_bringup/maps/task_field_map.yaml

# ===== 终端 F：串口桥（0x10 底盘 + 0x15 任务事件） =====
cd nav2_ws1
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 \
  active_state:=1 idle_state:=0

# ===== 终端 F2：视觉自动任务状态机 =====
source /opt/ros/jazzy/setup.bash
source nav2_ws1/install/setup.bash
python3 py/vision_auto_task_node.py
# 终端输入 start 开始，estop 急停，quit 退出

# ===== 终端 F3（可选）：YOLO 物资检测 =====
source /opt/ros/jazzy/setup.bash
python3 vision/src/predict.py --weights vision/weights/task3.pt --source 1 --draw-roi
# 数学符号识别
python3 vision/src/MATH.PY
```

## 一键启动

```bash
cd /home/hyper/program/2026_Gsing-second_ROS
bash run_auto_task.sh
# 另开终端启动 YOLO 检测：
cd vision && python3 src/predict.py --weights weights/task3.pt --source 1 --draw-roi
```

## 工具命令

```bash
source /opt/ros/jazzy/setup.bash

# 雷达立方体检测
python3 py/cube_detector.py

# 机械臂抓取控制
python3 py/catch.py

# 串口监听
python3 py/listen_serial.py

# 底盘运动测试（0=待机 1=前进 2=后退 3=左转 4=右转 5=蹲下）
python3 py/move.py 1

# 串口测试帧发送
python3 nav2_ws1/src/dog_nav2_bringup/scripts/send_chassis_test_serial.py \
  --port /dev/ttyACM0 --baud 115200 --vx 0.10 --wz 0.00 \
  --state 1 --rate 50 --duration 2 --send-stop-on-exit

# 实时位姿输出
python3 py/fastlio_pose.py

# 点云保存（空格键触发）
python3 py/pointcloud_saver.py

# HSV 颜色检测调参
python3 py/test_dog.py

# 竞赛地图生成
bash nav2_ws1/src/dog_nav2_bringup/scripts/task_field_competition.sh
```

## 串口权限

```bash
sudo usermod -aG dialout $USER   # 重登录生效
sudo chmod 666 /dev/ttyACM0      # 临时
```
