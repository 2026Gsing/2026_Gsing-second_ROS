# 运行流程

##   配置网卡（雷达通信）

```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

##   ROS2 环境

```bash
source /opt/ros/jazzy/setup.bash
```

##   激活 Python 虚拟环境

```bash
source /home/hyper/program/ROS/ros2_jazzy_venv/bin/activate
```

##   启动雷达驱动（先开雷达电源）

首次需要编译，之后直接启动：

```bash
# 首次
cd ~/program/ROS/fastlio2_v2
colcon build --symlink-install --packages-select unitree_lidar_ros2
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py &

# 之后只需
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py &
```

##   启动 FAST-LIO2 建图

```bash
cd ~/program/ROS/fastlio2_v2
source install/setup.bash

ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml


# 另一个终端：RViz
source /opt/ros/jazzy/setup.bash
source ~/program/ROS/fastlio2_v2/install/setup.bash
rviz2 -d ROS/fastlio2_v2/src/fast_lio_config.rviz
```

##   立方体检测

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/program/ROS/py/cube_detector.py
```

##   机械臂抓取

```bash
python3 ~/program/ROS/py/catch.py
```

##   Nav2 导航

首次需要编译，之后直接启动：

```bash
# 首次
cd ~/program/ROS/nav2_ws1
colcon build --symlink-install
source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py

# 之后只需
source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py
```
