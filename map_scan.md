# 扫描建图

## 首次准备

```bash
# 安装 CycloneDDS
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
ros2 daemon stop && ros2 daemon start

# 构建
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization
bash src/fast_lio_localization/scripts/hook_fix.sh
source install/setup.bash
```

## 网卡配置（首次或重启后）

```bash
sudo nmcli device set enp129s0 managed no
sudo ip addr add 192.168.1.2/24 dev enp129s0
```

## 终端 1 - LiDAR 驱动

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## 终端 2 - FAST-LIO2 SLAM 建图（回车保存并退出）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
bash auto_map_save.sh
```

## 终端 2b（可选）- RViz 可视化

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/install/setup.bash
rviz2 -d /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/src/fast_lio_config.rviz
```

## 终端 3 - PCD → PGM 栅格地图转换

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

## 工具

```bash
# 查看 PCD 点云
pcl_viewer /home/hyper/program/2026_Gsing-second_ROS/map/scans.pcd
```
