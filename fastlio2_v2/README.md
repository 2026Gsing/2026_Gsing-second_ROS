# fastlio2_v2 — FAST-LIO2 集成工作空间

集成多个 ROS2 包的 FAST-LIO2 编译运行工作空间。

## 目录结构

```
src/
├── unilidar_fastlio_ros2-ros2/   FAST-LIO2 主建图包
│   ├── src/laserMapping.cpp      主建图节点
│   ├── src/IMU_Processing.hpp    IMU 预积分
│   ├── src/preprocess.cpp/h      点云预处理
│   ├── src/odometry_to_tf.cpp    里程计 TF 发布
│   ├── include/ikd-Tree/         增量式 KD-Tree
│   ├── include/IKFoM_toolkit/    流形卡尔曼滤波工具包
│   ├── config/*.yaml             多雷达配置 (AVIA, MID360, Velodyne 等)
│   ├── launch/                   启动脚本 (mapping, map_server)
│   └── msg/Pose6D.msg            六自由度位姿消息
│
├── my_odom_tf_pkg/               里程计 → TF 变换发布
│   └── src/odom_to_tf.cpp
│
├── unitree_lidar_ros2/           Unitree LiDAR ROS2 驱动
│   └── src/unitree_lidar_ros2/
│       ├── src/unitree_lidar_ros2_node.cpp
│       └── include/unitree_lidar_ros2.h
│
├── unitree_lidar_sdk/            Unitree LiDAR C++ SDK
│   ├── include/unitree_lidar_sdk.h
│   ├── lib/x86_64/libunilidar_sdk2.a
│   └── examples/                  示例 (serial, udp, ip配置)
│
└── fastlio_loop_tools/           回环检测工具（预留）
```

## 构建

```bash
colcon build --symlink-install
source install/setup.bash
```
