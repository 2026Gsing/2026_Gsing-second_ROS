# ROS

hyper 的 ROS2 Jazzy 工作空间整合仓库。

## 目录结构

| 目录 | 说明 |
|------|------|
| `fastlio2_v2/` | 主工作空间：FAST-LIO2 SLAM + Unitree 雷达驱动 + 全局重定位 |
| `nav2_ws1/` | Nav2 导航启动工作空间（含竞赛用地图和脚本） |
| `2026_Gsing_Dog_simulation-main/` | 四足狗 Gazebo 仿真项目 |
| `py/` | 独立 Python 工具脚本（机械臂控制、立方体检测等） |
| `ros2_jazzy_venv/` | Python 虚拟环境 |


2026_Gsing-second_ROS/
├── fastlio2_v2/                    # 主工作空间 (SLAM + 雷达 + 定位)
│   └── src/
│       ├── unilidar_fastlio_ros2-ros2/   FAST-LIO2 核心
│       ├── fast_lio_localization/        全局重定位 (新增)
│       ├── unitree_lidar_ros2/           雷达驱动
│       ├── unitree_lidar_sdk/            SDK
│       ├── my_odom_tf_pkg/               TF 变换
│       └── fastlio_loop_tools/           回环检测 (预留)
├── nav2_ws1/                       # Nav2 导航
├── 2026_Gsing_Dog_simulation-main/ # 仿真
├── py/                             # 独立工具脚本
└── ros2_jazzy_venv/                # Python 虚拟环境