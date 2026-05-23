# 2026 Gsing 二队 — ROS2 Jazzy 工作空间

Gsing 二队竞赛用 ROS2 Jazzy 整合仓库，包含 FAST-LIO2 SLAM、Unitree 雷达驱动、全局重定位、Nav2 导航、四足仿真及工具脚本。

## 环境要求

- **OS**: Ubuntu 24.04
- **ROS2**: Jazzy
- **Python**: 3.12

## 目录结构

```
2026_Gsing-second_ROS/
├── fastlio2_v2/                  # 主工作空间：FAST-LIO2 SLAM + 雷达驱动 + 重定位
│   └── src/
│       ├── unilidar_fastlio_ros2-ros2/   FAST-LIO2 核心
│       ├── fast_lio_localization/        全局重定位
│       ├── unitree_lidar_ros2/           雷达驱动
│       ├── unitree_lidar_sdk/            SDK
│       ├── my_odom_tf_pkg/               TF 变换
│       └── fastlio_loop_tools/           回环检测（预留）
├── nav2_ws1/                     # Nav2 导航（含竞赛地图与脚本）
├── 2026_Gsing_Dog_simulation-main/ 四足狗 Gazebo 仿真
├── py/                           # 工具脚本（机械臂控制、立方体检测等）
```


详细运行流程（网卡配置 → 雷达驱动 → FAST-LIO2 建图 → 检测 → 抓取 → 导航）见 [run.md](run.md)。
