# nav2_ws1 — Nav2 导航启动工作空间

Navigation2 导航系统启动包，集成 FAST-LIO2 里程计与底盘串口控制。

## 目录结构

```
src/dog_nav2_bringup/
├── launch/
│   ├── nav2_fastlio_bringup.launch.py       FAST-LIO + Nav2 联合启动
│   ├── nav2_fastlio_static_map.launch.py     静态地图导航模式
│   └── chassis_serial_bridge.launch.py       底盘串口桥接
│
├── params/
│   ├── nav2_fastlio_params.yaml              Nav2 参数（动态建图）
│   └── nav2_fastlio_static_map_params.yaml   Nav2 参数（静态地图）
│
├── maps/
│   ├── scans.pcd.yaml / scans.pcd.pgm        建图导航地图
│   └── scans_new.pgm / scans_new.yaml        更新后的地图
│
├── scripts/
│   ├── cmd_vel_chassis_serial.py       速度指令 → 底盘串口转发
│   ├── goal_pose_to_nav2.py            目标点发布
│   ├── costmap_to_grid.py              代价地图转栅格
│   └── send_chassis_test_serial.py     底盘串口通信测试
│
├── rviz/
│   ├── nav2_fastlio.rviz               SLAM 模式可视化
│   └── nav2_fastlio_static_map.rviz    静态地图模式可视化
│
└── docs/CHASSIS_SERIAL.md              底盘串口协议说明
```

## 运行

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py
```
