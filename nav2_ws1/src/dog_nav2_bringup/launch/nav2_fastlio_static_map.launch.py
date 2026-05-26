#!/usr/bin/env python3
"""
nav2_fastlio_static_map.launch.py — 静态地图模式下的 Nav2 导航启动（竞赛主用）

工作流程：
  1. map_server 加载预先构建的 2D 栅格地图（PGM + YAML）
  2. FAST-LIO2 Localization 提供 map→camera_init 的实时位姿估计，
     transform_fusion.py 将其发布为 /localization 和 TF
  3. Nav2 的 global_costmap 使用 static_layer 加载预存地图，
     local_costmap 以 odom 为参考系做局部规划
  4. goal_pose_to_nav2.py 桥接：RViz 的 "2D Goal Pose" 点击 →
     NavigateToPose action 发送给 bt_navigator
  5. costmap_to_grid.py 将 costmap 重发布用于 RViz 可视化
  6. chassis_serial_bridge 将 /cmd_vel 通过串口发给下位机 STM32

与动态建图模式 (nav2_fastlio_bringup.launch.py) 的区别：
  - 使用 map_server + static_layer，不依赖实时建图
  - TF 树自动由 fast_lio_localization 发布 map→odom 变换
  - 启动了辅助脚本（goal_pose 桥接、costmap 转发）
  - Nav2 节点延迟 3 秒启动，确保 map_server 先就绪

使用方式：
  ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py \
    map:=src/dog_nav2_bringup/maps/scans_2d.yaml
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    bringup_share = get_package_share_directory('dog_nav2_bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    default_params_file = os.path.join(
        bringup_share, 'params', 'nav2_fastlio_static_map_params.yaml'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 params yaml'
    )

    default_map_yaml = os.path.join(bringup_share, 'maps', 'map.yaml')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Full path to the map yaml to load'
    )

    # ============ TF 树静态变换 ============
    # FAST-LIO2 的 TF 树: map ← camera_init ← body
    # Nav2 期望的 TF 树:   map ← odom ← base_link
    # fast_lio_localization 会动态发布 map → camera_init（即 map → odom）
    # 因此这里只需补齐：
    #   camera_init → odom       (恒等变换，名称桥接)
    #   body → base_link          (恒等变换，名称桥接)
    #   base_link → unilidar_lidar (激光雷达框架)
    static_tf_caminit_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_caminit_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'odom'],
        output='screen'
    )

    static_tf_body_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_baselink',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],
        output='screen'
    )

    # 激光雷达框架挂载到 base_link 下（恒等变换，用于 costmap 传感器层）
    static_tf_baselink_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_baselink_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar_lidar'],
        output='screen'
    )

    # ============ 地图服务器 ============
    # 加载预存的 PGM 栅格地图，通过 static_layer 提供给 global_costmap
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )

    # ============ Nav2 核心节点（延迟启动）============
    # controller_server: 局部路径跟踪控制器，根据全局路径和局部 costmap 计算 /cmd_vel
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # planner_server: 全局路径规划器，在 global_costmap 上搜索从起点到目标点的路径
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # behavior_server: 恢复行为服务器（卡住时旋转、后退等）
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # bt_navigator: 行为树导航器，按 "规划→控制→检查→恢复" 的行为树驱动导航流程
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # waypoint_follower: 多航点连续导航
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ============ 生命周期管理器 ============
    # map_server 也包含在生命周期管理中，确保地图在导航节点之前加载
    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    # ============ RViz2 可视化 ============
    rviz_config = os.path.join(bringup_share, 'rviz', 'nav2_fastlio_static_map.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='nav2_rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # ============ 辅助脚本 ============

    # goal_pose_to_nav2.py: 桥接 RViz 的 "2D Goal Pose" 点击
    #   订阅 /goal_pose (PoseStamped) → 发送 NavigateToPose action 给 bt_navigator
    pkg_prefix = get_package_prefix('dog_nav2_bringup')
    goal_bridge_script = os.path.join(pkg_prefix, 'lib', 'dog_nav2_bringup', 'goal_pose_to_nav2.py')
    goal_pose_bridge = ExecuteProcess(
        cmd=[sys.executable, goal_bridge_script],
        name='goal_pose_to_nav2',
        output='screen',
    )

    # costmap_to_grid.py: 将 Nav2 costmap 话题重发布为 /global_costmap_grid 和
    #   /local_costmap_grid，使 RViz 可以直接显示 costmap（因为原始 costmap 话题
    #   使用 transient_local QoS，部分 RViz 配置可能订阅不到）
    costmap_grid_script = os.path.join(pkg_prefix, 'lib', 'dog_nav2_bringup', 'costmap_to_grid.py')
    costmap_grid_bridge = ExecuteProcess(
        cmd=[sys.executable, costmap_grid_script],
        name='costmap_to_grid',
        output='screen',
    )

    # ============ 启动描述组装 ============
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    ld.add_action(declare_map)

    # TF 变换先启动
    ld.add_action(static_tf_caminit_odom)
    ld.add_action(static_tf_body_baselink)
    ld.add_action(static_tf_baselink_lidar)

    # map_server 先启动（无延迟）
    ld.add_action(map_server)

    # Nav2 核心节点 + 辅助脚本延迟 3 秒启动，原因：
    #   1. map_server 需要时间加载地图并发布 /map 话题
    #   2. FAST-LIO2 localization 需要时间建立初始 TF 关系
    #   3. 避免 Nav2 节点在 TF 缓存缺失时频繁报错
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[
                controller_server,
                planner_server,
                behavior_server,
                bt_navigator,
                waypoint_follower,
                lifecycle_manager,
                goal_pose_bridge,
                costmap_grid_bridge,
            ],
        )
    )
    ld.add_action(rviz_node)

    return ld

