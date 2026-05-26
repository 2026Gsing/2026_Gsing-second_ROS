#!/usr/bin/env python3
"""
nav2_fastlio_bringup.launch.py — 动态建图模式下的 Nav2 导航启动

工作流程：
  1. FAST-LIO2 在建图模式 (lidar + LIO) 下同时发布 /Odometry 和点云
  2. 本 launch 启动 Nav2 全套节点（规划器、控制器、行为树导航器），
     订阅 FAST-LIO2 的里程计作为定位输入
  3. TF 树（静态发布）： camera_init → odom, body → base_link
     让 Nav2 的 map→odom→base_link 链路与 FAST-LIO2 的 TF 树衔接
  4. 不使用 map_server，依赖 FAST-LIO2 实时建图

使用方式：
  ros2 launch dog_nav2_bringup nav2_fastlio_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

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

    bringup_share = get_package_share_directory('dog_nav2_bringup')
    default_params_file = os.path.join(
        bringup_share, 'params', 'nav2_fastlio_params.yaml'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    # ============ TF 树静态变换 ============
    # FAST-LIO2 的 TF 树结构: map ← camera_init ← body
    # Nav2 期望的 TF 树结构: map ← odom ← base_link
    # 以下两个静态变换将二者桥接：
    #   camera_init → odom  (Nav2 将 odom 视为局部参考系)
    #   body → base_link    (Nav2 将 base_link 作为机器人本体框架)
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

    # ============ Nav2 核心节点 ============
    # controller_server: 执行局部路径跟踪（DWB / Regulated Pure Pursuit），输出 /cmd_vel
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # planner_server: 全局路径规划（Navfn / Smac Planner），在 costmap 上搜索路径
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # behavior_server: 处理恢复行为（旋转、后退、原地掉头等）
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # bt_navigator: 行为树导航器，通过 XML 行为树编排完整导航流程（规划→控制→恢复）
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # waypoint_follower: 航点跟随器，支持多目标点连续导航
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ============ 生命周期管理 ============
    # Nav2 节点使用生命周期管理（LifecycleNode），需要统一管理器来
    # 按顺序配置→激活所有节点，确保依赖关系正确
    lifecycle_nodes = [
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
    rviz_config = os.path.join(bringup_share, 'rviz', 'nav2_fastlio.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='nav2_rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # ============ 启动描述组装 ============
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)

    # TF 静态变换必须在 Nav2 节点之前启动，避免 TF 缓存缺失
    ld.add_action(static_tf_caminit_odom)
    ld.add_action(static_tf_body_baselink)

    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz_node)

    return ld

