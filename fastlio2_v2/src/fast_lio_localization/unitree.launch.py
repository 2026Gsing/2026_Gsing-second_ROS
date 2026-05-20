#!/usr/bin/env python3
"""
宇树 L2 雷达 Fast-LIO-Localization 启动文件
优化：移除 pcd_to_pointcloud 节点，使用 Fast-LIO 自带地图
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):
    """设置启动参数和节点"""
    
    # 获取参数
    map_file = LaunchConfiguration('map_file').perform(context)
    rviz_enable = LaunchConfiguration('rviz_enable').perform(context).lower() == 'true'
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # 检查地图文件是否存在
    if not os.path.exists(map_file):
        print(f"警告: 地图文件 {map_file} 不存在，请检查路径！")
    
    # 1. Fast-LIO 定位节点（核心）
    fastlio_mapping = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            config_file,
            {'map_file_path': map_file},
        ],
        remappings=[
            ('/livox/lidar', LaunchConfiguration('lidar_topic')),
            ('/livox/imu', LaunchConfiguration('imu_topic')),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    # 2. 全局重定位节点
    global_localization = Node(
        package='fast_lio_localization',
        executable='global_localization.py',
        name='global_localization',
        output='screen',
        parameters=[config_file],
    )
    
    # 3. 变换融合节点
    transform_fusion = Node(
        package='fast_lio_localization',
        executable='transform_fusion.py',
        name='transform_fusion',
        output='screen',
    )
    
    # 节点列表：只包含3个核心节点
    # 注意：移除了 map_publisher_node
    nodes = [fastlio_mapping, global_localization, transform_fusion]
    
    if rviz_enable:
        # 加载默认的 RViz 配置
        rviz_config = PathJoinSubstitution([
            get_package_share_directory('fast_lio_localization'),
            'rviz',
            'localization.rviz'
        ])
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz_enable'))
        )
        nodes.append(rviz_node)
    
    return nodes

def generate_launch_description():
    """生成启动描述"""
    
    # 定义启动参数
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=TextSubstitution(text='map.pcd'),
        description='全局地图文件路径 (.pcd 格式)'
    )
    
    rviz_enable_arg = DeclareLaunchArgument(
        'rviz_enable',
        default_value='true',
        description='是否启动 RViz 可视化界面'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('fast_lio_localization'),
            'config',
            'unilidar_l2.yaml'
        ]),
        description='Fast-LIO 配置文件路径'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/unilidar/cloud',
        description='宇树 L2 雷达点云话题名称'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/unilidar/imu',
        description='宇树 L2 IMU 话题名称'
    )
    
    return LaunchDescription([
        map_file_arg,
        rviz_enable_arg,
        config_file_arg,
        lidar_topic_arg,
        imu_topic_arg,
        OpaqueFunction(function=launch_setup)
    ])