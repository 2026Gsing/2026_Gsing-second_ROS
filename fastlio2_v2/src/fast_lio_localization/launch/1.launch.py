#!/usr/bin/env python3
"""
宇树 L2 雷达 Fast-LIO 定位启动文件
基于 localization.launch.py 结构，适配宇树 L2 雷达
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.parameter_descriptions import Parameter
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    package_path = get_package_share_directory("fast_lio_localization")
    
    # 默认文件路径
    default_config_path = os.path.join(package_path, "config")
    default_config_file = os.path.join(default_config_path, "unilidar_l2.yaml")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")
    default_map_path = os.path.join(package_path, "map.pcd")
    
    # 启动参数
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")
    
    # 检查默认文件是否存在
    if not os.path.exists(default_map_path):
        default_map_path = "PCD/scans.pcd"
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="false", 
        description="使用仿真(Gazebo)时钟"
    )
    
    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path", 
        default_value=TextSubstitution(text=default_config_path), 
        description="YAML配置文件路径"
    )
    
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", 
        default_value="unilidar_l2.yaml",  # 修改为宇树配置文件
        description="配置文件名称"
    )
    
    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", 
        default_value="true", 
        description="使用RViz监控结果"
    )
    
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", 
        default_value=TextSubstitution(text=default_rviz_config_path), 
        description="RViz配置文件路径"
    )
    
    declare_map_path = DeclareLaunchArgument(
        "map", 
        default_value=TextSubstitution(text=default_map_path),  # 使用检查后的路径
        description="PCD地图文件路径"
    )
    
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic",
        default_value="/cloud_pcd",
        description="发布PCD地图的话题名称"
    )

    declare_map_voxel_size = DeclareLaunchArgument(
        "map_voxel_size",
        default_value="0.1",
        description="地图体素下采样大小 (m)，越小地图点越多"
    )
    declare_scan_voxel_size = DeclareLaunchArgument(
        "scan_voxel_size",
        default_value="0.1",
        description="扫描体素下采样大小 (m)"
    )
    declare_localization_threshold = DeclareLaunchArgument(
        "localization_threshold",
        default_value="0.3",
        description="ICP 配准 fitness 阈值，高于此值才更新位姿"
    )
    declare_freq_localization = DeclareLaunchArgument(
        "freq_localization",
        default_value="1.0",
        description="ICP 定位频率 (Hz)"
    )
    declare_freq_global_map = DeclareLaunchArgument(
        "freq_global_map",
        default_value="0.25",
        description="地图发布频率 (Hz)"
    )
    declare_fov = DeclareLaunchArgument(
        "fov",
        default_value="6.28319",
        description="FOV 范围 (弧度)，2π=全向"
    )
    declare_fov_far = DeclareLaunchArgument(
        "fov_far",
        default_value="300",
        description="FOV 最远距离 (m)"
    )

    # 1. Fast-LIO 定位节点（关键修改：添加重映射）
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_path, config_file]), 
            {"use_sim_time": use_sim_time}
        ],
        # 关键修改：重映射话题，适配宇树雷达
        remappings=[
            ("/livox/lidar", "/unilidar/cloud"),  # 宇树雷达点云话题
            ("/livox/imu", "/unilidar/imu"),      # 宇树IMU话题
        ],
    )
    
    # 2. 全局重定位节点
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization",
        name="global_localization",
        output="screen",
        prefix="env PATH=/usr/bin:/bin:/usr/sbin:/sbin",
        parameters=[
            Parameter("map_voxel_size", LaunchConfiguration("map_voxel_size"), value_type=float),
            Parameter("scan_voxel_size", LaunchConfiguration("scan_voxel_size"), value_type=float),
            Parameter("freq_localization", LaunchConfiguration("freq_localization"), value_type=float),
            Parameter("freq_global_map", LaunchConfiguration("freq_global_map"), value_type=float),
            Parameter("localization_threshold", LaunchConfiguration("localization_threshold"), value_type=float),
            Parameter("fov", LaunchConfiguration("fov"), value_type=float),
            Parameter("fov_far", LaunchConfiguration("fov_far"), value_type=int),
            {"pcd_map_path": pcd_map_path},
            {"pcd_map_topic": pcd_map_topic},
        ],
    )
    
    # 3. 变换融合节点
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion",
        name="transform_fusion",
        prefix="env PATH=/usr/bin:/bin:/usr/sbin:/sbin",
        output="screen",
    )
    
    # 4. PCD 到 PointCloud2 发布器（关键修改：适配宇树）
    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{
            "file_name": pcd_map_path,
            "tf_frame": "map",
            "cloud_topic": pcd_map_topic,
            "period_ms_": 1000,
            # 添加降采样参数，防止大文件崩溃
            "leaf_size": 0.1,  # 0.1米降采样
        }],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )
    
    # 5. RViz 节点
    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        arguments=["-d", rviz_cfg], 
        condition=IfCondition(rviz_use)
    )
    
    # 6. 宇树雷达驱动节点（可选，如果雷达驱动已单独运行）
    # 如果需要在此启动文件中启动宇树雷达驱动，取消注释以下代码
    # unitree_lidar_node = Node(
    #     package="unitree_lidar",
    #     executable="unitree_lidar_ros2_node",
    #     name="unitree_lidar",
    #     output="screen",
    #     parameters=[{
    #         "frame_id": "unilidar_lidar",
    #         "lidar_ip": "192.168.123.10",  # 宇树L2默认IP
    #         "point_cloud_port": 2368,
    #         "imu_port": 10110,
    #     }],
    # )
    
    # 构建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_topic)
    ld.add_action(declare_map_voxel_size)
    ld.add_action(declare_scan_voxel_size)
    ld.add_action(declare_localization_threshold)
    ld.add_action(declare_freq_localization)
    ld.add_action(declare_freq_global_map)
    ld.add_action(declare_fov)
    ld.add_action(declare_fov_far)
    
    # 添加节点
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(pcd_publisher_node)
    # 如果需要启动宇树雷达驱动，取消注释
    # ld.add_action(unitree_lidar_node)
    
    return ld