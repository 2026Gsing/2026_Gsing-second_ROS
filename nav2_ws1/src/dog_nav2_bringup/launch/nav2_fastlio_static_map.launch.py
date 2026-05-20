#!/usr/bin/env python3

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

    # Fast-LIO TF: map -> camera_init -> body
    # Nav2 expects: map -> odom -> base_link
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

    # Optional: publish lidar frame under base_link (no base needed)
    static_tf_baselink_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_baselink_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar_lidar'],
        output='screen'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

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

    rviz_config = os.path.join(bringup_share, 'rviz', 'nav2_fastlio_static_map.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='nav2_rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Bridge: RViz 2D Goal Pose (topic goal_pose) -> Nav2 NavigateToPose action
    pkg_prefix = get_package_prefix('dog_nav2_bringup')
    goal_bridge_script = os.path.join(pkg_prefix, 'lib', 'dog_nav2_bringup', 'goal_pose_to_nav2.py')
    goal_pose_bridge = ExecuteProcess(
        cmd=[sys.executable, goal_bridge_script],
        name='goal_pose_to_nav2',
        output='screen',
    )

    costmap_grid_script = os.path.join(pkg_prefix, 'lib', 'dog_nav2_bringup', 'costmap_to_grid.py')
    costmap_grid_bridge = ExecuteProcess(
        cmd=[sys.executable, costmap_grid_script],
        name='costmap_to_grid',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    ld.add_action(declare_map)

    ld.add_action(static_tf_caminit_odom)
    ld.add_action(static_tf_body_baselink)
    ld.add_action(static_tf_baselink_lidar)

    ld.add_action(map_server)
    # Delay navigation stack startup to avoid TF cache / timestamp issues on sensor messages
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

