"""
nav2_task_launch.py — 竞赛任务 Nav2 启动（AMCL + 地图服务器版）

功能：
  启动 Nav2 全套节点：map_server、AMCL、controller_server、planner_server、
  bt_navigator、lifecycle_manager。适用于有激光雷达但未使用 FAST-LIO2
  全局定位的场景。

与 nav2_fastlio_static_map.launch.py 的区别：
  - 使用 AMCL 粒子滤波定位（非 FAST-LIO2 ICP 重定位）
  - 手动内联生成 Nav2 参数配置
  - 不依赖 FAST-LIO2 节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory("dog_nav2_bringup")
    map_yaml_path = os.path.join(package_dir, "maps", "task_field_map.yaml")

    # Nav2参数文件路径
    params_path = os.path.join(package_dir, "params", "nav2_task_params.yaml")

    # 确保参数文件存在
    if not os.path.exists(params_path):
        # 生成默认参数文件
        os.makedirs(os.path.dirname(params_path), exist_ok=True)
        with open(params_path, 'w') as f:
            f.write("""
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom"
    laser_model_type: "likelihood_field"
    max_particles: 2000
    min_particles: 500
    tf_broadcast: true

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_footprint
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_wait_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.2
      min_linear_vel: 0.1
      max_linear_accel: 0.1
      lookahead_dist: 0.3

local_costmap:
  ros__parameters:
    use_sim_time: False
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_footprint
    robot_radius: 0.2
    inflation_radius: 0.3
    rolling_window: true
    width: 3.0
    height: 3.0
    resolution: 0.05
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

global_costmap:
  ros__parameters:
    use_sim_time: False
    update_frequency: 1.0
    publish_frequency: 1.0
    global_frame: map
    robot_base_frame: base_footprint
    robot_radius: 0.2
    inflation_radius: 0.3
    static_map: true
    resolution: 0.05
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

planner_server:
  ros__parameters:
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      use_astar: true
      allow_unknown: true

lifecycle_manager:
  ros__parameters:
    use_sim_time: False
    autostart: true
    node_names:
      - controller_server
      - planner_server
      - bt_navigator
      - map_server
      - amcl
""")

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    ld = LaunchDescription()

    # map_server: 加载预存 2D 栅格地图
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path}, {'use_sim_time': use_sim_time}]
    ))

    # AMCL: 自适应蒙特卡洛定位（粒子滤波），基于激光雷达与里程计进行定位
    ld.add_action(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))

    # controller_server: 局部路径跟踪（Regulated Pure Pursuit 控制器）
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))

    # planner_server: 全局路径规划（Navfn / A*）
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))

    # bt_navigator: 行为树导航器（规划→控制→恢复行为编排）
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))

    # lifecycle_manager: 统一管理所有 Nav2 节点的生命周期
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))

    return ld
