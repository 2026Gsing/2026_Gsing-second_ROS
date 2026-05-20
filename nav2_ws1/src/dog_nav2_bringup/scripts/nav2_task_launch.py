import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory("dog_nav2_bringup")
    # 地图文件路径
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

    # 定义启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # 构建启动描述
    ld = LaunchDescription()
    
    # 启动map_server
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path}, {'use_sim_time': use_sim_time}]
    ))
    
    # 启动amcl
    ld.add_action(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))
    
    # 启动controller_server
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))
    
    # 启动planner_server
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))
    
    # 启动bt_navigator
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))
    
    # 启动lifecycle_manager（关键：自动激活所有节点）
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}]
    ))
    
    return ld
