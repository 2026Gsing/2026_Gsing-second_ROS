#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    AppendEnvironmentVariable,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # ====================== 核心路径配置 =======================
    script_path = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.join(script_path, "../../../..")
    
    # 源码路径
    src_xacro_path = os.path.join(
        project_root,
        "description/src/car_test/urdf/robot.urdf.xacro"
    )
    
    # 安装路径（如果存在）
    try:
        desc_share = get_package_share_directory('description')
        install_xacro_path = os.path.join(desc_share, "urdf", "robot.urdf.xacro")
    except:
        install_xacro_path = ""
    
    # 优先使用源码路径（开发模式）
    if os.path.exists(src_xacro_path):
        xacro_file_path = src_xacro_path
        print(f"Debug: 使用源码 Xacro 文件: {xacro_file_path}")
    elif os.path.exists(install_xacro_path):
        xacro_file_path = install_xacro_path
        print(f"Debug: 使用安装 Xacro 文件: {xacro_file_path}")
    else:
        possible_paths = [
            os.path.join(project_root, "description/urdf/robot.urdf.xacro"),
            os.path.join(project_root, "src/description/urdf/robot.urdf.xacro"),
            os.path.join(project_root, "src/description/src/car_test/urdf/robot.urdf.xacro"),
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                xacro_file_path = path
                print(f"Debug: 找到备用 Xacro 文件: {xacro_file_path}")
                break
        else:
            raise FileNotFoundError(
                f"Xacro文件不存在！\n"
                f"源码路径：{src_xacro_path}\n"
                f"安装路径：{install_xacro_path}"
            )
    
    # 处理 xacro 文件
    robot_description_raw = process_xacro_with_paths(xacro_file_path, project_root)
    
    # Gazebo World文件路径
    world_source_path = os.path.join(project_root, "simulation/worlds/test.world")
    try:
        sim_share = get_package_share_directory('simulation')
        world_install_path = os.path.join(sim_share, "worlds", "test.world")
    except:
        world_install_path = ""

    if os.path.exists(world_source_path):
        world_file_path = world_source_path
        print(f"Debug: 使用源码 World 文件: {world_file_path}")
    elif os.path.exists(world_install_path):
        world_file_path = world_install_path
        print(f"Debug: 使用安装 World 文件: {world_file_path}")
    else:
        world_file_path = "empty.sdf"
        print(f"Warning: 自定义World文件不存在，使用内置空世界: {world_file_path}")
    
    # 设置资源路径
    src_model_path = os.path.join(project_root, "description/src")
    
    set_gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{src_model_path}:${{GZ_SIM_RESOURCE_PATH}}"
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ====================== 日志信息 =======================
    log_start = LogInfo(msg="🚀 启动Gsing Dog仿真...")
    log_gazebo_start = LogInfo(msg="📦 启动Gazebo...")
    log_spawn_delay = LogInfo(msg="⏳ 等待5秒后生成机器人...")
    log_spawn_start = LogInfo(msg="🤖 正在生成机器人...")

    # ====================== 节点配置 =======================
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file_path}',
            'use_sim_time': use_sim_time
        }.items()
    )

    # spawn 机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'car_test',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0',
            '-allow_renaming', 'false'
        ],
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 添加延迟 spawn
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[log_spawn_start, spawn_entity]
    )

    # ====================== 桥接配置 =======================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=['/camera'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true',
                             description='Use simulation time'),
        log_start,
        node_robot_state_publisher,
        log_gazebo_start,
        gz_sim,
        log_spawn_delay,
        delayed_spawn,
        bridge,
        image_bridge,
    ])

def process_xacro_with_paths(xacro_file_path, project_root):
    """处理 xacro 文件，设置正确的包含路径"""
    import xacro
    
    xacro_paths = [
        os.path.dirname(xacro_file_path),
        os.path.join(project_root, "description/src/car_test/urdf"),
        os.path.join(project_root, "description/urdf"),
        os.path.join(project_root, "description/src/car_test"),
    ]
    
    os.environ['XACRO_PATH'] = ':'.join(xacro_paths)
    doc = xacro.process_file(xacro_file_path, mappings={'xacro_path': ':'.join(xacro_paths)})
    return doc.toprettyxml(indent='  ')
