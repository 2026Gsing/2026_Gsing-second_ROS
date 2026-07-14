import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 是否启动 RViz（可通过 start_rviz:=false 关闭）
    start_rviz = LaunchConfiguration('start_rviz', default='true')

    # LiDAR 网络参数（由 launch_utils.py 自动检测传入，或手动指定）
    lidar_ip = LaunchConfiguration('lidar_ip', default='192.168.1.1')
    local_ip = LaunchConfiguration('local_ip', default='192.168.1.2')

    # Run unitree lidar
    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                {'xfer_format': 1},

                {'initialize_type': 2},
                {'work_mode': 0},
                {'use_system_timestamp': False},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},
                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': lidar_ip},
                {'local_port': 6201},
                {'local_ip': local_ip},

                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},
                ]
    )

    # Run Rviz (此部分无需修改)
    package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
    rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
    print("rviz_config_file = " + rviz_config_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
        condition=IfCondition(start_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.1', description='LiDAR sensor IP'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.2', description='Local machine IP'),
        node1, rviz_node,
    ])
