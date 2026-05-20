from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 定义地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': '/home/zhanghangming/fastlio2_v2/tools/pcd_to_gridmap/build/output.yaml'}]
    )

    # 2. 定义生命周期管理器节点
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'node_names': ['map_server'],  # 指定要管理的节点名
            'autostart': True,             # 系统启动时自动开始管理
        }]
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node
    ])