from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition 

def generate_launch_description():
    # 1. 是否使用自带的发布节点 (独立运行时用 True，配合 Gazebo 时用 False)
    use_publisher_arg = DeclareLaunchArgument(
        'use_publisher',
        default_value='true',
        description='Whether to start the image_publisher_node'
    )

    # 2. 输入话题名称 (方便重映射，默认 image_raw)
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='image_raw',
        description='The input topic for color detection'
    )

    # 3. 图片路径
    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value='',
        description='Path to image file. If empty, use camera.'
    )

    
    # 图像发布节点 (仅当 use_publisher 为 true 时启动)
    image_publisher_node = Node(
        package='color_test',
        executable='image_publisher_node',
        name='image_publisher_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_publisher')), 
        arguments=[LaunchConfiguration('image_path')],
    )

    # 颜色检测节点
    color_detection_node = Node(
        package='color_test',
        executable='color_detection_node',
        name='color_detection_node',
        output='screen',
        remappings=[
            ('image_raw', LaunchConfiguration('input_topic')) 
        ]
    )

    return LaunchDescription([
        use_publisher_arg,
        input_topic_arg,
        image_path_arg,
        image_publisher_node,
        color_detection_node
    ])