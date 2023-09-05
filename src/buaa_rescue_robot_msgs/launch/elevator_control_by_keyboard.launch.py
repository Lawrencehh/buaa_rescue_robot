import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # 启动keyboard_publisher节点
        Node(
            package='keyboard_controller',
            executable='keyboard_publisher',
            name='keyboard_publisher',
            output='screen'
        ),
        # 启动serial_sender节点
        Node(
            package='serial',
            executable='serial_sender',
            name='serial_sender',
            output='screen'
        )
    ])
