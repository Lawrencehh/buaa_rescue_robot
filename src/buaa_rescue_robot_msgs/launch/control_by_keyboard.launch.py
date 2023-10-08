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
        ),
        # 启动serial_robomaster_1节点
        Node(
            package='serial',
            executable='serial_robomaster_1',
            name='serial_robomaster_1',
            output='screen'
        ),
        # 启动serial_PullPushSensors_1节点
        Node(
            package='serial',
            executable='serial_PullPushSensors_1',
            name='serial_PullPushSensors_1',
            output='screen'
        )
    ])
