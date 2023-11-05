import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # 启动serial_master_devices节点
        Node(
            package='serial',
            executable='serial_master_devices',
            name='serial_master_devices',
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
        ),
        # 启动buaa_rescue_robot_gui_node节点
        Node(
            package='buaa_rescue_robot_gui',
            executable='buaa_rescue_robot_gui_node',
            name='buaa_rescue_robot_gui_node',
            output='screen'
        )
    ])
