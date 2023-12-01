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
        # 启动serial_robomaster_2节点
        Node(
            package='serial',
            executable='serial_robomaster_2',
            name='serial_robomaster_2',
            output='screen'
        ),
        # 启动serial_PullPushSensors_1节点
        Node(
            package='serial',
            executable='serial_PullPushSensors_1',
            name='serial_PullPushSensors_1',
            output='screen'
        ),
        # 启动serial_PullPushSensors_2节点
        Node(
            package='serial',
            executable='serial_PullPushSensors_2',
            name='serial_PullPushSensors_2',
            output='screen'
        ),
        # 启动buaa_rescue_robot_gui_node节点
        Node(
            package='buaa_rescue_robot_gui',
            executable='buaa_rescue_robot_gui_node',
            name='buaa_rescue_robot_gui_node',
            output='screen'
        ),
        # 启动auto_controller_1节点
        Node(
            package='controller',
            executable='auto_controller_1',
            name='auto_controller_1',
            output='screen'
        ),
        # 启动auto_controller_2节点
        Node(
            package='controller',
            executable='auto_controller_2',
            name='auto_controller_2',
            output='screen'
        ),
        # 启动omega7_sensor节点
        Node(
            package='omega7_sensor',
            executable='omega7_sensor',
            name='omega7_sensor',
            output='screen'
        ),
        # 启动omega7_sensor sub节点
        Node(
            package='omega7_sensor',
            executable='omega7_sub',
            name='omega7_sub',
            output='screen'
        )
    ])
