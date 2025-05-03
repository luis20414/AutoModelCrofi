from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_publicador',
            executable='ydlidar_publicador_node',
            name='ydlidar_publicador'
        ),
        #Node(
        #    package='ydlidar_client',
        #    executable='rebase',
        #    name='rebase_node'
        #),
        Node(
            package='driver',
            executable='driver',
            name='driver_node'
        ),
        Node(
            package='servo',
            executable='servo',
            name='servo_node'
        )
    ])
