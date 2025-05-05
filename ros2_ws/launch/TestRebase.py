from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(   # Publica los datos del lidar
            package='ydlidar_publicador', 
            executable='ydlidar_publicador_node',
            name='ydlidar_publicador'
        ),
        Node(   # Ordena los datos en dos arreglos
            package='ydlidar_client',
            executable='ydlidar_publisher',
            name='ydlidar_publicador_posterior'
        ),
        Node(
            package='ydlidar_client',
            executable='colision',
            name='rebase_node'
        ),
        Node(
            package='driver',
            executable='driver',
            name='driver_node'
        ),
        Node(
            package='directional_led',
            executable='directional_led',
            name='led_node'
        ),
        Node(
            package='servo',
            executable='servo',
            name='servo_node'
        )
    ])
