from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(   # Publica los datos del lidar
            package='ydlidar_publicador', 
            executable='ydlidar_publicador_node',
            name='ydlidar_publicador',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        ),
        Node(   # Ordena los datos en dos arreglos
            package='ydlidar_client',
            executable='ydlidar_publisher',
            name='ydlidar_publicador_posterior',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        ),
        Node(
            package='ydlidar_client',
            executable='colision',
            name='rebase_node',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        ),
        Node(
            package='driver',
            executable='driver',
            name='driver_node',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        ),
        Node(
            package='directional_led',
            executable='directional_led',
            name='led_node',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        ),
        Node(
            package='servo',
            executable='servo',
            name='servo_node',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        )
    ])
