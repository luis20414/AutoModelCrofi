from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webcam',
            executable='webcam',
            name='webcam_node'
        ),
        Node(   # Publica los datos del lidar
            package='control', 
            executable='master_node',
            name='maquinadeestados',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
                ]
        ),
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
            name='colision_node',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        ),
        Node(
            package='ydlidar_client',
            executable='rebase',
            name='rebase_node',
            output='screen',
            arguments=[
                "--ros-args",
                "--disable-external-lib-logs"
            ]
        ),
        Node(
            package='directional_led',
            executable='directional_led',
            name='directional_led_node'
        ),
        Node(
            package='steering_error',
            executable='steering_error',
            name='steering_error_node'
        ),
        Node(
            package='lanes_borders',
            executable='lanes_borders',
            name='lanes_borders_node'
        ),
        
	    Node(
	        package='servo',
	        executable='servo',
	        name='servo_node'
	    ),
	    Node(
	        package='ydlidar_client',
	        executable='rebase',
	        name='rebase_node'
	    ),
    	Node(
    	    package='driver',
    	    executable='driver',
    	    name='driver_node'
    	)
    ])
