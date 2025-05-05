from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webcam',
            executable='webcam',
            name='webcam_node'
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
            package='red_vision',
            executable='red_vision',
            name='red_vision_node'
        ),
	    Node(
	        package='servo',
	        executable='servo',
	        name='servo_node'
	    ),
    	Node(
    	    package='driver',
    	    executable='driver',
    	    name='driver_node'
    	)
    ])
