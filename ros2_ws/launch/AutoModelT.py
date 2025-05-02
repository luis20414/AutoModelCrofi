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
            package='lane_detector',
            executable='lane_detector',
            name='lane_detector_node'
        ),
        Node(
            package='lane_angle',
            executable='lane_angle',
            name='lane_angle_node'
        ),
        Node(
            package='servo',
            executable='servo',
            name='servo_node'
        )
    ])
