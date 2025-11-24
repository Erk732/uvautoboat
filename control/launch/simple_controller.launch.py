from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='simple_controller',
            name='simple_controller',
            parameters=[{
                'forward_cmd': 800.0,
                'turn_angle': 0.8,
                'publish_period': 0.5,
                'stop_topic': '/wamv/stop',
            }],
        ),
    ])
