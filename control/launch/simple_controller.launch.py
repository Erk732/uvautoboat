from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    forward_cmd = LaunchConfiguration('forward_cmd')
    turn_angle = LaunchConfiguration('turn_angle')
    publish_period = LaunchConfiguration('publish_period')
    stop_topic = LaunchConfiguration('stop_topic')

    return LaunchDescription([
        DeclareLaunchArgument('forward_cmd', default_value='800.0'),
        DeclareLaunchArgument('turn_angle', default_value='0.0'),
        DeclareLaunchArgument('publish_period', default_value='0.5'),
        DeclareLaunchArgument('stop_topic', default_value='/wamv/stop'),
        Node(
            package='control',
            executable='simple_controller',
            name='simple_controller',
            parameters=[{
                'forward_cmd': forward_cmd,
                'turn_angle': turn_angle,
                'publish_period': publish_period,
                'stop_topic': stop_topic,
            }],
        ),
    ])
