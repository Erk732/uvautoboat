import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'plan'
    
    # Get path to the Rviz config
    pkg_share = get_package_share_directory(pkg_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    return LaunchDescription([
        # 1. Perception Node (3D Eyes)
        Node(
            package=pkg_name,
            executable='simple_perception',
            name='perception_node',
            output='screen'
        ),

        # 2. Planner Node (Brain with LIDAR Obstacle Avoidance)
        Node(
            package=pkg_name,
            executable='atlantis_planner',
            name='atlantis_planner',
            output='screen'
        ),

        # 3. Controller Node (Real-time obstacle avoidance & path following)
        Node(
            package='control',
            executable='atlantis_controller',
            name='atlantis_controller',
            output='screen'
        ),

        # 4. Mission Trigger (The Auto-Start)
        Node(
            package=pkg_name,
            executable='mission_trigger',
            name='mission_trigger_node',
            output='screen'
        ),

        # 5. Rviz2 (Visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # 6. TF Broadcaster
        Node(
            package='plan',
            executable='tf_broadcaster',
            name='tf_broadcaster_node',
            output='screen'
        ),
    ])