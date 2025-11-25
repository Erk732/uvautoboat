import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # <--- UPDATED: Matches your folder name
    pkg_name = 'plan'
    
    # Get path to the Rviz config we are about to save
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

        # 2. Planner Node (Brain)
        Node(
            package=pkg_name,
            executable='astar_planner',
            name='planner_node',
            output='screen'
        ),

        # 3. Rviz2 (Visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
