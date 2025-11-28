#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the path follower controller.
    """

    path_follower_node = Node(
        package='control',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'forward_thrust': 400.0},
            {'kp_yaw': 600.0},
            {'waypoint_tolerance': 3.0},
            {'goal_tolerance': 4.0},
            {'control_rate': 20.0},
        ],
    )

    return LaunchDescription([
        path_follower_node
    ])
