#!/usr/bin/env python3
"""
Launch gps_imu_pose (local ENU), pose_filter, and all_in_one_stack.
Pose output is in a local ENU frame with the first GPS fix as origin.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='gps_imu_pose',
            name='gps_imu_pose',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'gps_topic': '/wamv/sensors/gps/gps/fix',
                'imu_topic': '/wamv/sensors/imu/imu/data',
                'pose_topic': '/wamv/pose_raw',
                'frame_id': 'world',
                'base_link_frame': 'wamv/wamv/base_link',
                # Use local ENU from GPS/IMU (first fix as origin)
                'use_tf_pose': False,
                'tf_target_frame': 'world',
                'tf_source_frame': 'wamv/wamv/base_link',
            }],
        ),
        Node(
            package='control',
            executable='pose_filter',
            name='pose_filter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/wamv/pose_raw',
                'output_topic': '/wamv/pose_filtered',
            }],
        ),
        Node(
            package='control',
            executable='all_in_one_stack',
            name='all_in_one_stack',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'pose_topic': '/wamv/pose_filtered',
                # Obstacle and control parameters
                'front_angle_deg': 60.0,
                'side_angle_deg': 80.0,
                'obstacle_slow_dist': 24.0,
                'obstacle_stop_dist': 14.0,
                'avoid_clear_margin': 5.0,
                'avoid_max_turn_time': 8.0,
                'plan_avoid_margin': 12.0,
                'hull_radius': 1.5,
                'forward_thrust': 260.0,
                'kp_yaw': 350.0,
                'heading_align_thresh_deg': 25.0,
                'approach_slow_dist': 12.0,
                'stuck_timeout': 8.0,
                'stuck_progress_epsilon': 0.1,
                'recover_reverse_time': 3.0,
                'recover_turn_time': 3.0,
                'recover_reverse_thrust': -200.0,
                'lidar_log_interval': 2.0,
            }],
        ),
    ])
