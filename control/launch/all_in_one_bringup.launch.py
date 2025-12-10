#!/usr/bin/env python3
"""
Launch gps_imu_pose (local ENU), pose_filter, and all_in_one_stack.
Pose output is in a local ENU frame with the first GPS fix as origin.
"""
# Quick test for beginners: launch this file, then publish a goal (frame=world) to start moving:
# ros2 topic pub /planning/goal geometry_msgs/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hazard_config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'hazard_world_boxes.yaml',
    )

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
                'front_angle_deg': 45.0,
                'side_angle_deg': 80.0,
                'obstacle_slow_dist': 12.0,
                'obstacle_stop_dist': 7.0,
                'avoid_clear_margin': 2.0,
                'avoid_max_turn_time': 5.0,
                'avoid_turn_thrust': 150.0,
                'avoid_diff_gain': 5.0,
                'diff_bias_alpha': 0.8,
                'side_change_threshold': 1.0,
                'side_straight_clearance': 6.0,
                'plan_avoid_margin': 8.0,
                'hull_radius': 1.5,
                'forward_thrust': 2200.0,
                'kp_yaw': 300.0,
                'heading_align_thresh_deg': 35.0,
                'approach_slow_dist': 8.0,
                'stuck_timeout': 25.0,
                'stuck_progress_epsilon': 0.5,
                'recover_reverse_time': 1.0,
                'recover_turn_time': 3.0,
                'recover_reverse_thrust': -120.0,
                'lidar_log_interval': 2.0,
            }, hazard_config],
        ),
    ])
