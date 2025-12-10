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
                
                # ==================== Thrust Control ====================
                'forward_thrust': 320.0,           # Base forward thrust (N) - lower to reduce post-avoid oscillation
                'kp_yaw': 600.0,                   # Heading P gain
                'control_rate': 30.0,              # Control loop frequency (Hz)
                
                # ==================== Waypoint Tracking ====================
                'waypoint_tolerance': 3.0,         # Intermediate waypoint tolerance (m)
                'goal_tolerance': 1.0,             # Final goal tolerance (m)
                'approach_slow_dist': 10.0,        # Start slowdown distance (m)
                'heading_align_thresh_deg': 25.0,  # Heading alignment threshold (deg) - higher to align before pushing
                'overshoot_margin': 1.0,           # Overshoot detection boundary (m)
                
                # ==================== Obstacle Avoidance ====================
                'obstacle_slow_dist': 10.0,        # Slowdown trigger distance (m)
                'obstacle_stop_dist': 7.0,         # Hard avoid trigger distance (m)
                'avoid_turn_thrust': 150.0,        # Avoidance turn thrust (N) - softer turning
                'avoid_diff_gain': 10.0,           # Avoidance steering gain - softer bias
                'avoid_clear_margin': 4.0,         # Safety margin to exit avoidance (m) - exit later
                'avoid_max_turn_time': 5.0,        # Maximum turn time (s)
                'full_clear_distance': 20.0,       # Force avoidance trigger distance (m) - FIXED: was 60.0 (caused avoidance mode to stick)
                'front_angle_deg': 30.0,           # Front sector angle (deg)
                'side_angle_deg': 60.0,            # Side sector angle (deg)
                'hazard_world_auto_origin': True,  # Auto-convert world hazard boxes using first pose
                # ==================== Auto Goal Sequence ====================
                # Leave disabled when using a separate goal sequencer on another machine
                'auto_goal_enable': False,         # Enable auto-publishing next goals
                # Format: "x1,y1;x2,y2" in same frame as pose (world/local ENU); ignored if auto_goal_enable=False
                'auto_next_goals': '130,0;130,50;80,60',
                
                # ==================== Stuck Detection & Recovery ====================
                'stuck_timeout': 8.0,              # Stuck detection timeout (s)
                'stuck_progress_epsilon': 0.1,     # Minimum movement distance (m)
                'recover_reverse_time': 3.0,       # Reverse recovery time (s)
                'recover_turn_time': 3.0,          # Turn recovery time (s)
                'recover_reverse_thrust': -200.0,  # Reverse thrust (N)
                'recover_reverse_time_long': 6.0,  # Extended reverse time (s)
                
                # ==================== Lidar Parameters ====================
                'min_range_filter': 3.0,           # Minimum range filter (m)
                'cloud_z_min': -10.0,              # Min height filter (m) - LOWER for deep underwater obstacles
                'cloud_z_max': 3.0,                # Max height filter (m) - HIGHER for tall obstacles
                'vfh_enabled': True,               # Enable VFH steering
                'vfh_bin_deg': 5.0,                # VFH bin angle (deg)
                'vfh_block_dist': 10.0,            # VFH blocking distance (m)
                'polar_use_scan': True,            # Use polar histogram
                'polar_min_range': 0.5,            # Polar histogram min range (m)
                
                # ==================== Planning Parameters ====================
                'plan_avoid_margin': 5.0,          # Planning detour margin (m)
                'hull_radius': 1.5,                # Hull radius (m)
                'path_step': 5.0,                  # Path interpolation step (m)
                
                # ==================== Data Timeouts ====================
                'pose_timeout': 1.0,               # Pose data timeout (s)
                'scan_timeout': 1.0,               # Lidar data timeout (s)
                'lidar_log_interval': 2.0,         # Lidar log interval (s)
                
            }, hazard_config],
        ),
    ])
