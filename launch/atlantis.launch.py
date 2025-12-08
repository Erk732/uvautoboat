#!/usr/bin/env python3
"""
Atlantis Launch File - Complete Autonomous Navigation System

Launches the Atlantis modular navigation stack:
- atlantis_planner (plan package) - Lawnmower path generation
- atlantis_controller (control package) - PID control with obstacle avoidance & SASS

Usage:
    # Basic launch with defaults:
    ros2 launch plan atlantis.launch.py

    # With custom PID gains:
    ros2 launch plan atlantis.launch.py kp:=500.0 ki:=30.0 kd:=150.0

    # With custom lawnmower pattern:
    ros2 launch plan atlantis.launch.py scan_length:=100.0 scan_width:=25.0 lanes:=6

    # Full custom configuration:
    ros2 launch plan atlantis.launch.py base_speed:=600.0 min_safe_distance:=20.0 stuck_timeout:=8.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========================================
    # PLANNER ARGUMENTS
    # ========================================
    scan_length_arg = DeclareLaunchArgument(
        'scan_length',
        default_value='150.0',
        description='Length of each scan lane in meters'
    )
    
    scan_width_arg = DeclareLaunchArgument(
        'scan_width',
        default_value='20.0',
        description='Width of each scan lane in meters'
    )
    
    lanes_arg = DeclareLaunchArgument(
        'lanes',
        default_value='4',
        description='Number of lanes in lawnmower pattern'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='TF frame ID for path visualization'
    )

    # ========================================
    # CONTROLLER - PID ARGUMENTS
    # ========================================
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='400.0',
        description='PID Proportional gain - Response to heading error'
    )
    
    ki_arg = DeclareLaunchArgument(
        'ki',
        default_value='20.0',
        description='PID Integral gain - Eliminates steady-state error'
    )
    
    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='100.0',
        description='PID Derivative gain - Dampens oscillations'
    )
    
    base_speed_arg = DeclareLaunchArgument(
        'base_speed',
        default_value='500.0',
        description='Base thrust speed when moving forward'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='800.0',
        description='Maximum thrust speed limit'
    )
    
    waypoint_tolerance_arg = DeclareLaunchArgument(
        'waypoint_tolerance',
        default_value='2.0',
        description='Distance threshold to consider waypoint reached (meters)'
    )

    # ========================================
    # CONTROLLER - OBSTACLE AVOIDANCE ARGUMENTS
    # ========================================
    min_safe_distance_arg = DeclareLaunchArgument(
        'min_safe_distance',
        default_value='15.0',
        description='Minimum safe distance to obstacles (meters)'
    )
    
    critical_distance_arg = DeclareLaunchArgument(
        'critical_distance',
        default_value='5.0',
        description='Critical distance triggering emergency stop (meters)'
    )
    
    obstacle_slow_factor_arg = DeclareLaunchArgument(
        'obstacle_slow_factor',
        default_value='0.3',
        description='Speed reduction factor when obstacles detected (0.0-1.0)'
    )
    
    hysteresis_distance_arg = DeclareLaunchArgument(
        'hysteresis_distance',
        default_value='2.0',
        description='Hysteresis buffer to prevent oscillation (meters)'
    )
    
    reverse_timeout_arg = DeclareLaunchArgument(
        'reverse_timeout',
        default_value='5.0',
        description='Maximum reverse duration before changing direction (seconds)'
    )

    # ========================================
    # CONTROLLER - STUCK RECOVERY (SASS) ARGUMENTS
    # ========================================
    stuck_timeout_arg = DeclareLaunchArgument(
        'stuck_timeout',
        default_value='5.0',
        description='Time without progress before triggering stuck recovery (seconds)'
    )
    
    stuck_threshold_arg = DeclareLaunchArgument(
        'stuck_threshold',
        default_value='1.0',
        description='Minimum distance required to not be considered stuck (meters)'
    )
    
    no_go_zone_radius_arg = DeclareLaunchArgument(
        'no_go_zone_radius',
        default_value='8.0',
        description='Radius around stuck positions to avoid (meters)'
    )
    
    drift_compensation_gain_arg = DeclareLaunchArgument(
        'drift_compensation_gain',
        default_value='0.3',
        description='Gain for lateral drift correction during escape'
    )
    
    probe_angle_arg = DeclareLaunchArgument(
        'probe_angle',
        default_value='45.0',
        description='Angle for directional probing during escape (degrees)'
    )
    
    detour_distance_arg = DeclareLaunchArgument(
        'detour_distance',
        default_value='12.0',
        description='Distance to travel during escape detour (meters)'
    )

    # ========================================
    # NODES
    # ========================================
    
    # Atlantis Planner Node (from plan package)
    atlantis_planner_node = Node(
        package='plan',
        executable='atlantis_planner',
        name='atlantis_planner',
        output='screen',
        parameters=[{
            'scan_length': LaunchConfiguration('scan_length'),
            'scan_width': LaunchConfiguration('scan_width'),
            'lanes': LaunchConfiguration('lanes'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )
    # Standalone OKO perception (allow Atlantis to rely on shared perception)
    oko_perception_node = Node(
        package='plan',
        executable='oko_perception',
        name='oko_perception_node',
        output='screen',
        parameters=[{
            'min_safe_distance': 12.0,
            'critical_distance': 4.0,
            'min_height': -15.0,
            'max_height': 10.0,
            'min_range': 3.0,
            'max_range': 60.0,
        }]
    )
    
    # Atlantis Controller Node (from control package)
    atlantis_controller_node = Node(
        package='control',
        executable='atlantis_controller',
        name='atlantis_controller',
        output='screen',
        parameters=[{
            # PID Control
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'base_speed': LaunchConfiguration('base_speed'),
            'max_speed': LaunchConfiguration('max_speed'),
            'waypoint_tolerance': LaunchConfiguration('waypoint_tolerance'),
            # Obstacle Avoidance
            'min_safe_distance': LaunchConfiguration('min_safe_distance'),
            'critical_distance': LaunchConfiguration('critical_distance'),
            'obstacle_slow_factor': LaunchConfiguration('obstacle_slow_factor'),
            'hysteresis_distance': LaunchConfiguration('hysteresis_distance'),
            'reverse_timeout': LaunchConfiguration('reverse_timeout'),
            # Stuck Recovery (SASS)
            'stuck_timeout': LaunchConfiguration('stuck_timeout'),
            'stuck_threshold': LaunchConfiguration('stuck_threshold'),
            'no_go_zone_radius': LaunchConfiguration('no_go_zone_radius'),
            'drift_compensation_gain': LaunchConfiguration('drift_compensation_gain'),
            'probe_angle': LaunchConfiguration('probe_angle'),
            'detour_distance': LaunchConfiguration('detour_distance'),
        }],
    )

    return LaunchDescription([
        # Planner arguments
        scan_length_arg,
        scan_width_arg,
        lanes_arg,
        frame_id_arg,
        # Controller - PID arguments
        kp_arg,
        ki_arg,
        kd_arg,
        base_speed_arg,
        max_speed_arg,
        waypoint_tolerance_arg,
        # Controller - Obstacle avoidance arguments
        min_safe_distance_arg,
        critical_distance_arg,
        obstacle_slow_factor_arg,
        hysteresis_distance_arg,
        reverse_timeout_arg,
        # Controller - Stuck recovery arguments
        stuck_timeout_arg,
        stuck_threshold_arg,
        no_go_zone_radius_arg,
        drift_compensation_gain_arg,
        probe_angle_arg,
        detour_distance_arg,
        # Nodes
        atlantis_planner_node,
        oko_perception_node,
        atlantis_controller_node,
    ])
