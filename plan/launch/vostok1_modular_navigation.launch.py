#!/usr/bin/env python3
"""
Modular Navigation Launch File
Système de Navigation Modulaire - French Academic Edition

Launches the modular Vostok1-based navigation system:
- oko_perception (plan package) - ŒIL - 3D LIDAR processing
- sputnik_planner (plan package) - SPOUTNIK - GPS waypoint planning
- buran_controller (control package) - BOURANE - PID control with obstacle avoidance

Usage:
    ros2 launch plan vostok1_modular_navigation.launch.py
    
    # With custom PID gains:
    ros2 launch plan vostok1_modular_navigation.launch.py kp:=500.0 ki:=30.0 kd:=150.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # === SPUTNIK Planner Arguments ===
    scan_length_arg = DeclareLaunchArgument(
        'scan_length',
        default_value='15.0',
        description='Length of each scan lane in meters'
    )
    
    scan_width_arg = DeclareLaunchArgument(
        'scan_width',
        default_value='30.0',
        description='Width of each scan lane in meters'
    )
    
    lanes_arg = DeclareLaunchArgument(
        'lanes',
        default_value='10',
        description='Number of lanes in lawnmower pattern'
    )
    
    # === OKO Perception Arguments ===
    min_safe_distance_arg = DeclareLaunchArgument(
        'min_safe_distance',
        default_value='12.0',
        description='Minimum safe distance to obstacles (meters)'
    )
    
    # === BURAN Controller - PID Arguments ===
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='400.0',
        description='PID Proportional gain (Kp) - Controls response to heading error'
    )
    
    ki_arg = DeclareLaunchArgument(
        'ki',
        default_value='20.0',
        description='PID Integral gain (Ki) - Eliminates steady-state error'
    )
    
    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='100.0',
        description='PID Derivative gain (Kd) - Dampens oscillations'
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

    # ŒIL Perception Node - 3D LIDAR Processing
    # Parameters tuned for lake bank and harbour detection
    oko_perception = Node(
        package='plan',
        executable='oko_perception',
        name='oko_perception_node',
        output='screen',
        parameters=[{
            'min_safe_distance': LaunchConfiguration('min_safe_distance'),
            'critical_distance': 4.0,
            'hysteresis_distance': 1.5,
            'min_height': -15.0,     # Catch lake bank, harbour, water-level obstacles
            'max_height': 10.0,      # Catch tall structures
            'min_range': 5.0,        # Ignore spawn dock and boat structure
            'max_range': 50.0,
            'sample_rate': 1,        # Process ALL points for maximum detection
            # Enhanced OKO v2.0 parameters
            'temporal_history_size': 5,      # Scans to keep in history
            'temporal_threshold': 3,         # Min detections to confirm obstacle
            'cluster_distance': 2.0,         # Max distance between cluster points (m)
            'min_cluster_size': 5,           # Min points per cluster
            'water_plane_threshold': 0.5,    # Tolerance for water plane removal (m)
            'velocity_history_size': 10,     # Frames for velocity estimation
        }]
    )

    # SPOUTNIK Planner Node - GPS Waypoint Navigation
    sputnik_planner = Node(
        package='plan',
        executable='sputnik_planner',
        name='sputnik_planner_node',
        output='screen',
        parameters=[{
            'scan_length': LaunchConfiguration('scan_length'),
            'scan_width': LaunchConfiguration('scan_width'),
            'lanes': LaunchConfiguration('lanes'),
            'waypoint_tolerance': 2.0,
            'waypoint_skip_timeout': 45.0,  # Skip waypoint if obstacle blocks
        }]
    )

    # BOURANE Navigation Controller - PID Control
    buran_controller = Node(
        package='control',
        executable='buran_controller',
        name='buran_controller_node',
        output='screen',
        parameters=[{
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'base_speed': LaunchConfiguration('base_speed'),
            'max_speed': LaunchConfiguration('max_speed'),
            # Obstacle Avoidance
            'obstacle_slow_factor': 0.3,
            'critical_distance': 5.0,
            'reverse_timeout': 5.0,
            # Smart Anti-Stuck System (SASS)
            'stuck_timeout': 3.0,
            'stuck_threshold': 0.5,
            'no_go_zone_radius': 8.0,
            'detour_distance': 12.0,
        }]
    )

    return LaunchDescription([
        # === Launch Arguments ===
        # Planner
        scan_length_arg,
        scan_width_arg,
        lanes_arg,
        # Perception
        min_safe_distance_arg,
        # PID Controller
        kp_arg,
        ki_arg,
        kd_arg,
        base_speed_arg,
        max_speed_arg,
        # === Nodes - TNO/Soviet-era ===
        oko_perception,
        sputnik_planner,
        buran_controller,
    ])
