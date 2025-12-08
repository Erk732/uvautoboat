#!/usr/bin/env python3
"""
Modular Navigation Launch File (v2.1)

Launches the modular Vostok1-based navigation system with AllInOneStack enhancements:
- oko_perception (plan package) - OKO v2.1 - 3D LIDAR + LaserScan fusion, VFH steering
- sputnik_planner (plan package) - SPUTNIK v2.1 - GPS waypoint planning with hazard zones
- buran_controller (control package) - BURAN - PID control with obstacle avoidance

v2.1 Enhancements (ported from AllInOneStack):
- VFH steering for gap-finding navigation
- Polar histogram for smooth steering bias
- LaserScan + PointCloud2 fusion for robust detection
- Hazard zone pre-planning support
- Extended detection horizon (60m full_clear_distance)

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
        default_value='1000.0',
        description='Base thrust speed when moving forward'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='2000.0',
        description='Maximum thrust speed limit'
    )

    # OKO Perception Node - 3D LIDAR Processing (v2.1)
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
            'min_range': 3.0,        # Reduced: detect closer obstacles (was 5.0)
            'max_range': 60.0,       # Increased: longer detection range (was 50.0)
            'sample_rate': 1,        # Process ALL points for maximum detection
            # Enhanced OKO v2.0 parameters (tuned for faster response)
            'temporal_history_size': 3,      # Reduced: faster response
            'temporal_threshold': 2,         # Reduced: 2/3 detections to confirm
            'cluster_distance': 2.0,         # Max distance between cluster points (m)
            'min_cluster_size': 3,           # Reduced: detect smaller obstacles
            'water_plane_threshold': 0.5,    # Tolerance for water plane removal (m)
            'velocity_history_size': 5,      # Reduced: faster velocity estimate
            # === v2.1: VFH Steering (from AllInOneStack) ===
            'vfh_enabled': True,             # Enable VFH gap-finding
            'vfh_bin_deg': 5.0,              # VFH bin angle (degrees)
            'vfh_block_dist': 15.0,          # VFH blocking distance (m)
            'vfh_clearance_deg': 10.0,       # Inflation clearance (degrees)
            # === v2.1: Polar Histogram (from AllInOneStack) ===
            'polar_enabled': True,           # Enable polar histogram bias
            'polar_weight_power': 1.0,       # Weight power for distance
            # === v2.1: LaserScan Fusion ===
            'laserscan_enabled': True,       # Enable LaserScan fusion
            'laserscan_topic': '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            'laserscan_topic_alt': '/wamv/sensors/lidars/lidar_wamv/scan',
            # === v2.1: Extended Detection Horizon ===
            'full_clear_distance': 60.0,     # Force avoidance trigger distance (m)
        }]
    )

    # SPUTNIK Planner Node - GPS Waypoint Navigation (v2.1)
    sputnik_planner = Node(
        package='plan',
        executable='sputnik_planner',
        name='sputnik_planner_node',
        output='screen',
        parameters=[{
            'scan_length': LaunchConfiguration('scan_length'),
            'scan_width': LaunchConfiguration('scan_width'),
            'lanes': LaunchConfiguration('lanes'),
            'waypoint_tolerance': 1.0,                         # Reduced from 2.0m - stricter tolerance
            'waypoint_skip_timeout': 20.0,                     # Reduced from 45s - faster skip
            # === v2.1: Hazard Zone Planning (from AllInOneStack) ===
            'hazard_enabled': False,         # Set to True and provide hazard_world_boxes to enable
            'hazard_boxes': '',              # Local frame boxes: "xmin,ymin,xmax,ymax;..."
            'hazard_world_boxes': '',        # World frame boxes (see hazard_world_boxes.yaml)
            'hazard_origin_world_x': 0.0,    # Origin for world->local conversion
            'hazard_origin_world_y': 0.0,
            'plan_avoid_margin': 5.0,        # Planning detour margin (m)
            'hull_radius': 1.5,              # Boat hull radius for clearance (m)
        }]
    )

    # BURAN Navigation Controller - PID Control (v2.1)
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
            # === Obstacle Avoidance (v2.1: increased horizon) ===
            'obstacle_slow_factor': 0.3,
            'critical_distance': 10.0,      # Increased: hard stop distance (v2.1: increased from 5.0)
            'reverse_timeout': 5.0,
            # === Smart Anti-Stuck System (SASS) ===
            'stuck_timeout': 3.0,
            'stuck_threshold': 0.5,
            'no_go_zone_radius': 10.0,      # Increased: larger no-go zones (v2.1: was 8.0)
            'detour_distance': 15.0,        # Increased: longer detours (v2.1: was 12.0)
            # === Kalman Filter Tuning ===
            'kalman_process_noise': 0.01,
            'kalman_measurement_noise': 0.5,
            'drift_compensation_gain': 0.3,
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
