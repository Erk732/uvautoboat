#!/usr/bin/env python3
"""
OKO Perception - 3D LiDAR Point Cloud Processing (Enhanced v2.0)

Part of the modular Vostok1 architecture.
Subscribes to 3D LiDAR data, processes point cloud, publishes obstacle information.

Features:
- Three-sector obstacle detection (front, left, right)
- 10th percentile filtering for noise rejection
- Height-based filtering to reject sky/water reflections
- Hysteresis to prevent oscillation at detection boundary

Enhanced Features (v2.0):
- Temporal filtering (multi-scan history for noise rejection)
- Distance-weighted urgency scoring
- Obstacle clustering (gap detection)
- Adaptive sector analysis (target-aware)
- Ground plane removal (water surface filtering)
- Velocity estimation (moving obstacle tracking)

Topics:
    Subscribes:
        /wamv/sensors/lidars/lidar_wamv_sensor/points (PointCloud2)
        /planning/current_target (String) - For adaptive sectors
    
    Publishes:
        /perception/obstacle_info (String) - JSON with obstacle distances per sector
        /perception/obstacle_detected (Bool) - Simple obstacle detection flag
"""

import rclpy
from rclpy.node import Node
import math
import struct
import json
import numpy as np
from collections import deque

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String, Bool


class OkoPerception(Node):
    """
    OKO - Obstacle detection and perception system (Enhanced version v2.1)

    Enhanced with temporal filtering, clustering, and velocity estimation.
    """
    def __init__(self):
        super().__init__('oko_perception_node')

        # --- PARAMETERS ---
        # Tuned for lake bank detection (LiDAR mounted high on WAM-V frame)
        # Lake banks appear BELOW the LiDAR (negative Z values)
        self.declare_parameter('min_safe_distance', 12.0)  # Detection threshold
        self.declare_parameter('critical_distance', 4.0)   # Emergency stop threshold
        self.declare_parameter('hysteresis_distance', 1.5) # Prevent oscillation
        self.declare_parameter('min_height', -15.0)  # Lake bank is ~2-3m below LiDAR
        self.declare_parameter('max_height', 10.0)   # Include terrain above water
        self.declare_parameter('min_range', 5.0)     # Ignore spawn dock and boat structure
        self.declare_parameter('max_range', 50.0)    # Focus on nearby obstacles
        self.declare_parameter('sample_rate', 1)     # Process ALL points
        
        # Enhanced parameters (v2.0)
        self.declare_parameter('temporal_history_size', 3)    # Reduced: faster response
        self.declare_parameter('temporal_threshold', 2)       # Reduced: 2/3 detections to confirm
        self.declare_parameter('cluster_distance', 2.0)       # Max distance between cluster points
        self.declare_parameter('min_cluster_size', 3)         # Reduced: detect smaller obstacles
        self.declare_parameter('water_plane_threshold', 0.5)  # Tolerance for water plane removal
        self.declare_parameter('velocity_history_size', 5)    # Reduced: faster velocity estimate

        # v2.1: VFH Steering parameters (from AllInOneStack)
        self.declare_parameter('vfh_enabled', True)           # Enable VFH steering
        self.declare_parameter('vfh_bin_deg', 5.0)            # VFH bin angle (degrees)
        self.declare_parameter('vfh_block_dist', 15.0)        # VFH blocking distance (m)
        self.declare_parameter('vfh_clearance_deg', 10.0)     # Inflation clearance (degrees)

        # v2.1: Polar histogram parameters (from AllInOneStack)
        self.declare_parameter('polar_enabled', True)         # Enable polar histogram bias
        self.declare_parameter('polar_weight_power', 1.0)     # Weight power for distance

        # v2.1: LaserScan fusion parameters
        self.declare_parameter('laserscan_enabled', True)     # Enable LaserScan fusion
        self.declare_parameter('laserscan_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/scan')
        self.declare_parameter('laserscan_topic_alt', '/wamv/sensors/lidars/lidar_wamv/scan')

        # v2.1: Extended detection horizon (from AllInOneStack)
        self.declare_parameter('full_clear_distance', 60.0)   # Force avoidance trigger distance

        # Get parameters
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.hysteresis_distance = self.get_parameter('hysteresis_distance').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # Enhanced parameters
        self.temporal_history_size = self.get_parameter('temporal_history_size').value
        self.temporal_threshold = self.get_parameter('temporal_threshold').value
        self.cluster_distance = self.get_parameter('cluster_distance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.water_plane_threshold = self.get_parameter('water_plane_threshold').value
        self.velocity_history_size = self.get_parameter('velocity_history_size').value

        # v2.1: VFH parameters
        self.vfh_enabled = self.get_parameter('vfh_enabled').value
        self.vfh_bin_deg = self.get_parameter('vfh_bin_deg').value
        self.vfh_block_dist = self.get_parameter('vfh_block_dist').value
        self.vfh_clearance_deg = self.get_parameter('vfh_clearance_deg').value

        # v2.1: Polar histogram parameters
        self.polar_enabled = self.get_parameter('polar_enabled').value
        self.polar_weight_power = self.get_parameter('polar_weight_power').value

        # v2.1: LaserScan fusion parameters
        self.laserscan_enabled = self.get_parameter('laserscan_enabled').value
        laserscan_topic = self.get_parameter('laserscan_topic').value
        laserscan_topic_alt = self.get_parameter('laserscan_topic_alt').value

        # v2.1: Extended detection horizon
        self.full_clear_distance = self.get_parameter('full_clear_distance').value

        # --- STATE ---
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        
        # Enhanced state (v2.0)
        # 1. Temporal filtering
        self.detection_history = deque(maxlen=self.temporal_history_size)
        self.sector_history = {
            'front': deque(maxlen=self.temporal_history_size),
            'left': deque(maxlen=self.temporal_history_size),
            'right': deque(maxlen=self.temporal_history_size)
        }
        
        # 2. Urgency scoring
        self.front_urgency = 0.0
        self.left_urgency = 0.0
        self.right_urgency = 0.0
        self.overall_urgency = 0.0
        
        # 3. Obstacle clusters
        self.clusters = []  # List of (centroid_x, centroid_y, size, min_dist)
        self.gaps = []      # List of (angle_start, angle_end, width)
        
        # 4. Adaptive sectors
        self.target_angle = 0.0  # Direction to target waypoint
        self.front_half_width = math.pi / 4  # Default ±45°
        
        # 5. Ground plane
        self.water_plane_z = None  # Estimated water surface height
        
        # 6. Velocity estimation
        self.obstacle_tracks = {}  # {id: deque of (x, y, timestamp)}
        self.obstacle_velocities = {}  # {id: (vx, vy)}

        # 7. v2.1: VFH steering state
        self.vfh_best_direction = None  # Best gap direction (radians)
        self.vfh_blocked_bins = []      # List of blocked bins

        # 8. v2.1: Polar histogram state
        self.polar_bias = 0.0           # Steering bias [-1, 1]

        # 9. v2.1: LaserScan state
        self.latest_scan = None         # Latest LaserScan message
        self.scan_front_clear = float('inf')
        self.scan_left_clear = float('inf')
        self.scan_right_clear = float('inf')

        # 10. v2.1: Force avoidance state
        self.force_avoid_active = False

        # --- PARAMETER CALLBACK ---
        # Allow runtime parameter updates from web dashboard
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscribe to config updates from web dashboard
        self.create_subscription(
            String,
            '/sputnik/set_config',
            self.config_callback,
            10
        )

        # --- SUBSCRIBER ---
        self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        # Subscribe to target for adaptive sectors
        self.create_subscription(
            String,
            '/planning/current_target',
            self.target_callback,
            10
        )

        # v2.1: LaserScan fusion (dual topic support like AllInOneStack)
        if self.laserscan_enabled:
            self.create_subscription(
                LaserScan,
                laserscan_topic,
                self.laserscan_callback,
                rclpy.qos.qos_profile_sensor_data
            )
            if laserscan_topic_alt and laserscan_topic_alt != laserscan_topic:
                self.create_subscription(
                    LaserScan,
                    laserscan_topic_alt,
                    self.laserscan_callback,
                    rclpy.qos.qos_profile_sensor_data
                )
            self.get_logger().info(f"LaserScan fusion enabled: {laserscan_topic}")

        # --- PUBLISHERS ---
        self.pub_obstacle_info = self.create_publisher(
            String, '/perception/obstacle_info', 10
        )
        self.pub_obstacle_detected = self.create_publisher(
            Bool, '/perception/obstacle_detected', 10
        )

        # Publish at 20Hz tune it 
        self.create_timer(0.05, self.publish_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("OKO v2.1 - Enhanced Obstacle Detection System")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Safe Distance: {self.min_safe_distance}m | Critical: {self.critical_distance}m")
        self.get_logger().info(f"Height Filter: {self.min_height}m to {self.max_height}m")
        self.get_logger().info(f"Temporal Filter: {self.temporal_threshold}/{self.temporal_history_size} scans")
        self.get_logger().info(f"Clustering: {self.cluster_distance}m eps, {self.min_cluster_size} min points")
        self.get_logger().info("--- v2.1 Enhancements ---")
        self.get_logger().info(f"VFH Steering: {'Enabled' if self.vfh_enabled else 'Disabled'} (bin={self.vfh_bin_deg}°, block={self.vfh_block_dist}m)")
        self.get_logger().info(f"Polar Histogram: {'Enabled' if self.polar_enabled else 'Disabled'}")
        self.get_logger().info(f"LaserScan Fusion: {'Enabled' if self.laserscan_enabled else 'Disabled'}")
        self.get_logger().info(f"Full Clear Distance: {self.full_clear_distance}m")
        self.get_logger().info("=" * 60)
    
    def target_callback(self, msg):
        """Update target direction for adaptive sectors"""
        try:
            data = json.loads(msg.data)
            target_heading = data.get('target_heading', 0.0)
            self.target_angle = math.radians(target_heading)

            # Adaptive front sector width based on target direction
            # Narrower when heading straight, wider when turning
            heading_diff = abs(self.target_angle)
            self.front_half_width = math.pi/6 + (math.pi/12) * min(1.0, heading_diff / (math.pi/2))
        except:
            pass

    def laserscan_callback(self, msg):
        """Process LaserScan for fusion with PointCloud2 (v2.1)"""
        self.latest_scan = msg

        # Analyze scan for sector clearance
        front_angle = math.pi / 6  # ±30° front
        side_angle = math.pi / 3   # ±60° sides

        f_min = float('inf')
        l_min = float('inf')
        r_min = float('inf')

        angle = msg.angle_min
        for rng in msg.ranges:
            if math.isinf(rng) or math.isnan(rng) or rng <= 0.0 or rng < self.min_range:
                angle += msg.angle_increment
                continue

            # Front sector
            if -front_angle <= angle <= front_angle:
                f_min = min(f_min, rng)
            # Left sector
            if 0.0 <= angle <= side_angle:
                l_min = min(l_min, rng)
            # Right sector
            if -side_angle <= angle <= 0.0:
                r_min = min(r_min, rng)

            angle += msg.angle_increment

        # Store scan results for fusion
        fallback = msg.range_max if not math.isinf(msg.range_max) else self.max_range
        self.scan_front_clear = f_min if f_min != float('inf') else fallback
        self.scan_left_clear = l_min if l_min != float('inf') else fallback
        self.scan_right_clear = r_min if r_min != float('inf') else fallback

        # v2.1: Calculate VFH steering from scan
        if self.vfh_enabled:
            self._calculate_vfh_steering()

        # v2.1: Calculate polar histogram bias from scan
        if self.polar_enabled:
            self._calculate_polar_bias()

        # v2.1: Update force avoidance state
        self._update_force_avoid_state()

    def parameter_callback(self, params):
        """Handle ROS 2 parameter changes for real-time tuning"""
        from rcl_interfaces.srv import SetParametersResult
        
        for param in params:
            try:
                if param.name == 'min_safe_distance':
                    self.min_safe_distance = param.value
                    self.get_logger().info(f"✓ Updated min_safe_distance: {self.min_safe_distance}m")
                elif param.name == 'critical_distance':
                    self.critical_distance = param.value
                    self.get_logger().info(f"✓ Updated critical_distance: {self.critical_distance}m")
                elif param.name == 'hysteresis_distance':
                    self.hysteresis_distance = param.value
                    self.get_logger().info(f"✓ Updated hysteresis_distance: {self.hysteresis_distance}m")
                elif param.name == 'vfh_block_dist':
                    self.vfh_block_dist = param.value
                    self.get_logger().info(f"✓ Updated vfh_block_dist: {self.vfh_block_dist}m")
                elif param.name == 'full_clear_distance':
                    self.full_clear_distance = param.value
                    self.get_logger().info(f"✓ Updated full_clear_distance: {self.full_clear_distance}m")
            except Exception as e:
                self.get_logger().error(f"Error updating parameter {param.name}: {e}")
                return SetParametersResult(successful=False)
        
        return SetParametersResult(successful=True)

    def config_callback(self, msg):
        """Handle configuration updates from web dashboard"""
        try:
            config = json.loads(msg.data)
            
            if 'min_safe_distance' in config:
                self.min_safe_distance = float(config['min_safe_distance'])
                self.get_logger().info(f"✓ Config: min_safe_distance = {self.min_safe_distance}m")
            
            if 'critical_distance' in config:
                self.critical_distance = float(config['critical_distance'])
                self.get_logger().info(f"✓ Config: critical_distance = {self.critical_distance}m")
            
            if 'vfh_block_dist' in config:
                self.vfh_block_dist = float(config['vfh_block_dist'])
                self.get_logger().info(f"✓ Config: vfh_block_dist = {self.vfh_block_dist}m")
                
        except Exception as e:
            self.get_logger().error(f"Config parse error: {e}")

    def lidar_callback(self, msg):
        """Process 3D LIDAR point cloud for obstacle detection (Enhanced v2.0)"""
        points = []
        all_z_values = []  # For water plane estimation
        total_points = 0
        valid_points = 0
        height_filtered = 0
        range_filtered = 0
        behind_filtered = 0
        water_filtered = 0
        
        point_step = msg.point_step
        data = msg.data
        current_time = self.get_clock().now()
        
        # Process points - sample every Nth point
        for i in range(0, len(data) - point_step, point_step * self.sample_rate):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                total_points += 1
                
                # Skip invalid points
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    continue
                if math.isinf(x) or math.isinf(y) or math.isinf(z):
                    continue
                
                valid_points += 1
                
                # Collect Z values for water plane estimation
                dist_2d = math.sqrt(x*x + y*y)
                if self.min_range < dist_2d < self.max_range:
                    all_z_values.append(z)
                
                # Filter by height
                if z < self.min_height or z > self.max_height:
                    height_filtered += 1
                    continue
                
                # Calculate horizontal distance
                dist = math.sqrt(x*x + y*y)
                
                # Filter by range
                if dist < self.min_range or dist > self.max_range:
                    range_filtered += 1
                    continue
                
                # Only consider points in front (positive x)
                if x < 0.2:
                    behind_filtered += 1
                    continue
                
                # 5. Ground plane removal - filter water surface
                if self.water_plane_z is not None:
                    if abs(z - self.water_plane_z) < self.water_plane_threshold:
                        water_filtered += 1
                        continue
                
                points.append((x, y, z, dist))
                    
            except (struct.error, Exception):
                continue
        
        # 5. Update water plane estimate (lowest 5th percentile of Z values)
        if len(all_z_values) > 100:
            sorted_z = sorted(all_z_values)
            self.water_plane_z = sorted_z[len(sorted_z) // 20]  # 5th percentile
        
        # Debug logging (throttled)
        self.get_logger().info(
            f"LiDAR: {total_points} pts, valid={valid_points}, "
            f"h_filt={height_filtered}, r_filt={range_filtered}, water={water_filtered}, "
            f"final={len(points)}",
            throttle_duration_sec=2.0
        )
        
        if not points:
            self._handle_no_points()
            return
        
        # Convert to numpy for efficient processing
        points_array = np.array(points)
        
        # Get minimum distance
        distances = points_array[:, 3]
        raw_min_distance = np.min(distances)
        
        # 2. Obstacle clustering
        self.clusters = self._cluster_obstacles(points_array)
        self.gaps = self._find_gaps(points_array)
        
        # 6. Velocity estimation - track clusters over time
        self._update_obstacle_tracks(self.clusters, current_time)
        
        # 1. Temporal filtering for obstacle detection
        raw_detected = raw_min_distance < self.min_safe_distance
        self.detection_history.append(raw_detected)
        
        # Confirm detection only if threshold met
        confirmed_detections = sum(self.detection_history)
        self.obstacle_detected = confirmed_detections >= self.temporal_threshold
        
        # Use filtered min distance (median of recent scans for stability)
        self.min_obstacle_distance = raw_min_distance
        
        # Hysteresis for obstacle detection
        if self.obstacle_detected:
            exit_threshold = self.min_safe_distance + self.hysteresis_distance
            self.obstacle_detected = self.min_obstacle_distance < exit_threshold

        # 4. Adaptive sector analysis
        self._analyze_sectors_adaptive(points_array)
        
        # 2. Calculate urgency scores
        self._calculate_urgency()

    def _handle_no_points(self):
        """Handle case when no valid points detected"""
        self.min_obstacle_distance = self.max_range
        self.detection_history.append(False)
        if not self.obstacle_detected:
            self.front_clear = self.max_range
            self.left_clear = self.max_range
            self.right_clear = self.max_range
        self.front_urgency = 0.0
        self.left_urgency = 0.0
        self.right_urgency = 0.0
        self.overall_urgency = 0.0
        self.clusters = []
        self.gaps = []

    def _cluster_obstacles(self, points):
        """
        Simple distance-based clustering (no sklearn dependency)
        Groups nearby points into obstacle clusters
        """
        if len(points) < self.min_cluster_size:
            return []
        
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if used[i]:
                continue
            
            # Start new cluster
            cluster_indices = [i]
            used[i] = True
            
            # Find all points within cluster_distance
            for j in range(i + 1, len(points)):
                if used[j]:
                    continue
                
                # Check distance to any point in cluster
                for ci in cluster_indices:
                    dx = points[j, 0] - points[ci, 0]
                    dy = points[j, 1] - points[ci, 1]
                    dist = math.sqrt(dx*dx + dy*dy)
                    
                    if dist < self.cluster_distance:
                        cluster_indices.append(j)
                        used[j] = True
                        break
            
            # Only keep significant clusters
            if len(cluster_indices) >= self.min_cluster_size:
                cluster_points = points[cluster_indices]
                centroid_x = np.mean(cluster_points[:, 0])
                centroid_y = np.mean(cluster_points[:, 1])
                min_dist = np.min(cluster_points[:, 3])
                size = len(cluster_indices)
                
                clusters.append({
                    'centroid': (centroid_x, centroid_y),
                    'size': size,
                    'min_distance': min_dist,
                    'angle': math.atan2(centroid_y, centroid_x)
                })
        
        return clusters

    def _find_gaps(self, points):
        """Find gaps between obstacles that boat could pass through"""
        if len(points) < 10:
            return []
        
        # Sort points by angle
        angles = np.arctan2(points[:, 1], points[:, 0])
        sorted_indices = np.argsort(angles)
        sorted_angles = angles[sorted_indices]
        sorted_dists = points[sorted_indices, 3]
        
        gaps = []
        min_gap_angle = math.radians(15)  # Minimum gap width (15 degrees)
        
        for i in range(len(sorted_angles) - 1):
            angle_diff = sorted_angles[i + 1] - sorted_angles[i]
            
            # Check for significant gap
            if angle_diff > min_gap_angle:
                # Gap found - check if it's passable (obstacles on both sides far enough)
                left_dist = sorted_dists[i]
                right_dist = sorted_dists[i + 1]
                
                if left_dist > self.min_range and right_dist > self.min_range:
                    gap_center = (sorted_angles[i] + sorted_angles[i + 1]) / 2
                    avg_dist = (left_dist + right_dist) / 2
                    
                    # Estimate gap width at average distance
                    gap_width = angle_diff * avg_dist
                    
                    if gap_width > 3.0:  # Minimum 3m gap for boat
                        gaps.append({
                            'angle': gap_center,
                            'width': gap_width,
                            'distance': avg_dist
                        })
        
        return gaps

    def _update_obstacle_tracks(self, clusters, current_time):
        """Track obstacle positions over time for velocity estimation"""
        # Simple nearest-neighbor tracking
        for i, cluster in enumerate(clusters):
            cx, cy = cluster['centroid']
            cluster_id = f"obs_{i}"
            
            # Find closest existing track
            best_match = None
            best_dist = 3.0  # Max matching distance
            
            for track_id, track_history in self.obstacle_tracks.items():
                if len(track_history) > 0:
                    last_pos = track_history[-1]
                    dist = math.sqrt((cx - last_pos[0])**2 + (cy - last_pos[1])**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_match = track_id
            
            if best_match:
                # Update existing track
                self.obstacle_tracks[best_match].append((cx, cy, current_time))
                
                # Calculate velocity if enough history
                if len(self.obstacle_tracks[best_match]) >= 3:
                    self._estimate_velocity(best_match)
            else:
                # Create new track
                self.obstacle_tracks[cluster_id] = deque(maxlen=self.velocity_history_size)
                self.obstacle_tracks[cluster_id].append((cx, cy, current_time))
        
        # Clean up old tracks
        self._cleanup_old_tracks(current_time)

    def _estimate_velocity(self, track_id):
        """Estimate obstacle velocity from track history"""
        track = self.obstacle_tracks[track_id]
        if len(track) < 2:
            return
        
        # Use last two positions
        p1 = track[-2]
        p2 = track[-1]
        
        dt = (p2[2] - p1[2]).nanoseconds / 1e9
        if dt > 0:
            vx = (p2[0] - p1[0]) / dt
            vy = (p2[1] - p1[1]) / dt
            self.obstacle_velocities[track_id] = (vx, vy)

    def _cleanup_old_tracks(self, current_time):
        """Remove tracks that haven't been updated recently"""
        stale_tracks = []
        for track_id, track_history in self.obstacle_tracks.items():
            if len(track_history) > 0:
                last_time = track_history[-1][2]
                age = (current_time - last_time).nanoseconds / 1e9
                if age > 1.0:  # 1 second timeout
                    stale_tracks.append(track_id)
        
        for track_id in stale_tracks:
            del self.obstacle_tracks[track_id]
            if track_id in self.obstacle_velocities:
                del self.obstacle_velocities[track_id]

    def _analyze_sectors_adaptive(self, points):
        """Adaptive sector analysis based on target direction"""
        front_points = []
        left_points = []
        right_points = []
        
        # 4. Adaptive sectors - adjust based on target heading
        front_min = -self.front_half_width
        front_max = self.front_half_width
        
        for i in range(len(points)):
            x, y, z, dist = points[i]
            angle = math.atan2(y, x)
            
            if front_min < angle < front_max:  # Front (adaptive width)
                front_points.append(dist)
            elif angle >= front_max and angle <= math.pi * 3/4:  # Left
                left_points.append(dist)
            elif angle <= front_min and angle >= -math.pi * 3/4:  # Right
                right_points.append(dist)
        
        # Calculate clearance per sector (10th percentile for robustness)
        raw_front = self._get_clearance(front_points)
        raw_left = self._get_clearance(left_points)
        raw_right = self._get_clearance(right_points)
        
        # 1. Temporal filtering for sectors
        self.sector_history['front'].append(raw_front)
        self.sector_history['left'].append(raw_left)
        self.sector_history['right'].append(raw_right)
        
        # Use median of recent values for stability
        self.front_clear = np.median(list(self.sector_history['front'])) if self.sector_history['front'] else raw_front
        self.left_clear = np.median(list(self.sector_history['left'])) if self.sector_history['left'] else raw_left
        self.right_clear = np.median(list(self.sector_history['right'])) if self.sector_history['right'] else raw_right

    def _calculate_urgency(self):
        """Calculate distance-weighted urgency scores"""
        self.front_urgency = self._distance_to_urgency(self.front_clear)
        self.left_urgency = self._distance_to_urgency(self.left_clear)
        self.right_urgency = self._distance_to_urgency(self.right_clear)
        
        # Overall urgency is max of all sectors
        self.overall_urgency = max(self.front_urgency, self.left_urgency, self.right_urgency)

    def _distance_to_urgency(self, distance):
        """Convert distance to urgency score (0.0 = safe, 1.0 = critical)"""
        if distance <= self.critical_distance:
            return 1.0
        elif distance >= self.min_safe_distance:
            return 0.0
        else:
            # Linear interpolation
            return 1.0 - (distance - self.critical_distance) / (self.min_safe_distance - self.critical_distance)

    # ==================== v2.1: VFH STEERING ====================

    def _calculate_vfh_steering(self):
        """
        VFH-like steering: bin scan, mark blocked bins within vfh_block_dist,
        find the best clear direction closest to target heading.
        (Ported from AllInOneStack)
        """
        if self.latest_scan is None:
            self.vfh_best_direction = None
            return

        scan = self.latest_scan
        if not scan.ranges:
            self.vfh_best_direction = None
            return

        bin_rad = math.radians(max(self.vfh_bin_deg, 1e-3))
        clearance = math.radians(self.vfh_clearance_deg)
        num_bins = int(math.ceil((scan.angle_max - scan.angle_min) / bin_rad))
        blocked = [False] * num_bins

        # Mark blocked bins
        angle = scan.angle_min
        for rng in scan.ranges:
            idx = int((angle - scan.angle_min) / bin_rad)
            if 0 <= idx < num_bins:
                if rng > 0.0 and rng < self.vfh_block_dist:
                    blocked[idx] = True
            angle += scan.angle_increment

        # Inflate blocked bins by clearance
        inflate_bins = int(math.ceil(clearance / bin_rad))
        if inflate_bins > 0:
            blocked_inf = blocked[:]
            for i, b in enumerate(blocked):
                if not b:
                    continue
                for k in range(-inflate_bins, inflate_bins + 1):
                    j = i + k
                    if 0 <= j < num_bins:
                        blocked_inf[j] = True
            blocked = blocked_inf

        self.vfh_blocked_bins = blocked

        # Find best free bin closest to target heading (0 = straight ahead)
        desired_yaw = self.target_angle  # Target direction
        best_idx = None
        best_err = None

        for i, b in enumerate(blocked):
            if b:
                continue
            center_ang = scan.angle_min + (i + 0.5) * bin_rad
            err = abs(self._normalize_angle(center_ang - desired_yaw))
            if best_err is None or err < best_err:
                best_err = err
                best_idx = i

        if best_idx is not None:
            self.vfh_best_direction = scan.angle_min + (best_idx + 0.5) * bin_rad
        else:
            self.vfh_best_direction = None

    # ==================== v2.1: POLAR HISTOGRAM ====================

    def _calculate_polar_bias(self):
        """
        Polar histogram: accumulate weighted free space left vs right.
        Returns a normalized bias in [-1, 1] (positive => turn left).
        (Ported from AllInOneStack)
        """
        if self.latest_scan is None:
            self.polar_bias = 0.0
            return

        scan = self.latest_scan
        if not scan.ranges:
            self.polar_bias = 0.0
            return

        angle = scan.angle_min
        left_score = 0.0
        right_score = 0.0
        power = max(self.polar_weight_power, 0.0)

        for rng in scan.ranges:
            if rng < self.min_range:
                rng = self.min_range
            if math.isinf(rng) or math.isnan(rng):
                rng = self.max_range
            w = rng ** power
            if angle > 0.0:
                left_score += w
            else:
                right_score += w
            angle += scan.angle_increment

        total = left_score + right_score
        if total <= 0.0:
            self.polar_bias = 0.0
        else:
            self.polar_bias = (left_score - right_score) / total

    # ==================== v2.1: FORCE AVOIDANCE ====================

    def _update_force_avoid_state(self):
        """
        Force avoidance whenever any sector is below full_clear_distance.
        Resume only when all clear. (Ported from AllInOneStack)
        """
        # Use fused clearance values
        f_val = min(self.front_clear, self.scan_front_clear) if self.laserscan_enabled else self.front_clear
        l_val = min(self.left_clear, self.scan_left_clear) if self.laserscan_enabled else self.left_clear
        r_val = min(self.right_clear, self.scan_right_clear) if self.laserscan_enabled else self.right_clear

        if f_val < self.full_clear_distance or l_val < self.full_clear_distance or r_val < self.full_clear_distance:
            self.force_avoid_active = True
        elif f_val >= self.full_clear_distance and l_val >= self.full_clear_distance and r_val >= self.full_clear_distance:
            self.force_avoid_active = False

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _get_clearance(self, distances):
        """Get clearance distance using 10th percentile"""
        if not distances:
            return self.max_range
        
        if isinstance(distances, np.ndarray):
            sorted_dists = np.sort(distances)
        else:
            sorted_dists = sorted(distances)
        
        if len(sorted_dists) > 10:
            return min(self.max_range, sorted_dists[len(sorted_dists)//10])
        return min(self.max_range, sorted_dists[0])

    def publish_status(self):
        """Publish enhanced obstacle information with bilingual logging"""
        # Prepare cluster info for JSON
        cluster_info = []
        for cluster in self.clusters[:5]:  # Limit to 5 closest clusters
            cluster_info.append({
                'x': round(cluster['centroid'][0], 2),
                'y': round(cluster['centroid'][1], 2),
                'size': cluster['size'],
                'distance': round(cluster['min_distance'], 2),
                'angle_deg': round(math.degrees(cluster['angle']), 1)
            })
        
        # Prepare gap info
        gap_info = []
        for gap in self.gaps[:3]:  # Limit to 3 best gaps
            gap_info.append({
                'angle_deg': round(math.degrees(gap['angle']), 1),
                'width': round(gap['width'], 2),
                'distance': round(gap['distance'], 2)
            })
        
        # Prepare velocity info for moving obstacles
        moving_obstacles = []
        for track_id, velocity in self.obstacle_velocities.items():
            speed = math.sqrt(velocity[0]**2 + velocity[1]**2)
            if speed > 0.5:  # Only report if moving > 0.5 m/s
                moving_obstacles.append({
                    'id': track_id,
                    'vx': round(velocity[0], 2),
                    'vy': round(velocity[1], 2),
                    'speed': round(speed, 2)
                })
        
        # Calculate best_gap for BURAN compatibility (direction in degrees, width in degrees)
        best_gap = None
        if self.gaps:
            best = max(self.gaps, key=lambda g: g['width'])
            best_gap = {
                'direction': float(round(math.degrees(best['angle']), 1)),
                'width': float(round(math.degrees(best['width'] / best['distance']), 1)) if best['distance'] > 0 else 0.0,
                'distance': float(round(best['distance'], 1))
            }
        
        # v2.1: Fuse PointCloud2 and LaserScan clearance values (take minimum)
        fused_front = min(self.front_clear, self.scan_front_clear) if self.laserscan_enabled else self.front_clear
        fused_left = min(self.left_clear, self.scan_left_clear) if self.laserscan_enabled else self.left_clear
        fused_right = min(self.right_clear, self.scan_right_clear) if self.laserscan_enabled else self.right_clear
        fused_min = min(fused_front, fused_left, fused_right)

        # v2.1: Build VFH best gap for BURAN
        vfh_gap = None
        if self.vfh_best_direction is not None:
            vfh_gap = {
                'direction': float(round(math.degrees(self.vfh_best_direction), 1)),
                'vfh_blocked_ratio': float(sum(self.vfh_blocked_bins) / max(1, len(self.vfh_blocked_bins)))
            }

        # Publish detailed obstacle info as JSON (enhanced)
        # Convert numpy types to Python native types for JSON serialization
        obstacle_info = {
            'obstacle_detected': bool(self.obstacle_detected),
            'min_distance': float(round(min(self.min_obstacle_distance, fused_min), 2)),
            'front_clear': float(round(fused_front, 2)),
            'left_clear': float(round(fused_left, 2)),
            'right_clear': float(round(fused_right, 2)),
            'is_critical': bool(min(self.min_obstacle_distance, fused_min) < self.critical_distance),
            # BURAN-compatible fields
            'urgency': float(round(self.overall_urgency, 3)),
            'obstacle_count': int(len(self.clusters)),
            'best_gap': best_gap,
            # Enhanced fields (v2.0)
            'front_urgency': float(round(self.front_urgency, 3)),
            'left_urgency': float(round(self.left_urgency, 3)),
            'right_urgency': float(round(self.right_urgency, 3)),
            'overall_urgency': float(round(self.overall_urgency, 3)),
            'clusters': cluster_info,
            'gaps': gap_info,
            'moving_obstacles': moving_obstacles,
            'water_plane_z': float(round(self.water_plane_z, 2)) if self.water_plane_z else None,
            'temporal_confidence': float(len(self.detection_history) / self.temporal_history_size),
            # v2.1: New fields
            'vfh_gap': vfh_gap,
            'polar_bias': float(round(self.polar_bias, 3)),
            'force_avoid_active': bool(self.force_avoid_active),
            'laserscan_fused': bool(self.laserscan_enabled and self.latest_scan is not None),
            'scan_front_clear': float(round(self.scan_front_clear, 2)) if self.laserscan_enabled else None,
            'scan_left_clear': float(round(self.scan_left_clear, 2)) if self.laserscan_enabled else None,
            'scan_right_clear': float(round(self.scan_right_clear, 2)) if self.laserscan_enabled else None
        }
        
        info_msg = String()
        info_msg.data = json.dumps(obstacle_info)
        self.pub_obstacle_info.publish(info_msg)
        
        # Publish simple detection flag
        detected_msg = Bool()
        detected_msg.data = bool(self.obstacle_detected)
        self.pub_obstacle_detected.publish(detected_msg)
        
        # Bilingual logging (throttled)
        if self.obstacle_detected:
            if self.min_obstacle_distance < self.critical_distance:
                self.get_logger().warn(
                    f"🚨 CRITIQUE! {self.min_obstacle_distance:.1f}m (urgence={self.overall_urgency:.0%}) | "
                    f"CRITICAL! (F:{self.front_clear:.1f} L:{self.left_clear:.1f} R:{self.right_clear:.1f})",
                    throttle_duration_sec=1.0
                )
            else:
                gap_hint = ""
                if self.gaps:
                    best_gap = max(self.gaps, key=lambda g: g['width'])
                    gap_hint = f" | GAP: {best_gap['width']:.1f}m @ {math.degrees(best_gap['angle']):.0f}°"
                
                self.get_logger().info(
                    f"⚠️ OBSTACLE: {self.min_obstacle_distance:.1f}m (u={self.overall_urgency:.0%}) "
                    f"(F:{self.front_clear:.1f} L:{self.left_clear:.1f} R:{self.right_clear:.1f}){gap_hint}",
                    throttle_duration_sec=2.0
                )
        else:
            self.get_logger().info(
                f"CLEAR (F:{self.front_clear:.1f} L:{self.left_clear:.1f} R:{self.right_clear:.1f}) "
                f"[{len(self.clusters)} clusters]",
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = OkoPerception()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
