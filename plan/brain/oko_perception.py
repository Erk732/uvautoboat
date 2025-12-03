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

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool


class OkoPerception(Node):
    """
    OKO - "Œil" (Référence au système d'alerte précoce par satellite)
    Système de perception et détection d'obstacles
    
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
        self.declare_parameter('temporal_history_size', 5)    # Scans to keep in history
        self.declare_parameter('temporal_threshold', 3)       # Min detections to confirm
        self.declare_parameter('cluster_distance', 2.0)       # Max distance between cluster points
        self.declare_parameter('min_cluster_size', 5)         # Min points per cluster
        self.declare_parameter('water_plane_threshold', 0.5)  # Tolerance for water plane removal
        self.declare_parameter('velocity_history_size', 10)   # Frames for velocity estimation

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

        # --- PUBLISHERS ---
        self.pub_obstacle_info = self.create_publisher(
            String, '/perception/obstacle_info', 10
        )
        self.pub_obstacle_detected = self.create_publisher(
            Bool, '/perception/obstacle_detected', 10
        )

        # Publish at 20Hz
        self.create_timer(0.05, self.publish_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("OKO v2.0 - Enhanced Obstacle Detection System")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Safe Distance: {self.min_safe_distance}m | Critical: {self.critical_distance}m")
        self.get_logger().info(f"Height Filter: {self.min_height}m to {self.max_height}m")
        self.get_logger().info(f"Temporal Filter: {self.temporal_threshold}/{self.temporal_history_size} scans")
        self.get_logger().info(f"Clustering: {self.cluster_distance}m eps, {self.min_cluster_size} min points")
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
        
        # Publish detailed obstacle info as JSON (enhanced)
        obstacle_info = {
            'obstacle_detected': self.obstacle_detected,
            'min_distance': round(self.min_obstacle_distance, 2),
            'front_clear': round(self.front_clear, 2),
            'left_clear': round(self.left_clear, 2),
            'right_clear': round(self.right_clear, 2),
            'is_critical': self.min_obstacle_distance < self.critical_distance,
            # Enhanced fields (v2.0)
            'front_urgency': round(self.front_urgency, 3),
            'left_urgency': round(self.left_urgency, 3),
            'right_urgency': round(self.right_urgency, 3),
            'overall_urgency': round(self.overall_urgency, 3),
            'clusters': cluster_info,
            'gaps': gap_info,
            'moving_obstacles': moving_obstacles,
            'water_plane_z': round(self.water_plane_z, 2) if self.water_plane_z else None,
            'temporal_confidence': len(self.detection_history) / self.temporal_history_size
        }
        
        info_msg = String()
        info_msg.data = json.dumps(obstacle_info)
        self.pub_obstacle_info.publish(info_msg)
        
        # Publish simple detection flag
        detected_msg = Bool()
        detected_msg.data = self.obstacle_detected
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
                f"✅ DÉGAGÉ | CLEAR (F:{self.front_clear:.1f} L:{self.left_clear:.1f} R:{self.right_clear:.1f}) "
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
