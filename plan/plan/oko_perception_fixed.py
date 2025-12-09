#!/usr/bin/env python3
"""
OKO Perception - Enhanced Obstacle Detection

Features:
- LiDAR point cloud processing
- Temporal filtering for stability
- Cluster detection with centroids for A* planning
- Sector analysis (front, left, right)
- Urgency calculation for reactive avoidance
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
    def __init__(self):
        super().__init__('oko_perception_node')

        # --- DETECTION PARAMETERS ---
        self.declare_parameter('min_safe_distance', 12.0)
        self.declare_parameter('critical_distance', 4.0)
        self.declare_parameter('min_height', -1.0)  # Catch water line
        self.declare_parameter('max_height', 3.0)
        self.declare_parameter('min_range', 2.0)    # Look closer
        self.declare_parameter('max_range', 100.0)
        self.declare_parameter('sample_rate', 1)    # Process every Nth point
        
        # Temporal filtering
        self.declare_parameter('temporal_history_size', 3)
        self.declare_parameter('temporal_threshold', 2)
        
        # Clustering
        self.declare_parameter('cluster_distance', 3.0)
        self.declare_parameter('min_cluster_size', 5)
        
        # Sector angles (radians)
        self.declare_parameter('front_sector_angle', 0.5)   # ~30 degrees each side
        self.declare_parameter('side_sector_inner', 0.5)
        self.declare_parameter('side_sector_outer', 2.0)

        # Get parameters
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.temporal_threshold = self.get_parameter('temporal_threshold').value
        self.cluster_distance = self.get_parameter('cluster_distance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        
        # Sector angles
        self.front_angle = self.get_parameter('front_sector_angle').value
        self.side_inner = self.get_parameter('side_sector_inner').value
        self.side_outer = self.get_parameter('side_sector_outer').value

        # State
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.urgency = 0.0
        self.best_gap = None
        
        self.detection_history = deque(maxlen=self.get_parameter('temporal_history_size').value)
        self.clusters = []
        self.obstacle_count = 0

        # Pub/Sub
        self.create_subscription(
            PointCloud2, 
            '/wamv/sensors/lidars/lidar_wamv_sensor/points', 
            self.lidar_callback, 
            10
        )
        self.pub_obstacle_info = self.create_publisher(String, '/perception/obstacle_info', 10)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("OKO Perception System")
        self.get_logger().info("Enhanced for A* Planning Integration")
        self.get_logger().info(f"Range: {self.min_range}m - {self.max_range}m")
        self.get_logger().info(f"Height: {self.min_height}m - {self.max_height}m")
        self.get_logger().info(f"Clustering: {self.cluster_distance}m")
        self.get_logger().info("=" * 50)

    def lidar_callback(self, msg):
        """Process LiDAR point cloud"""
        points = []
        point_step = msg.point_step
        data = msg.data
        
        # Extract valid points
        for i in range(0, len(data) - point_step, point_step * self.sample_rate):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                
                # Filter out invalid points
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    continue
                
                dist = math.sqrt(x*x + y*y)
                
                # Range filter
                if dist < self.min_range or dist > self.max_range:
                    continue
                
                # Height filter
                if z < self.min_height or z > self.max_height:
                    continue
                
                # Only forward-facing points
                if x < 0.0:
                    continue
                
                points.append((x, y, z, dist))
                
            except:
                continue
        
        # No valid points
        if not points:
            self._reset_detection()
            return

        # Convert to numpy for faster processing
        points_np = np.array(points)
        
        # Calculate raw minimum distance
        raw_min = np.min(points_np[:, 3])
        
        # Temporal filtering for stability
        self.detection_history.append(raw_min < self.min_safe_distance)
        self.obstacle_detected = sum(self.detection_history) >= self.temporal_threshold
        
        self.min_obstacle_distance = raw_min
        
        # Cluster obstacles for A* planning
        self.clusters = self._cluster_obstacles(points_np)
        self.obstacle_count = len(self.clusters)
        
        # Analyze sectors for reactive avoidance
        self._analyze_sectors(points_np)
        
        # Calculate urgency (0.0 = safe, 1.0 = critical)
        self._calculate_urgency()
        
        # Find best navigation gap
        self._find_best_gap()
        
        # Publish info
        self.publish_info()

    def _reset_detection(self):
        """Reset all detection state"""
        self.obstacle_detected = False
        self.min_obstacle_distance = self.max_range
        self.front_clear = self.max_range
        self.left_clear = self.max_range
        self.right_clear = self.max_range
        self.clusters = []
        self.obstacle_count = 0
        self.urgency = 0.0
        self.best_gap = None
        self.publish_info()

    def _cluster_obstacles(self, points):
        """
        Cluster nearby points into obstacle groups
        Returns list of cluster centroids for A* planning
        """
        if len(points) < self.min_cluster_size:
            return []
        
        # Simple distance-based clustering
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if used[i]:
                continue
            
            # Start new cluster
            cluster_points = [i]
            used[i] = True
            
            # Find nearby points
            for j in range(i+1, len(points)):
                if used[j]:
                    continue
                
                # Check distance to any point in current cluster
                for k in cluster_points:
                    dist = math.hypot(
                        points[j, 0] - points[k, 0],
                        points[j, 1] - points[k, 1]
                    )
                    if dist < self.cluster_distance:
                        cluster_points.append(j)
                        used[j] = True
                        break
            
            # Only keep clusters with enough points
            if len(cluster_points) >= self.min_cluster_size:
                # Calculate centroid
                cluster_data = points[cluster_points]
                cx = np.mean(cluster_data[:, 0])
                cy = np.mean(cluster_data[:, 1])
                cz = np.mean(cluster_data[:, 2])
                
                clusters.append({
                    'x': float(cx),
                    'y': float(cy),
                    'z': float(cz),
                    'size': len(cluster_points),
                    'min_dist': float(np.min(cluster_data[:, 3]))
                })
        
        return clusters

    def _analyze_sectors(self, points):
        """
        Analyze clearance in front, left, and right sectors
        """
        # Calculate angles for all points
        angles = np.arctan2(points[:, 1], points[:, 0])
        distances = points[:, 3]
        
        # Front sector: ±front_angle
        front_mask = np.abs(angles) < self.front_angle
        front_points = distances[front_mask]
        self.front_clear = float(np.min(front_points)) if len(front_points) > 0 else self.max_range
        
        # Left sector: side_inner to side_outer
        left_mask = (angles > self.side_inner) & (angles < self.side_outer)
        left_points = distances[left_mask]
        self.left_clear = float(np.min(left_points)) if len(left_points) > 0 else self.max_range
        
        # Right sector: -side_outer to -side_inner
        right_mask = (angles < -self.side_inner) & (angles > -self.side_outer)
        right_points = distances[right_mask]
        self.right_clear = float(np.min(right_points)) if len(right_points) > 0 else self.max_range

    def _calculate_urgency(self):
        """
        Calculate urgency level (0.0 = safe, 1.0 = critical)
        Based on closest obstacle distance
        """
        if self.min_obstacle_distance >= self.min_safe_distance:
            self.urgency = 0.0
        elif self.min_obstacle_distance <= self.critical_distance:
            self.urgency = 1.0
        else:
            # Linear interpolation between critical and safe
            self.urgency = 1.0 - (
                (self.min_obstacle_distance - self.critical_distance) /
                (self.min_safe_distance - self.critical_distance)
            )

    def _find_best_gap(self):
        """
        Find the best navigation gap
        Returns direction and width of clearest path
        """
        # Sample directions from -90° to +90° (left to right)
        num_samples = 18  # 10° increments
        max_angle = math.pi / 2
        
        gap_scores = []
        
        for i in range(num_samples):
            angle = -max_angle + (2 * max_angle * i / (num_samples - 1))
            
            # Calculate clearance in this direction
            # Use front, left, right clearances as proxy
            if abs(angle) < self.front_angle:
                clearance = self.front_clear
            elif angle > 0:
                clearance = self.left_clear
            else:
                clearance = self.right_clear
            
            gap_scores.append({
                'direction': math.degrees(angle),
                'clearance': clearance
            })
        
        # Find best gap (longest clearance)
        if gap_scores:
            best = max(gap_scores, key=lambda x: x['clearance'])
            
            # Calculate gap width (rough estimate)
            # Count adjacent directions with good clearance
            width = 0
            threshold = self.min_safe_distance
            for gap in gap_scores:
                if gap['clearance'] > threshold:
                    width += 10  # Each sample is ~10°
            
            if best['clearance'] > self.min_safe_distance:
                self.best_gap = {
                    'direction': best['direction'],
                    'width': width,
                    'clearance': best['clearance']
                }
            else:
                self.best_gap = None
        else:
            self.best_gap = None

    def publish_info(self):
        """Publish obstacle information"""
        msg = String()
        info = {
            'obstacle_detected': bool(self.obstacle_detected),
            'min_distance': float(round(self.min_obstacle_distance, 2)),
            'front_clear': float(round(self.front_clear, 2)),
            'left_clear': float(round(self.left_clear, 2)),
            'right_clear': float(round(self.right_clear, 2)),
            'is_critical': bool(self.min_obstacle_distance < self.critical_distance),
            'urgency': float(round(self.urgency, 2)),
            'obstacle_count': int(self.obstacle_count),
            'best_gap': self.best_gap,
            'clusters': self.clusters  # For A* planning
        }
        msg.data = json.dumps(info)
        self.pub_obstacle_info.publish(msg)


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