"""
LIDAR-based Real-Time Obstacle Avoidance Module
- Detects obstacles from LIDAR point cloud
- Clusters nearby obstacles
- Provides avoidance waypoints to return to original path
- Integrates with path planner and controller
"""

import math
import struct
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Obstacle:
    """Represents a detected obstacle"""
    x: float
    y: float
    z: float
    distance: float
    angle: float
    confidence: int = 1


class LidarObstacleDetector:
    """Detects and tracks obstacles from LIDAR point cloud"""
    
    def __init__(self, 
                 min_distance: float = 3.0,    # Reduced from 5.0 for earlier detection
                 max_distance: float = 50.0,
                 min_height: float = -0.2,
                 max_height: float = 3.0,
                 z_filter_enabled: bool = True):
        """
        Initialize obstacle detector
        
        Args:
            min_distance: Ignore points closer than this (self-filter)
            max_distance: Ignore points farther than this
            min_height: Ignore points below this height (ground)
            max_height: Ignore points above this height (sky)
            z_filter_enabled: Enable height-based filtering
        """
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.min_height = min_height
        self.max_height = max_height
        self.z_filter_enabled = z_filter_enabled
        self.obstacle_history: List[Tuple[float, float, int]] = []
        self.max_history = 100
        
    def process_pointcloud(self, data: bytes, point_step: int, 
                          sampling_factor: int = 10) -> List[Obstacle]:
        """
        Extract obstacles from LIDAR point cloud
        
        Args:
            data: Raw PointCloud2 data
            point_step: Bytes per point
            sampling_factor: Process every Nth point (save CPU)
            
        Returns:
            List of detected obstacles
        """
        obstacles = []
        
        for i in range(0, len(data) - point_step, point_step * sampling_factor):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                
                # Skip invalid points
                if math.isnan(x) or math.isinf(x) or math.isnan(y) or math.isinf(y):
                    continue
                
                distance = math.sqrt(x*x + y*y)
                
                # Apply distance filter
                if distance < self.min_distance or distance > self.max_distance:
                    continue
                
                # Apply height filter if enabled
                if self.z_filter_enabled and (z < self.min_height or z > self.max_height):
                    continue
                
                # Calculate angle for later use
                angle = math.atan2(y, x)
                
                obs = Obstacle(
                    x=x, y=y, z=z,
                    distance=distance,
                    angle=angle
                )
                obstacles.append(obs)
                
            except struct.error:
                continue
        
        return obstacles


class ObstacleClustering:
    """Groups nearby obstacles into coherent clusters"""
    
    def __init__(self, cluster_radius: float = 2.0):
        """
        Initialize clustering
        
        Args:
            cluster_radius: Points within this distance belong to same cluster
        """
        self.cluster_radius = cluster_radius
    
    def cluster_obstacles(self, obstacles: List[Obstacle]) -> List[List[Obstacle]]:
        """
        Group obstacles into clusters using simple spatial clustering
        
        Args:
            obstacles: List of detected obstacles
            
        Returns:
            List of obstacle clusters
        """
        if not obstacles:
            return []
        
        clusters = []
        used = set()
        
        for i, obs in enumerate(obstacles):
            if i in used:
                continue
            
            # Start new cluster
            cluster = [obs]
            used.add(i)
            
            # Find neighbors
            for j, other in enumerate(obstacles):
                if j in used:
                    continue
                
                dist = math.sqrt((obs.x - other.x)**2 + (obs.y - other.y)**2)
                if dist < self.cluster_radius:
                    cluster.append(other)
                    used.add(j)
            
            clusters.append(cluster)
        
        return clusters


class AvoidanceWaypoint:
    """Represents a waypoint to avoid obstacles and return to path"""
    
    def __init__(self, x: float, y: float, reason: str = ""):
        self.x = x
        self.y = y
        self.reason = reason


class ObstacleAvoider:
    """Generates avoidance waypoints to navigate around obstacles"""
    
    def __init__(self,
                 safe_distance: float = 10.0,
                 look_ahead: float = 15.0):
        """
        Initialize avoidance planner
        
        Args:
            safe_distance: Keep at least this far from obstacles
            look_ahead: How far ahead to plan avoidance
        """
        self.safe_distance = safe_distance
        self.look_ahead = look_ahead
        self.last_avoidance_point: Optional[Tuple[float, float]] = None
    
    def get_avoidance_waypoint(self, 
                               clusters: List[List[Obstacle]],
                               current_pos: Tuple[float, float],
                               target_waypoint: Tuple[float, float]) -> Optional[AvoidanceWaypoint]:
        """
        Generate an intermediate waypoint to avoid obstacles
        
        Args:
            clusters: List of obstacle clusters
            current_pos: Current position (x, y)
            target_waypoint: Target waypoint to eventually reach
            
        Returns:
            Avoidance waypoint or None if path is clear
        """
        if not clusters:
            return None
        
        # Find closest obstacle
        min_dist = float('inf')
        closest_obs = None
        
        for cluster in clusters:
            for obs in cluster:
                # Only care about obstacles in front
                if obs.x < 0:  # Behind us
                    continue
                
                dist = obs.distance
                if dist < min_dist:
                    min_dist = dist
                    closest_obs = obs
        
        if closest_obs is None or min_dist > self.safe_distance:
            return None  # Path is clear
        
        # Generate avoidance waypoint perpendicular to obstacle
        curr_x, curr_y = current_pos
        tgt_x, tgt_y = target_waypoint
        
        # Direction to target
        target_dx = tgt_x - curr_x
        target_dy = tgt_y - curr_y
        target_dist = math.sqrt(target_dx**2 + target_dy**2)
        
        if target_dist < 0.1:
            target_dist = 0.1
        
        # Normalize
        target_dx /= target_dist
        target_dy /= target_dist
        
        # Perpendicular direction (rotate 90 degrees)
        perp_dx = -target_dy
        perp_dy = target_dx
        
        # Choose direction away from closest obstacle
        if closest_obs.y > 0:  # Obstacle on left
            avoidance_x = curr_x + perp_dx * self.safe_distance + target_dx * self.look_ahead
            avoidance_y = curr_y + perp_dy * self.safe_distance + target_dy * self.look_ahead
        else:  # Obstacle on right
            avoidance_x = curr_x - perp_dx * self.safe_distance + target_dx * self.look_ahead
            avoidance_y = curr_y - perp_dy * self.safe_distance + target_dy * self.look_ahead
        
        self.last_avoidance_point = (avoidance_x, avoidance_y)
        
        return AvoidanceWaypoint(
            avoidance_x, avoidance_y,
            reason=f"Avoid obstacle at ({closest_obs.x:.1f}, {closest_obs.y:.1f})"
        )
    
    def can_return_to_path(self, 
                          clusters: List[List[Obstacle]],
                          current_pos: Tuple[float, float],
                          target_waypoint: Tuple[float, float]) -> bool:
        """
        Check if path is clear to return to original target
        
        Args:
            clusters: Current obstacle clusters
            current_pos: Current position
            target_waypoint: Original target waypoint
            
        Returns:
            True if safe to return to path
        """
        if not clusters:
            return True
        
        # Check if any obstacle is in direct path to target
        curr_x, curr_y = current_pos
        tgt_x, tgt_y = target_waypoint
        
        for cluster in clusters:
            for obs in cluster:
                # Simple check: is obstacle within safety distance of path?
                # Distance from obstacle to line between current and target
                dx = tgt_x - curr_x
                dy = tgt_y - curr_y
                
                if dx == 0 and dy == 0:
                    continue
                
                dist_line = abs(dy * obs.x - dx * obs.y + tgt_x * curr_y - tgt_y * curr_x) / math.sqrt(dx**2 + dy**2)
                
                if dist_line < self.safe_distance and obs.x > 0:  # In front
                    return False
        
        return True


class RealtimeObstacleMonitor:
    """Monitors obstacle status in real-time with sector analysis"""
    
    def __init__(self):
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.sector_angles = {
            'front': (-math.pi/4, math.pi/4),
            'left': (math.pi/4, 3*math.pi/4),
            'right': (-3*math.pi/4, -math.pi/4)
        }
    
    def analyze_sectors(self, obstacles: List[Obstacle]):
        """Analyze obstacles by direction sectors"""
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        for obs in obstacles:
            angle = obs.angle
            dist = obs.distance
            
            # Front sector
            if self.sector_angles['front'][0] < angle < self.sector_angles['front'][1]:
                self.front_distance = min(self.front_distance, dist)
            
            # Left sector
            elif self.sector_angles['left'][0] <= angle <= self.sector_angles['left'][1]:
                self.left_distance = min(self.left_distance, dist)
            
            # Right sector
            elif (angle <= self.sector_angles['right'][1] or 
                  angle >= self.sector_angles['right'][0]):
                self.right_distance = min(self.right_distance, dist)
    
    def get_best_direction(self) -> str:
        """Recommend best direction: LEFT, RIGHT, or FRONT"""
        if self.left_distance > self.right_distance:
            return "LEFT"
        elif self.right_distance > self.left_distance:
            return "RIGHT"
        else:
            return "FRONT"
    
    def is_critical(self, threshold: float = 5.0) -> bool:
        """Check if any obstacle is critically close"""
        return (self.front_distance < threshold or 
                self.left_distance < threshold or 
                self.right_distance < threshold)
