"""
LIDAR-based Real-Time Obstacle Avoidance Module
SHARED MODULE: Keep a copy of this in both 'plan' and 'control' packages.
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
                 min_distance: float = 0.3,    
                 max_distance: float = 50.0,
                 min_height: float = -10.0,   # CHANGED: Allow very low objects
                 max_height: float = 10.0,    # CHANGED: Allow very high objects
                 z_filter_enabled: bool = False): # CHANGED: Disabled by default for safety
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.min_height = min_height
        self.max_height = max_height
        self.z_filter_enabled = z_filter_enabled
        
    def process_pointcloud(self, data: bytes, point_step: int, 
                          sampling_factor: int = 10) -> List[Obstacle]:
        obstacles = []
        # Process loop
        for i in range(0, len(data) - point_step, point_step * sampling_factor):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                
                # Skip invalid points
                if math.isnan(x) or math.isinf(x) or math.isnan(y) or math.isinf(y):
                    continue
                
                distance = math.sqrt(x*x + y*y)
                
                # Filters
                if distance < self.min_distance or distance > self.max_distance:
                    continue
                
                # Only filter by height if explicitly enabled
                if self.z_filter_enabled and (z < self.min_height or z > self.max_height):
                    continue
                
                angle = math.atan2(y, x)
                obstacles.append(Obstacle(x=x, y=y, z=z, distance=distance, angle=angle))
                
            except struct.error:
                continue
        
        return obstacles

class ObstacleClustering:
    """Groups nearby obstacles into coherent clusters"""
    def __init__(self, cluster_radius: float = 2.0):
        self.cluster_radius = cluster_radius
    
    def cluster_obstacles(self, obstacles: List[Obstacle]) -> List[List[Obstacle]]:
        if not obstacles: return []
        clusters = []
        used = set()
        
        for i, obs in enumerate(obstacles):
            if i in used: continue
            cluster = [obs]
            used.add(i)
            for j, other in enumerate(obstacles):
                if j in used: continue
                dist = math.sqrt((obs.x - other.x)**2 + (obs.y - other.y)**2)
                if dist < self.cluster_radius:
                    cluster.append(other)
                    used.add(j)
            clusters.append(cluster)
        return clusters

class ObstacleAvoider:
    def __init__(self, safe_distance: float = 10.0, look_ahead: float = 15.0):
        self.safe_distance = safe_distance
        self.look_ahead = look_ahead
    
    def get_avoidance_waypoint(self, clusters, current_pos, target_waypoint):
        if not clusters: return None
        
        # Find closest obstacle in front
        min_dist = float('inf')
        closest_obs = None
        for cluster in clusters:
            for obs in cluster:
                if obs.x < 0: continue # Ignore behind
                if obs.distance < min_dist:
                    min_dist = obs.distance
                    closest_obs = obs
        
        if closest_obs is None or min_dist > self.safe_distance:
            return None

        curr_x, curr_y = current_pos
        tgt_x, tgt_y = target_waypoint
        
        target_dx = tgt_x - curr_x
        target_dy = tgt_y - curr_y
        target_dist = math.sqrt(target_dx**2 + target_dy**2)
        
        if target_dist < 0.1:
            return None 
        
        target_dx /= target_dist
        target_dy /= target_dist
        
        # Perpendicular shift
        perp_dx = -target_dy
        perp_dy = target_dx
        
        if closest_obs.y > 0: # Obstacle Left -> Go Right
            avoidance_x = curr_x + perp_dx * self.safe_distance + target_dx * self.look_ahead
            avoidance_y = curr_y + perp_dy * self.safe_distance + target_dy * self.look_ahead
        else: # Obstacle Right -> Go Left
            avoidance_x = curr_x - perp_dx * self.safe_distance + target_dx * self.look_ahead
            avoidance_y = curr_y - perp_dy * self.safe_distance + target_dy * self.look_ahead
            
        return (avoidance_x, avoidance_y)

class RealtimeObstacleMonitor:
    def __init__(self):
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
    
    def analyze_sectors(self, obstacles: List[Obstacle]):
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        for obs in obstacles:
            angle = obs.angle
            dist = obs.distance
            if -math.pi/4 < angle < math.pi/4:
                self.front_distance = min(self.front_distance, dist)
            elif math.pi/4 <= angle <= 3*math.pi/4:
                self.left_distance = min(self.left_distance, dist)
            elif -3*math.pi/4 <= angle <= -math.pi/4:
                self.right_distance = min(self.right_distance, dist)

    def get_best_direction(self) -> str:
        if self.left_distance > self.right_distance: return "LEFT"
        elif self.right_distance > self.left_distance: return "RIGHT"
        else: return "FRONT"
    
    def is_critical(self, threshold: float = 5.0) -> bool:
        return (self.front_distance < threshold or 
                self.left_distance < threshold or 
                self.right_distance < threshold)