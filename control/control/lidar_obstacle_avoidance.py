"""
LIDAR-based Real-Time Obstacle Avoidance Module
SHARED MODULE: Keep a copy of this in both 'plan' and 'control' packages.
"""

import json
import math
import struct
from typing import List, Tuple, Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

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
                 min_height: float = -0.2,
                 max_height: float = 3.0,
                 z_filter_enabled: bool = True):
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
        
        # FIX: Avoid division by zero or micro-movements
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


class LidarObstacleAvoidanceNode(Node):
    """Lightweight ROS2 node wrapping the library for quick CLI testing."""

    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')

        # Parameters mirror the underlying helper classes
        self.declare_parameter('sampling_factor', 10)
        self.declare_parameter('cluster_radius', 2.0)
        self.declare_parameter('safe_distance', 10.0)
        self.declare_parameter('look_ahead', 15.0)
        self.declare_parameter('min_distance', 0.3)
        self.declare_parameter('max_distance', 50.0)
        self.declare_parameter('min_height', -0.2)
        self.declare_parameter('max_height', 3.0)
        self.declare_parameter('z_filter_enabled', True)

        # Core processing helpers
        self.detector = LidarObstacleDetector(
            min_distance=self.get_parameter('min_distance').value,
            max_distance=self.get_parameter('max_distance').value,
            min_height=self.get_parameter('min_height').value,
            max_height=self.get_parameter('max_height').value,
            z_filter_enabled=self.get_parameter('z_filter_enabled').value,
        )
        self.clusterer = ObstacleClustering(
            cluster_radius=self.get_parameter('cluster_radius').value
        )
        self.avoider = ObstacleAvoider(
            safe_distance=self.get_parameter('safe_distance').value,
            look_ahead=self.get_parameter('look_ahead').value,
        )
        self.monitor = RealtimeObstacleMonitor()

        # ROS I/O
        self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(String, '/perception/lidar_obstacle_status', 10)

        self.get_logger().info("Lidar obstacle avoidance node ready (listening to lidar_wamv_sensor).")

    def lidar_callback(self, msg: PointCloud2):
        try:
            sampling_factor = int(self.get_parameter('sampling_factor').value)
            obstacles = self.detector.process_pointcloud(msg.data, msg.point_step, sampling_factor)
            clusters = self.clusterer.cluster_obstacles(obstacles)
            self.monitor.analyze_sectors(obstacles)

            critical_thresh = float(self.get_parameter('safe_distance').value)
            critical = self.monitor.is_critical(critical_thresh)

            status = {
                'obstacle_count': len(obstacles),
                'cluster_count': len(clusters),
                'front_distance': round(self.monitor.front_distance, 2),
                'left_distance': round(self.monitor.left_distance, 2),
                'right_distance': round(self.monitor.right_distance, 2),
                'best_direction': self.monitor.get_best_direction(),
                'critical': critical,
            }
            msg_out = String()
            msg_out.data = json.dumps(status)
            self.status_pub.publish(msg_out)

            log_fn = self.get_logger().warn if critical else self.get_logger().info
            log_fn(
                f"LIDAR obs: {status['obstacle_count']} pts, clusters={status['cluster_count']} | "
                f"F:{status['front_distance']}m L:{status['left_distance']}m R:{status['right_distance']}m | "
                f"dir={status['best_direction']}",
                throttle_duration_sec=1.0,
            )
        except Exception as e:
            self.get_logger().error(f"Lidar callback failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
