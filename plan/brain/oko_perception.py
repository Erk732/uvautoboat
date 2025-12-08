#!/usr/bin/env python3
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

        # --- CRITICAL FIXES FOR PIER ---
        self.declare_parameter('min_safe_distance', 12.0)
        self.declare_parameter('critical_distance', 4.0)
        self.declare_parameter('min_height', -1.0)  # Catch water line
        self.declare_parameter('max_height', 3.0)
        self.declare_parameter('min_range', 2.0)    # Look closer (was 5.0)
        self.declare_parameter('max_range', 100.0)
        self.declare_parameter('sample_rate', 1)
        
        # Tuning
        self.declare_parameter('temporal_history_size', 3)
        self.declare_parameter('temporal_threshold', 2)
        self.declare_parameter('cluster_distance', 2.0)
        self.declare_parameter('min_cluster_size', 3)

        # Get params
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.temporal_threshold = self.get_parameter('temporal_threshold').value

        # State
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        
        self.detection_history = deque(maxlen=self.get_parameter('temporal_history_size').value)
        self.clusters = []

        # Pub/Sub
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
        self.pub_obstacle_info = self.create_publisher(String, '/perception/obstacle_info', 10)
        
        self.get_logger().info("OKO Perception Ready (Pier Config Loaded)")

    def lidar_callback(self, msg):
        points = []
        point_step = msg.point_step
        data = msg.data
        
        for i in range(0, len(data) - point_step, point_step * self.sample_rate):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                dist = math.sqrt(x*x + y*y)
                
                # Filter Logic
                if math.isnan(x): continue
                if dist < self.min_range or dist > self.max_range: continue
                if z < self.min_height or z > self.max_height: continue
                if x < 0.0: continue # Only forward points
                
                points.append((x, y, z, dist))
            except: continue
            
        if not points:
            self._reset_detection()
            return

        # Process Points
        points_np = np.array(points)
        raw_min = np.min(points_np[:, 3])
        
        # Temporal Filter
        self.detection_history.append(raw_min < self.min_safe_distance)
        self.obstacle_detected = sum(self.detection_history) >= self.temporal_threshold
        
        self.min_obstacle_distance = raw_min
        self.clusters = self._cluster_obstacles(points_np)
        self._analyze_sectors(points_np)
        
        self.publish_info()

    def _reset_detection(self):
        self.obstacle_detected = False
        self.min_obstacle_distance = self.max_range
        self.front_clear = self.max_range
        self.left_clear = self.max_range
        self.right_clear = self.max_range
        self.clusters = []
        self.publish_info()

    def _cluster_obstacles(self, points):
        # Simplified clustering for speed
        if len(points) < 5: return []
        # Return just the centroid of all points for simplicity in A* mapping
        # (A* treats them as individual blocks anyway)
        cx = np.mean(points[:, 0])
        cy = np.mean(points[:, 1])
        return [{'x': float(cx), 'y': float(cy)}]

    def _analyze_sectors(self, points):
        # Simple Sector Analysis
        front = points[np.abs(np.arctan2(points[:,1], points[:,0])) < 0.5]
        left = points[(np.arctan2(points[:,1], points[:,0]) > 0.5) & (np.arctan2(points[:,1], points[:,0]) < 2.0)]
        right = points[(np.arctan2(points[:,1], points[:,0]) < -0.5) & (np.arctan2(points[:,1], points[:,0]) > -2.0)]
        
        self.front_clear = np.min(front[:, 3]) if len(front) > 0 else self.max_range
        self.left_clear = np.min(left[:, 3]) if len(left) > 0 else self.max_range
        self.right_clear = np.min(right[:, 3]) if len(right) > 0 else self.max_range

    def publish_info(self):
        msg = String()
        info = {
            'obstacle_detected': bool(self.obstacle_detected),
            'min_distance': float(round(self.min_obstacle_distance, 2)),
            'front_clear': float(round(self.front_clear, 2)),
            'left_clear': float(round(self.left_clear, 2)),
            'right_clear': float(round(self.right_clear, 2)),
            'is_critical': bool(self.min_obstacle_distance < self.critical_distance),
            'clusters': self.clusters
        }
        msg.data = json.dumps(info)
        self.pub_obstacle_info.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OkoPerception()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()