#!/usr/bin/env python3
"""
Perception Node - 3D LIDAR Point Cloud Processing

Part of the modular Vostok1 architecture.
Subscribes to 3D LIDAR data, processes point cloud, publishes obstacle information.

Topics:
    Subscribes:
        /wamv/sensors/lidars/lidar_wamv_sensor/points (PointCloud2)
    
    Publishes:
        /perception/obstacle_info (String) - JSON with obstacle distances per sector
        /perception/obstacle_detected (Bool) - Simple obstacle detection flag
"""

import rclpy
from rclpy.node import Node
import math
import struct
import json

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool


class OkoPerception(Node):
    """
    OKO (ОКО) - "Eye" in Russian
    Soviet-era satellite early warning system reference
    """
    def __init__(self):
        super().__init__('oko_perception_node')

        # --- PARAMETERS ---
        # Increased sensitivity for small obstacles like buoys
        self.declare_parameter('min_safe_distance', 12.0)  # Reduced from 15 for earlier detection
        self.declare_parameter('critical_distance', 4.0)   # Reduced from 5
        self.declare_parameter('hysteresis_distance', 1.5) # Reduced from 2
        self.declare_parameter('min_height', -10.0)  # Relaxed - LiDAR frame varies
        self.declare_parameter('max_height', 20.0)   # Relaxed - catch all obstacles
        self.declare_parameter('min_range', 0.5)    # Reduced from 1.0 to catch closer objects
        self.declare_parameter('max_range', 100.0)  # Ignore points farther than this
        self.declare_parameter('sample_rate', 2)    # Process every 2nd point for better detection

        # Get parameters
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.hysteresis_distance = self.get_parameter('hysteresis_distance').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # --- STATE ---
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')

        # --- SUBSCRIBER ---
        self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data
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

        self.get_logger().info("=" * 50)
        self.get_logger().info("ОКО (OKO) - Система Обнаружения Препятствий")
        self.get_logger().info("OKO Perception - 3D LIDAR Processing")
        self.get_logger().info(f"Безопасная дистанция | Safe Distance: {self.min_safe_distance}m")
        self.get_logger().info(f"Critical Distance: {self.critical_distance}m")
        self.get_logger().info(f"Height Filter: {self.min_height}m to {self.max_height}m")
        self.get_logger().info("=" * 50)

    def lidar_callback(self, msg):
        """Process 3D LIDAR point cloud for obstacle detection"""
        points = []
        total_points = 0
        valid_points = 0
        height_filtered = 0
        range_filtered = 0
        behind_filtered = 0
        
        point_step = msg.point_step
        data = msg.data
        
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
                # Reduced threshold from 0.5 to 0.2 to catch more obstacles
                if x < 0.2:
                    behind_filtered += 1
                    continue
                
                points.append((x, y, z, dist))
                    
            except (struct.error, Exception):
                continue
        
        # Debug logging (throttled)
        self.get_logger().info(
            f"LiDAR: {total_points} total, {valid_points} valid, "
            f"height_filt={height_filtered}, range_filt={range_filtered}, behind={behind_filtered}, "
            f"final={len(points)}",
            throttle_duration_sec=2.0
        )
        
        if not points:
            self.min_obstacle_distance = self.max_range
            if not self.obstacle_detected:
                self.front_clear = self.max_range
                self.left_clear = self.max_range
                self.right_clear = self.max_range
            return
        
        # Get minimum distance
        distances = [p[3] for p in points]
        self.min_obstacle_distance = min(distances)

        # Hysteresis for obstacle detection
        if self.obstacle_detected:
            exit_threshold = self.min_safe_distance + self.hysteresis_distance
            self.obstacle_detected = self.min_obstacle_distance < exit_threshold
        else:
            self.obstacle_detected = self.min_obstacle_distance < self.min_safe_distance

        # Analyze sectors
        self.analyze_sectors(points)

    def analyze_sectors(self, points):
        """Divide point cloud into sectors for navigation decisions"""
        front_points = []
        left_points = []
        right_points = []
        
        for x, y, z, dist in points:
            angle = math.atan2(y, x)
            
            if -math.pi/4 < angle < math.pi/4:  # Front ±45°
                front_points.append(dist)
            elif math.pi/4 <= angle <= 3*math.pi/4:  # Left 45° to 135°
                left_points.append(dist)
            elif -3*math.pi/4 <= angle < -math.pi/4:  # Right -135° to -45°
                right_points.append(dist)
        
        # Calculate clearance per sector (10th percentile for robustness)
        self.front_clear = self._get_clearance(front_points)
        self.left_clear = self._get_clearance(left_points)
        self.right_clear = self._get_clearance(right_points)

    def _get_clearance(self, distances):
        """Get clearance distance using 10th percentile"""
        if not distances:
            return self.max_range
        
        sorted_dists = sorted(distances)
        if len(sorted_dists) > 10:
            return min(self.max_range, sorted_dists[len(sorted_dists)//10])
        return min(self.max_range, sorted_dists[0])

    def publish_status(self):
        """Publish obstacle information with bilingual logging"""
        # Publish detailed obstacle info as JSON
        obstacle_info = {
            'obstacle_detected': self.obstacle_detected,
            'min_distance': round(self.min_obstacle_distance, 2),
            'front_clear': round(self.front_clear, 2),
            'left_clear': round(self.left_clear, 2),
            'right_clear': round(self.right_clear, 2),
            'is_critical': self.min_obstacle_distance < self.critical_distance
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
                    f"🚨 КРИТИЧЕСКОЕ! {self.min_obstacle_distance:.1f}m | CRITICAL! "
                    f"(Ф:{self.front_clear:.1f} Л:{self.left_clear:.1f} П:{self.right_clear:.1f})",
                    throttle_duration_sec=1.0
                )
            else:
                self.get_logger().info(
                    f"⚠️ ПРЕПЯТСТВИЕ: {self.min_obstacle_distance:.1f}m | OBSTACLE "
                    f"(F:{self.front_clear:.1f} L:{self.left_clear:.1f} R:{self.right_clear:.1f})",
                    throttle_duration_sec=2.0
                )
        else:
            self.get_logger().info(
                f"✅ СВОБОДНО | CLEAR (F:{self.front_clear:.1f} L:{self.left_clear:.1f} R:{self.right_clear:.1f})",
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
