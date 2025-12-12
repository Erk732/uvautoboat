#!/usr/bin/env python3
"""
Waypoint Visualizer for RViz - Modular Architecture

Subscribes to Sputnik planner's mission status and waypoints topics,
and publishes visualization markers for waypoints, current target,
boat path, and no-go zones.

Run this alongside the modular Sputnik planner to see waypoints in RViz.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import NavSatFix
import json
import math


class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_pub = self.create_publisher(Marker, '/boat_path', 10)
        
        # State
        self.waypoints = []
        self.current_wp_index = 0
        self.start_gps = None
        self.current_local_pos = None
        self.path_points = []
        self.no_go_zones = []
        
        # Reference point for GPS->local conversion
        self.ref_lat = None
        self.ref_lon = None
        
        # Subscribe to modular architecture topics only
        self.create_subscription(
            String,
            '/planning/mission_status',
            self.mission_status_callback_modular,
            10
        )

        self.create_subscription(
            String,
            '/planning/waypoints',
            self.waypoints_callback_modular,
            10
        )
        
        # Subscribe to GPS for current position and reference
        self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10
        )
        
        # Timer to publish markers at 2Hz
        self.create_timer(0.5, self.publish_markers)
        
        self.get_logger().info("🎯 Waypoint Visualizer Started!")
        self.get_logger().info("   View markers in RViz: Add MarkerArray → /waypoint_markers")
        
    def gps_callback(self, msg):
        """Track GPS for path and reference"""
        if self.ref_lat is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.get_logger().info(f"📍 Reference set: {self.ref_lat:.6f}, {self.ref_lon:.6f}")
            
        # Convert to local coordinates
        x, y = self.gps_to_local(msg.latitude, msg.longitude)
        self.current_local_pos = (x, y)
        
        # Add to path (limit to last 500 points)
        self.path_points.append((x, y))
        if len(self.path_points) > 500:
            self.path_points.pop(0)
            
    def mission_status_callback_modular(self, msg):
        """Parse mission status from Sputnik (modular mode)"""
        try:
            data = json.loads(msg.data)
            if 'current_waypoint' in data:
                self.current_wp_index = data['current_waypoint'] - 1  # Convert 1-indexed to 0-indexed
            if 'total_waypoints' in data:
                self.total_waypoints = data['total_waypoints']
        except json.JSONDecodeError:
            pass

    def waypoints_callback_modular(self, msg):
        """Receive waypoint list from Sputnik planner - handles both formats"""
        try:
            data = json.loads(msg.data)
            if 'waypoints' in data:
                # Handle both formats: [(x, y), ...] or [{'x': x, 'y': y}, ...]
                raw_waypoints = data['waypoints']
                if raw_waypoints and len(raw_waypoints) > 0:
                    if isinstance(raw_waypoints[0], (list, tuple)):
                        # Tuple/list format
                        self.waypoints = [(wp[0], wp[1]) for wp in raw_waypoints]
                    elif isinstance(raw_waypoints[0], dict):
                        # Dict format
                        self.waypoints = [(wp['x'], wp['y']) for wp in raw_waypoints]
                    self.get_logger().info(f"📍 Received {len(self.waypoints)} waypoints")
            if 'no_go_zones' in data:
                self.no_go_zones = data['no_go_zones']
        except (json.JSONDecodeError, KeyError, IndexError) as e:
            self.get_logger().warn(f"Waypoint parse error: {e}")
                
    def gps_to_local(self, lat, lon):
        """Convert GPS to local ENU coordinates"""
        if self.ref_lat is None:
            return 0.0, 0.0
            
        R = 6371000.0
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)
        
        dlat = lat_rad - ref_lat_rad
        dlon = lon_rad - ref_lon_rad
        
        x = R * dlon * math.cos(ref_lat_rad)
        y = R * dlat
        
        return x, y
        
    def publish_markers(self):
        """Publish all visualization markers"""
        markers = MarkerArray()
        
        # 1. Waypoint spheres
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = 3.0
            marker.scale.y = 3.0
            marker.scale.z = 3.0
            
            # Color: green=done, yellow=current, red=future
            if i < self.current_wp_index:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)  # Green
            elif i == self.current_wp_index:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
                marker.scale.x = 5.0  # Bigger
                marker.scale.y = 5.0
                marker.scale.z = 5.0
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)  # Red
                
            markers.markers.append(marker)
            
            # 2. Waypoint number text
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = wp[0]
            text_marker.pose.position.y = wp[1]
            text_marker.pose.position.z = 5.0
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 3.0
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = str(i)
            
            markers.markers.append(text_marker)
            
        # 3. Planned path line
        if len(self.waypoints) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "planned_path"
            path_marker.id = 2000
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            
            path_marker.scale.x = 0.5
            path_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            path_marker.pose.orientation.w = 1.0
            
            for wp in self.waypoints:
                p = Point()
                p.x = wp[0]
                p.y = wp[1]
                p.z = 0.5
                path_marker.points.append(p)
                
            markers.markers.append(path_marker)
            
        # 4. No-go zones (red cylinders)
        for i, zone in enumerate(self.no_go_zones):
            zone_marker = Marker()
            zone_marker.header.frame_id = "world"
            zone_marker.header.stamp = self.get_clock().now().to_msg()
            zone_marker.ns = "no_go_zones"
            zone_marker.id = 3000 + i
            zone_marker.type = Marker.CYLINDER
            zone_marker.action = Marker.ADD
            
            zone_marker.pose.position.x = zone[0]
            zone_marker.pose.position.y = zone[1]
            zone_marker.pose.position.z = 0.0
            zone_marker.pose.orientation.w = 1.0
            
            radius = zone[2] if len(zone) > 2 else 5.0
            zone_marker.scale.x = radius * 2
            zone_marker.scale.y = radius * 2
            zone_marker.scale.z = 0.5
            
            zone_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
            
            markers.markers.append(zone_marker)
            
        # 5. Actual traveled path
        if len(self.path_points) > 1:
            actual_path = Marker()
            actual_path.header.frame_id = "world"
            actual_path.header.stamp = self.get_clock().now().to_msg()
            actual_path.ns = "actual_path"
            actual_path.id = 4000
            actual_path.type = Marker.LINE_STRIP
            actual_path.action = Marker.ADD
            
            actual_path.scale.x = 0.3
            actual_path.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            actual_path.pose.orientation.w = 1.0
            
            for pt in self.path_points:
                p = Point()
                p.x = pt[0]
                p.y = pt[1]
                p.z = 0.3
                actual_path.points.append(p)
                
            markers.markers.append(actual_path)
            
        # 6. Current position arrow
        if self.current_local_pos:
            pos_marker = Marker()
            pos_marker.header.frame_id = "world"
            pos_marker.header.stamp = self.get_clock().now().to_msg()
            pos_marker.ns = "current_position"
            pos_marker.id = 5000
            pos_marker.type = Marker.ARROW
            pos_marker.action = Marker.ADD
            
            pos_marker.pose.position.x = self.current_local_pos[0]
            pos_marker.pose.position.y = self.current_local_pos[1]
            pos_marker.pose.position.z = 2.0
            pos_marker.pose.orientation.w = 1.0
            
            pos_marker.scale.x = 5.0
            pos_marker.scale.y = 1.0
            pos_marker.scale.z = 1.0
            
            pos_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            
            markers.markers.append(pos_marker)
            
        # Publish
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
