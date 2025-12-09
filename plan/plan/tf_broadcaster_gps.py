#!/usr/bin/env python3
"""
TF Broadcaster using GPS data (works when ground_truth_odometry is unavailable)

This is a fallback TF broadcaster that converts GPS coordinates (lat/lon) 
to local ENU (East-North-Up) coordinates relative to a reference point.

The ground_truth_odometry topic sometimes has no publishers in VRX.
This version uses GPS + IMU which are always available.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix, Imu
from tf2_ros import TransformBroadcaster
import math


class TFBroadcasterGPS(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_gps')
        
        self.br = TransformBroadcaster(self)
        
        # Reference point (first GPS reading will set this)
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        
        # Current orientation from IMU
        self.current_orientation = None
        
        # Subscribe to GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.handle_gps,
            10
        )
        
        # Subscribe to IMU for orientation
        self.imu_sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.handle_imu,
            10
        )
        
        self.get_logger().info("📢 TF Broadcaster (GPS Mode) Started!")
        self.get_logger().info("   Using GPS + IMU instead of ground_truth_odometry")
        
    def handle_imu(self, msg):
        """Store the latest orientation from IMU"""
        self.current_orientation = msg.orientation
        
    def handle_gps(self, msg):
        """Convert GPS to local coordinates and broadcast TF"""
        
        # Set reference point on first message
        if self.ref_lat is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_alt = msg.altitude
            self.get_logger().info(f"📍 Reference point set: lat={self.ref_lat:.6f}, lon={self.ref_lon:.6f}")
            return
            
        # Convert lat/lon to local ENU coordinates
        x, y, z = self.gps_to_enu(msg.latitude, msg.longitude, msg.altitude)
        
        # Create transform
        t = TransformStamped()
        # Use the message timestamp (simulation time) to match Gazebo TF timestamps!
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'wamv/wamv/base_link'
        
        # Position from GPS (ENU coordinates)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Orientation from IMU (if available)
        if self.current_orientation:
            t.transform.rotation = self.current_orientation
        else:
            # Default orientation (facing east)
            t.transform.rotation.w = 1.0
            
        # Broadcast
        self.br.sendTransform(t)
        
    def gps_to_enu(self, lat, lon, alt):
        """
        Convert GPS coordinates to local ENU (East-North-Up) frame
        relative to reference point.
        
        Uses simple equirectangular approximation (good for small areas)
        """
        # Earth radius in meters
        R = 6371000.0
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)
        
        # Calculate differences
        dlat = lat_rad - ref_lat_rad
        dlon = lon_rad - ref_lon_rad
        
        # Convert to meters (equirectangular approximation)
        # East = x, North = y
        x = R * dlon * math.cos(ref_lat_rad)  # East
        y = R * dlat                           # North
        z = alt - self.ref_alt                 # Up
        
        return x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterGPS()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
