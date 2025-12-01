#!/usr/bin/env python3
# PLEASE READ COMMENTS
# atlanis_planner.py for only planning with dynamic reconfigure feature
# WE CAN USE ANY NODE WE CAN THIS IS ONLY A PSEUDO CODE FOR THE NEW PLANNER WITH DYNAMIC PARAMETERS
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import math
import struct
import json
import time

# Messages
from sensor_msgs.msg import NavSatFix, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import String

class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')

        # --- 1. DECLARE PARAMETERS ---
        self.declare_parameter('scan_length', 40.0)
        self.declare_parameter('scan_width', 20.0)
        self.declare_parameter('lanes', 6)
        
        self.scan_length = self.get_parameter('scan_length').value
        self.scan_width = self.get_parameter('scan_width').value
        self.lanes = self.get_parameter('lanes').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.waypoints = []
        self.current_wp_index = 0
        
        # Logic
        self.last_pos = None
        self.last_check_time = time.time()
        self.is_stuck = False
        
        # VISION STATE
        self.obstacle_detected = False
        self.min_obstacle_distance = 50.0
        self.front_clear = 50.0  # Added Front Sector
        self.left_clear = 50.0
        self.right_clear = 50.0
        
        # --- PUBS/SUBS ---
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

        self.pub_target = self.create_publisher(Point, '/atlantis/target', 10)
        self.pub_status = self.create_publisher(String, '/atlantis/planner_status', 10)

        self.create_timer(0.1, self.planning_loop)
        self.get_logger().info("ATLANTIS PLANNER: Online (Vision + 180° Limit)")

    def parameter_callback(self, params):
        regenerate = False
        for param in params:
            if param.name == 'scan_length':
                self.scan_length = param.value
                regenerate = True
            elif param.name == 'scan_width':
                self.scan_width = param.value
                regenerate = True
            elif param.name == 'lanes':
                self.lanes = param.value
                regenerate = True

        if regenerate and self.start_gps:
            self.generate_waypoints()
            self.get_logger().warn("MISSION RECALCULATED!")

        return SetParametersResult(successful=True)

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.generate_waypoints()
            self.get_logger().info("Home Base Set.")

    def generate_waypoints(self):
        self.waypoints = []
        for i in range(self.lanes):
            x = self.scan_length if (i % 2 == 0) else 0.0
            y = i * self.scan_width
            self.waypoints.append((x, y))
            if i < self.lanes - 1:
                self.waypoints.append((x, (i + 1) * self.scan_width))

    def latlon_to_meters(self, lat, lon):
        if not self.start_gps: return 0,0
        R = 6371000.0
        dx = (math.radians(lon - self.start_gps[1]) * R * math.cos(math.radians(self.start_gps[0])))
        dy = (math.radians(lat - self.start_gps[0]) * R)
        return dx, dy 

    # =========================================================
    #  VISION LOGIC (With Limitations)
    # =========================================================
    def lidar_callback(self, msg):
        points = []
        point_step = msg.point_step
        data = msg.data
        
        # Process every 10th point
        for i in range(0, len(data) - point_step, point_step * 10):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                if math.isnan(x) or math.isinf(x): continue
                if z < -0.2 or z > 3.0: continue
                dist = math.sqrt(x*x + y*y)
                if dist < 1.0 or dist > 100.0: continue
                
                # --- LIMITATION 1: NO REAR VISION ---
                # We only want to see things in front of the boat (x > 0.5)
                # This effectively gives us 180 degree vision, preventing 360 issues.
                if x < 0.5: continue 
                
                points.append((x, y, z, dist))
            except: continue
        
        if not points:
            self.obstacle_detected = False
            self.min_obstacle_distance = 50.0
            self.front_clear = 50.0
            self.left_clear = 50.0
            self.right_clear = 50.0
            return
        
        distances = [p[3] for p in points]
        self.min_obstacle_distance = min(distances)
        
        # Trigger avoidance if anything is closer than 15m
        self.obstacle_detected = self.min_obstacle_distance < 15.0

        self.analyze_sectors(points)

    def analyze_sectors(self, points):
        front_points = []
        left_points = []
        right_points = []
        
        for x, y, z, dist in points:
            angle = math.atan2(y, x) 
            
            # --- LIMITATION 2: SECTOR DEFINITIONS ---
            
            # Front Sector: +/- 45 degrees
            if -0.78 < angle < 0.78:
                front_points.append(dist)
            
            # Left Sector: 45 to 135 degrees
            elif 0.78 <= angle <= 2.35: 
                left_points.append(dist)
                
            # Right Sector: -135 to -45 degrees
            elif -2.35 <= angle <= -0.78: 
                right_points.append(dist)
        
        max_range = 50.0
        def get_min(pts):
            if not pts: return max_range
            pts.sort()
            return pts[len(pts)//10] if len(pts)>10 else pts[0]

        self.front_clear = get_min(front_points)
        self.left_clear = get_min(left_points)
        self.right_clear = get_min(right_points)
    # =========================================================

    def planning_loop(self):
        if not self.current_gps or not self.start_gps: return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        # Stuck Check
        if time.time() - self.last_check_time > 5.0:
            if self.last_pos:
                dist = math.hypot(curr_x - self.last_pos[0], curr_y - self.last_pos[1])
                self.is_stuck = (dist < 1.0)
            self.last_pos = (curr_x, curr_y)
            self.last_check_time = time.time()

        cmd = Point()
        
        if self.is_stuck:
            cmd.z = -1.0 # Reverse
            self.pub_status.publish(String(data="STUCK! Reversing"))
        
        elif self.obstacle_detected:
            # --- EVASION TECHNIQUE ---
            
            # 1. If blocked directly in front, turn HARD
            if self.front_clear < 10.0:
                 # If very close, stop forward motion and turn
                if self.left_clear > self.right_clear:
                    cmd.x = curr_x - 5.0
                    cmd.y = curr_y + 10.0 # Left
                    direction = "HARD LEFT"
                else:
                    cmd.x = curr_x + 5.0
                    cmd.y = curr_y - 10.0 # Right
                    direction = "HARD RIGHT"
            
            # 2. If just generally close, gently turn
            else:
                if self.left_clear > self.right_clear:
                    cmd.x = curr_x - 5.0 
                    cmd.y = curr_y + 10.0
                    direction = "SOFT LEFT"
                else:
                    cmd.x = curr_x + 5.0 
                    cmd.y = curr_y - 10.0
                    direction = "SOFT RIGHT"
                
            cmd.z = 1.0 
            self.pub_status.publish(String(data=f"OBSTACLE! {direction}"))

        else:
            if self.current_wp_index >= len(self.waypoints):
                cmd.z = 0.0 # Stop
                self.pub_status.publish(String(data="MISSION COMPLETE"))
            else:
                target = self.waypoints[self.current_wp_index]
                dist = math.hypot(target[0] - curr_x, target[1] - curr_y)
                
                if dist < 4.0:
                    self.current_wp_index += 1
                
                cmd.x = float(target[0])
                cmd.y = float(target[1])
                cmd.z = 1.0 
                self.pub_status.publish(String(data=f"To Waypoint {self.current_wp_index+1}"))

        self.pub_target.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()