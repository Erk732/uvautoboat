#!/usr/bin/env python3
# PLEASE READ COMMENTS
# atlanis_planner.py for only planning with dynamic reconfigure feature
# WE CAN USE ANY NODE WE CAN THIS IS ONLY A PSEUDO CODE FOR THE NEW PLANNER WITH DYNAMIC PARAMETERS
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult # FOR DYNAMIC PARAMETERS
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
        
        # --- 2. LOAD INITIAL VALUES ---
        self.scan_length = self.get_parameter('scan_length').value
        self.scan_width = self.get_parameter('scan_width').value
        self.lanes = self.get_parameter('lanes').value

        # --- 3. ENABLE DYNAMIC UPDATES --- # I am not sure did I do it right or no? 
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
        self.obstacle_mode = False
        
        # --- PUBLISHERS/SUBSCRIBERS ---
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

        self.pub_target = self.create_publisher(Point, '/atlantis/target', 10)
        self.pub_status = self.create_publisher(String, '/atlantis/planner_status', 10)

        self.create_timer(0.1, self.planning_loop)
        self.get_logger().info("ATLANTIS PLANNER: Online (with Dynamic Reconfigure)")

    def parameter_callback(self, params):
        # This function runs AUTOMATICALLY when you use 'ros2 param set'
        regenerate = False
        
        for param in params:
            if param.name == 'scan_length':
                self.scan_length = param.value
                regenerate = True
                self.get_logger().info(f"Updated Scan Length: {self.scan_length}")
            elif param.name == 'scan_width':
                self.scan_width = param.value
                regenerate = True
                self.get_logger().info(f"Updated Scan Width: {self.scan_width}")
            elif param.name == 'lanes':
                self.lanes = param.value
                regenerate = True
                self.get_logger().info(f"Updated Lanes: {self.lanes}")

        if regenerate and self.start_gps:
            self.generate_waypoints()
            self.get_logger().warn("MISSION ROUTE RECALCULATED!")
            # Optional: Reset progress if parameters change drastically
            # self.current_wp_index = 0 

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

    def lidar_callback(self, msg):
        points = []
        point_step = msg.point_step
        for i in range(0, len(msg.data) - point_step, point_step * 10):
            try:
                x, _, _, _ = struct.unpack_from('ffff', msg.data, i)
                if 0.5 < x < 15.0: 
                    self.obstacle_mode = True
                    return
            except: pass
        self.obstacle_mode = False

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
        elif self.obstacle_mode:
            cmd.x = curr_x + 5.0 
            cmd.y = curr_y - 10.0 
            cmd.z = 1.0 
            self.pub_status.publish(String(data="OBSTACLE! Evading"))
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