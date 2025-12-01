import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import Float64, String
import math
import struct
import json

class AtlantisController(Node):
    def __init__(self):
        super().__init__('atlantis_controller')

        # --- CONTROL PARAMETERS ---
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)
        
        # Obstacle & Stuck parameters (Same as before)
        self.declare_parameter('min_safe_distance', 15.0)
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('obstacle_slow_factor', 0.3)
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('stuck_threshold', 0.5)

        # Get values
        self.base_speed = self.get_parameter('base_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        # ... (Get other parameters as needed) ...
        
        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        self.waypoints = [] # Format: [(x,y), (x,y)]
        self.current_wp_index = 0
        self.state = "WAITING_FOR_PATH" 
        
        # PID & Navigation State
        self.previous_error = 0.0
        self.integral_error = 0.0
        
        # Obstacle & Stuck State (simplified for brevity, keep your original logic here)
        self.min_obstacle_distance = float('inf')
        self.obstacle_detected = False
        
        # --- SUBSCRIBERS ---
        self.create_subscription(Path, '/atlantis/path', self.path_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # --- TIMER ---
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Atlantis Controller Waiting for Path...")

    def path_callback(self, msg):
        """Receive path from Planner"""
        # If we already have a path and are driving, we might want to check if it changed
        # For now, let's say if we get a new path, we restart following it
        new_waypoints = []
        for pose in msg.poses:
            new_waypoints.append((pose.pose.position.x, pose.pose.position.y))
            
        if new_waypoints != self.waypoints:
            self.waypoints = new_waypoints
            self.current_wp_index = 0
            self.get_logger().info(f"Received new plan with {len(self.waypoints)} waypoints")
            if self.start_gps is not None:
                self.state = "DRIVING"

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"Home Position Set: {self.start_gps}")
            if len(self.waypoints) > 0:
                self.state = "DRIVING"

    def imu_callback(self, msg):
        # (Keep your existing quaternion to yaw conversion)
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        # (Keep your existing point cloud processing logic here)
        pass 

    def latlon_to_meters(self, lat, lon):
        # (Keep your existing conversion)
        R = 6371000.0
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])
        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return y, x # East, North

    def control_loop(self):
        if self.state != "DRIVING" or self.current_gps is None or not self.waypoints:
            return

        # 1. Get Position
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        
        # 2. Check Waypoint Reached
        target_x, target_y = self.waypoints[self.current_wp_index]
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)
        
        if dist < self.waypoint_tolerance:
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.waypoints):
                self.state = "FINISHED"
                self.stop_boat()
                self.get_logger().info("Mission Finished")
                return
            else:
                self.get_logger().info(f"Waypoint {self.current_wp_index} Reached")
                # Update target
                target_x, target_y = self.waypoints[self.current_wp_index]
                dx = target_x - curr_x
                dy = target_y - curr_y

        # 3. Calculate Heading & PID (Keep your exact PID logic here)
        dt = 0.05  # matches your timer period
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_yaw

        # Normalize angle
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi
        
        # Full PID (KP/KD/KI)
        self.integral_error += angle_error * dt
        self.integral_error += angle_error * dt
        # Anti-windup: clamp integral term
        self.integral_error = max(-100.0, min(100.0, self.integral_error))
        derivative_error = (angle_error - self.previous_error) / dt
        turn_power = self.kp * angle_error + self.ki * self.integral_error + self.kd * derivative_error
        self.previous_error = angle_error
        # Clamp the output
        turn_power = max(-800.0, min(800.0, turn_power))
        speed = self.base_speed
        
        # Send Thrust
        self.pub_left.publish(Float64(data=speed - turn_power))
        self.pub_right.publish(Float64(data=speed + turn_power))

    def stop_boat(self):
        self.pub_left.publish(Float64(data=0.0))
        self.pub_right.publish(Float64(data=0.0))

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()