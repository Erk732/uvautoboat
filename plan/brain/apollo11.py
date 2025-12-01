#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

# Messages
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Float64

class Apollo11(Node):
    def __init__(self):
        super().__init__('apollo11_node')

        # --- CONFIGURATION ---
        self.scan_length = 100.0   
        self.scan_width = 100.0    
        self.lanes = 8            
        self.speed = 1000.0        # Normal speed
        
        # --- STUCK DETECTION SETTINGS ---
        self.check_interval = 5.0 # Check for stuck every 5 seconds
        self.min_move_dist = 1.0  # Must move at least 1 meter every 5 sec
        
        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        
        self.waypoints = []
        self.current_wp_index = 0
        
        # State Machine: "INIT", "DRIVING", "REVERSING", "TURNING", "AVOIDING", "FINISHED"
        self.state = "INIT" 
        
        # Variables for Stuck Logic
        self.last_check_time = time.time()
        self.last_check_pos = None
        self.recovery_start_time = 0.0

        # --- OBSTACLE AVOIDANCE SETTINGS ---
        self.avoidance_distance = 12.0  # meters
        self.avoid_duration = 1.5       # seconds
        self.obstacle_state = 0          # -1 = avoid right, +1 = avoid left, 0 = clear
        self.avoid_start_time = 0.0

        # --- SENSORS ---
        self.sub_gps = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.sub_lidar = self.create_subscription(LaserScan, '/wamv/sensors/lidars/lidar_wamv_sensor/scan', self.lidar_callback, 10)

        # --- ACTUATORS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Apollo 11: Systems Initialized with Anti-Stuck Technology and Obstacle Avoidance.")

    # --- GPS callback ---
    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"Launch Position has been established: {self.start_gps}")
            self.generate_mission_waypoints()
            self.state = "DRIVING"
            
            # Initialize stuck checker
            self.last_check_pos = self.latlon_to_meters(msg.latitude, msg.longitude)
            self.last_check_time = time.time()

    # --- IMU callback ---
    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # --- LIDAR callback (simple obstacle avoidance) ---
    def lidar_callback(self, msg: LaserScan):
        num_points = len(msg.ranges)
        front_indices = range(num_points // 2 - num_points // 8, num_points // 2 + num_points // 8)
        front_ranges = [msg.ranges[i] for i in front_indices if not math.isinf(msg.ranges[i])]

        if not front_ranges:
            self.obstacle_state = 0
            return

        min_front = min(front_ranges)
        if min_front < self.avoidance_distance:
            left_indices  = range(num_points // 2, num_points // 2 + num_points // 8)
            right_indices = range(num_points // 2 - num_points // 8, num_points // 2)

            left_avg  = sum([msg.ranges[i] for i in left_indices if not math.isinf(msg.ranges[i])]) / max(1, len(left_indices))
            right_avg = sum([msg.ranges[i] for i in right_indices if not math.isinf(msg.ranges[i])]) / max(1, len(right_indices))

            self.obstacle_state = 1 if left_avg > right_avg else -1
            self.avoid_start_time = time.time()
            self.state = "AVOIDING"
        else:
            self.obstacle_state = 0

    # --- Utility: convert lat/lon to meters ---
    def latlon_to_meters(self, lat, lon):
        if self.start_gps is None: return 0.0, 0.0
        R = 6371000.0 
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])
        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return y, x 

    # --- Generate waypoints (lawnmower pattern) ---
    def generate_mission_waypoints(self):
        self.waypoints = []
        for i in range(self.lanes):
            x_dist = self.scan_length if (i % 2 == 0) else 0.0
            y_dist = i * self.scan_width
            self.waypoints.append((x_dist, y_dist))
            if i < self.lanes - 1:
                next_y = (i + 1) * self.scan_width
                self.waypoints.append((x_dist, next_y))
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints.")

    # --- Stuck detection ---
    def check_if_stuck(self, curr_x, curr_y):
        now = time.time()
        if (now - self.last_check_time) > self.check_interval:
            last_x, last_y = self.last_check_pos
            dist_moved = math.hypot(curr_x - last_x, curr_y - last_y)
            if dist_moved < self.min_move_dist:
                self.get_logger().warn(f"WAMV IS STUCK! Moved only {dist_moved:.2f}m in 5s.") # maybe we could make the 5s prin prompt into paramater?
                self.get_logger().warn("Initiating Recovery Maneuver...")
                self.state = "REVERSING"
                self.recovery_start_time = time.time()
            self.last_check_pos = (curr_x, curr_y)
            self.last_check_time = now

    # --- Main control loop ---
    def control_loop(self):
        if self.current_gps is None or self.state == "INIT": return
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        # --- Recovery logic ---
        if self.state == "REVERSING":
            self.send_thrust(-1000.0, -1000.0)
            if (time.time() - self.recovery_start_time) > 10.0:
                self.state = "TURNING"
                self.recovery_start_time = time.time()
            return

        if self.state == "TURNING":
            self.send_thrust(800.0, -800.0)
            if (time.time() - self.recovery_start_time) > 2.0:
                self.get_logger().info("Recovery Complete. Resuming Mission.")
                self.state = "DRIVING"
                self.last_check_time = time.time()
                self.last_check_pos = (curr_x, curr_y)
            return

        # --- Obstacle Avoidance --- CHANGE IT
        if self.state == "AVOIDING":
            turn_power = 600.0
            if self.obstacle_state == 1:   # more space on left then turn left
                self.send_thrust(-turn_power, turn_power)
            elif self.obstacle_state == -1:  # more space on right then turn right
                self.send_thrust(turn_power, -turn_power)
            else:
                self.send_thrust(0.0, 0.0)

            if (time.time() - self.avoid_start_time) > self.avoid_duration:
                self.state = "DRIVING"
            return

        # --- Normal Driving ---
        if self.state == "DRIVING":
            self.check_if_stuck(curr_x, curr_y)

            if self.current_wp_index >= len(self.waypoints):
                self.stop_boat()
                self.get_logger().info("Mission Complete.")
                self.state = "FINISHED"
                return

            target_x, target_y = self.waypoints[self.current_wp_index]
            dx = target_x - curr_x
            dy = target_y - curr_y
            dist = math.hypot(dx, dy)

            if dist < 4.0:
                self.get_logger().info(f"Waypoint {self.current_wp_index + 1} Reached")
                self.current_wp_index += 1
                return

            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_yaw
            while angle_error > math.pi: angle_error -= 2 * math.pi
            while angle_error < -math.pi: angle_error += 2 * math.pi

            kp = 300.0
            turn_power = max(-800.0, min(800.0, angle_error * kp))

            left = self.speed - turn_power
            right = self.speed + turn_power
            left = max(-1000.0, min(1000.0, left))
            right = max(-1000.0, min(1000.0, right))

            self.send_thrust(left, right)

    # --- Thruster control ---
    def send_thrust(self, l, r):
        msg_l = Float64()
        msg_r = Float64()
        msg_l.data = float(l)
        msg_r.data = float(r)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def stop_boat(self):
        self.send_thrust(0.0, 0.0)

# --- Main function ---
def main(args=None):
    rclpy.init(args=args)
    node = Apollo11()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 