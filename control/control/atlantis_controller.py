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
        
        # Obstacle parameters
        self.declare_parameter('min_safe_distance', 15.0)
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('obstacle_slow_factor', 0.3)
        self.declare_parameter('hysteresis_distance', 2.0)

        # Get values
        self.base_speed = self.get_parameter('base_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        self.waypoints = [] 
        self.current_wp_index = 0
        self.state = "WAITING_FOR_PATH" 
        
        # PID State
        self.previous_error = 0.0
        self.integral_error = 0.0
        
        # Obstacle State
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.obstacle_detected = False
        
        # --- SUBSCRIBERS ---
        self.create_subscription(Path, '/atlantis/path', self.path_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, rclpy.qos.qos_profile_sensor_data)

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # --- TIMER ---
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Atlantis Controller Ready - Waiting for Path...")

    def path_callback(self, msg):
        """Receive path from Planner"""
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
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        """Process 3D LIDAR point cloud for obstacle detection"""
        points = []
        point_step = msg.point_step
        data = msg.data
        
        # Sample every 10th point to save CPU
        for i in range(0, len(data) - point_step, point_step * 10):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                
                if math.isnan(x) or math.isinf(x): continue
                if z < -0.2 or z > 3.0: continue # Filter height
                
                dist = math.sqrt(x*x + y*y)
                if dist < 1.0 or dist > 100.0: continue
                if x < 0.5: continue # Only forward points
                
                points.append((x, y, z, dist))
            except struct.error:
                continue

        if not points:
            self.min_obstacle_distance = 50.0
            return

        distances = [p[3] for p in points]
        self.min_obstacle_distance = min(distances)
        
        # Analyze sectors
        self.analyze_scan_sectors_3d(points)

        # Hysteresis Logic
        min_safe = self.get_parameter('min_safe_distance').value
        if self.obstacle_detected:
            exit_threshold = min_safe + self.get_parameter('hysteresis_distance').value
            self.obstacle_detected = self.min_obstacle_distance < exit_threshold
        else:
            self.obstacle_detected = self.min_obstacle_distance < min_safe

    def analyze_scan_sectors_3d(self, points):
        """Divide 3D point cloud into sectors"""
        front_points = []
        left_points = []
        right_points = []
        
        for x, y, z, dist in points:
            angle = math.atan2(y, x)
            if -math.pi/4 < angle < math.pi/4:
                front_points.append(dist)
            elif math.pi/4 <= angle <= 3*math.pi/4:
                left_points.append(dist)
            elif -3*math.pi/4 <= angle < -math.pi/4:
                right_points.append(dist)
        
        max_range = 100.0
        self.front_clear = min(front_points) if front_points else max_range
        self.left_clear = min(left_points) if left_points else max_range
        self.right_clear = min(right_points) if right_points else max_range

    def latlon_to_meters(self, lat, lon):
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

        # 1. Get Position first
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        
        # 2. Calculate Distance to Waypoint
        target_x, target_y = self.waypoints[self.current_wp_index]
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)
        
        # 3. Check Waypoint Reached
        if dist < self.waypoint_tolerance:
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.waypoints):
                self.state = "FINISHED"
                self.stop_boat()
                self.get_logger().info("Mission Finished")
                return
            else:
                self.get_logger().info(f"Waypoint {self.current_wp_index} Reached")
                # Update target immediately
                target_x, target_y = self.waypoints[self.current_wp_index]
                dx = target_x - curr_x
                dy = target_y - curr_y

        # 4. Determine Target Heading (Obstacle vs Path)
        critical_dist = self.get_parameter('critical_distance').value
        
        if self.min_obstacle_distance < critical_dist:
            # CRITICAL: Reverse
            self.get_logger().warn(f"CRITICAL OBSTACLE {self.min_obstacle_distance:.1f}m! Reversing...")
            self.pub_left.publish(Float64(data=-500.0))
            self.pub_right.publish(Float64(data=-500.0))
            return

        target_angle = 0.0
        speed_multiplier = 1.0

        if self.obstacle_detected:
            # Obstacle Mode: 90 degree turn away from obstacle
            speed_multiplier = self.get_parameter('obstacle_slow_factor').value
            if self.left_clear > self.right_clear:
                target_angle = self.current_yaw + 1.57 # Turn Left
            else:
                target_angle = self.current_yaw - 1.57 # Turn Right
            self.get_logger().info(f"Avoiding Obstacle! Dist: {self.min_obstacle_distance:.1f}m")
        else:
            # Normal Mode: Point to waypoint
            target_angle = math.atan2(dy, dx)

        # 5. PID Control
        dt = 0.05
        angle_error = target_angle - self.current_yaw

        # Normalize angle to [-pi, pi]
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi
        
        self.integral_error += angle_error * dt
        self.integral_error = max(-1.0, min(1.0, self.integral_error)) # Tighter clamp
        
        derivative_error = (angle_error - self.previous_error) / dt
        
        turn_power = (self.kp * angle_error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
        self.previous_error = angle_error
        
        # Clamp turn power
        turn_power = max(-800.0, min(800.0, turn_power))
        
        # Calculate final motor speeds
        speed = self.base_speed * speed_multiplier
        
        # Differential thrust mixing
        left_thrust = speed - turn_power
        right_thrust = speed + turn_power
        
        # Clamp final output
        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))

        self.pub_left.publish(Float64(data=left_thrust))
        self.pub_right.publish(Float64(data=right_thrust))

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