import rclpy
from rclpy.node import Node
import math

# Messages
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64

class LawnMower(Node):
    def __init__(self):
        super().__init__('lawn_mower_node')

        # --- CONFIGURATION ---
        self.scan_length = 3000.0  # Go forward what meters you want between lanes
        self.scan_width = 1000.0   # Move sideways what meters you want between lanes
        self.lanes = 10           # How many times to go back and forth
        self.speed = 500.0       # Thrust power

        # --- STATE ---
        self.start_gps = None    # Will store (lat, lon) where we started
        self.current_gps = None
        self.current_yaw = 0.0
        
        self.waypoints = []      # List of (x, y) meters relative to start
        self.current_wp_index = 0
        self.state = "INIT"      # INIT -> PLANNING -> DRIVING -> FINISHED

        # --- SUBSCRIBERS (Sensors) ---
        # 1. GPS for Position
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        # 2. IMU for Heading (Compass)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)

        # --- PUBLISHERS (Muscles) ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # --- CONTROL LOOP (10Hz) ---
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("🚜 Lawn Mower Mode: Waiting for GPS...")

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        
        # If this is the first time we see GPS, save it as "Home"
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"🏠 Home Base Set: {self.start_gps}")
            self.generate_lawnmower_path()
            self.state = "DRIVING"

    def imu_callback(self, msg):
        # Convert Quaternion to Yaw (Heading) manually
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def latlon_to_meters(self, lat, lon):
        # Convert current GPS to X,Y meters relative to Start GPS
        # Simple Equirectangular projection (Good enough for small distances)
        R = 6371000.0 # Earth radius in meters
        
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])

        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        
        # Note: In Standard ROS ENU (East-North-Up):
        # X is East (Longitude), Y is North (Latitude)
        return y, x  # Returns (East, North) in meters

    def generate_lawnmower_path(self):
        # Create a zigzag path
        # 0,0 -> 30,0 -> 30,10 -> 0,10 -> 0,20 -> ...
        self.waypoints = []
        
        for i in range(self.lanes):
            # X coordinate (Forward/Back)
            x_dist = self.scan_length if (i % 2 == 0) else 0.0
            y_dist = i * self.scan_width
            
            self.waypoints.append((x_dist, y_dist))
            
            # Add the "Lane Switch" point (move sideways)
            if i < self.lanes - 1:
                next_y = (i + 1) * self.scan_width
                self.waypoints.append((x_dist, next_y))

        self.get_logger().info(f"📋 Generated {len(self.waypoints)} waypoints for scanning.")

    def control_loop(self):
        if self.state != "DRIVING" or self.current_gps is None:
            return

        # 1. Where am I in meters?
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        
        # 2. Where is the target?
        if self.current_wp_index >= len(self.waypoints):
            self.state = "FINISHED"
            self.stop_boat()
            self.get_logger().info("🏁 Scanning Complete!")
            return

        target_x, target_y = self.waypoints[self.current_wp_index]

        # 3. Distance to Target
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)

        # Check if we reached the waypoint (within 3 meters)
        if dist < 3.0:
            self.get_logger().info(f"✅ Reached Waypoint {self.current_wp_index}")
            self.current_wp_index += 1
            return

        # 4. Heading to Target
        # In ROS ENU: 0 is East, PI/2 is North.
        target_angle = math.atan2(dy, dx)
        
        # Calculate Error
        angle_error = target_angle - self.current_yaw

        # Normalize error to -PI to +PI
        while angle_error > math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi

        # 5. Simple P-Controller
        kp = 400.0
        turn_power = angle_error * kp
        
        # Limit turn power
        turn_power = max(-800.0, min(800.0, turn_power))

        # Drive!
        left_thrust = self.speed - turn_power
        right_thrust = self.speed + turn_power
        
        # Clamp to max range
        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))

        self.send_thrust(left_thrust, right_thrust)

    def send_thrust(self, l, r):
        msg_l = Float64()
        msg_r = Float64()
        msg_l.data = float(l)
        msg_r.data = float(r)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def stop_boat(self):
        self.send_thrust(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = LawnMower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()