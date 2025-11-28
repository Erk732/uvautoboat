import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Float64

class Vostok1(Node):
    def __init__(self):
        super().__init__('vostok1_node')

        # --- CONFIGURATION PARAMETERS ---
        self.declare_parameter('scan_length', 100.0)
        self.declare_parameter('scan_width', 10.0)
        self.declare_parameter('lanes', 10)
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 2.0)

        # PID Controller gains
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)

        # Obstacle avoidance parameters
        self.declare_parameter('min_safe_distance', 5.0)
        self.declare_parameter('critical_distance', 2.5)
        self.declare_parameter('obstacle_slow_factor', 0.3)

        # Get parameters
        self.scan_length = self.get_parameter('scan_length').value
        self.scan_width = self.get_parameter('scan_width').value
        self.lanes = self.get_parameter('lanes').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.obstacle_slow_factor = self.get_parameter('obstacle_slow_factor').value

        # --- STATE VARIABLES ---
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        self.previous_error = 0.0
        self.integral_error = 0.0

        self.waypoints = []
        self.current_wp_index = 0
        self.state = "INIT"  # INIT -> DRIVING -> FINISHED

        # Obstacle avoidance state
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.avoidance_mode = False

        # Statistics
        self.total_distance = 0.0
        self.start_time = None

        # --- SUBSCRIBERS ---
        self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10
        )
        self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10
        )
        self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            self.lidar_callback,
            10
        )

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # --- CONTROL LOOP (20Hz for smoother control) ---
        self.dt = 0.05  # 20 Hz
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Vostok 1 - GPS Navigator with Obstacle Avoidance")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Scan Area: {self.scan_length}m × {self.scan_width * self.lanes}m")
        self.get_logger().info(f"Lanes: {self.lanes}, Width: {self.scan_width}m")
        self.get_logger().info(f"Speed: {self.base_speed} (max: {self.max_speed})")
        self.get_logger().info(f"PID: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info(f"Obstacle Avoidance: Safe={self.min_safe_distance}m, Critical={self.critical_distance}m")
        self.get_logger().info("Waiting for GPS fix...")
        self.get_logger().info("=" * 60)

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)

        # First GPS fix - initialize mission
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Home Base: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
            self.generate_lawnmower_path()
            self.state = "DRIVING"
            self.get_logger().info("Mission Started!")

    def imu_callback(self, msg):
        # Convert quaternion to yaw (heading)
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        """Process LIDAR scan for obstacle detection"""
        ranges = np.array(msg.ranges)

        # Filter out invalid readings
        valid_indices = np.isfinite(ranges)
        if not np.any(valid_indices):
            return

        valid_ranges = ranges[valid_indices]

        # Get minimum distance
        self.min_obstacle_distance = np.min(valid_ranges)

        # Check if obstacle is within safety threshold
        self.obstacle_detected = self.min_obstacle_distance < self.min_safe_distance

        # Analyze sectors for navigation
        self.analyze_scan_sectors(ranges)

    def analyze_scan_sectors(self, ranges):
        """Divide scan into sectors to determine best direction"""
        num_ranges = len(ranges)

        # Divide into front, left, right sectors
        front_sector = ranges[int(num_ranges*0.4):int(num_ranges*0.6)]
        left_sector = ranges[int(num_ranges*0.6):]
        right_sector = ranges[:int(num_ranges*0.4)]

        # Calculate average distance in each sector
        self.front_clear = np.mean(front_sector[np.isfinite(front_sector)]) if np.any(np.isfinite(front_sector)) else float('inf')
        self.left_clear = np.mean(left_sector[np.isfinite(left_sector)]) if np.any(np.isfinite(left_sector)) else float('inf')
        self.right_clear = np.mean(right_sector[np.isfinite(right_sector)]) if np.any(np.isfinite(right_sector)) else float('inf')

    def latlon_to_meters(self, lat, lon):
        """Convert GPS coordinates to local meters (Equirectangular projection)"""
        R = 6371000.0  # Earth radius in meters

        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])

        x = d_lat * R
        y = d_lon * R * math.cos(lat0)

        # Return (East, North) in meters
        return y, x

    def generate_lawnmower_path(self):
        """Generate zigzag lawn mower pattern waypoints"""
        self.waypoints = []

        for i in range(self.lanes):
            # Alternate direction each lane
            if i % 2 == 0:
                x_end = self.scan_length
            else:
                x_end = 0.0

            y_pos = i * self.scan_width

            # Lane endpoint
            self.waypoints.append((x_end, y_pos))

            # Add transition to next lane
            if i < self.lanes - 1:
                next_y = (i + 1) * self.scan_width
                self.waypoints.append((x_end, next_y))

        total_waypoints = len(self.waypoints)
        estimated_distance = self.scan_length * self.lanes + self.scan_width * (self.lanes - 1)

        self.get_logger().info(f"Generated {total_waypoints} waypoints")
        self.get_logger().info(f"Estimated path length: {estimated_distance:.1f}m")

    def control_loop(self):
        if self.state != "DRIVING" or self.current_gps is None:
            return

        # Get current position in meters
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        # Check if mission complete
        if self.current_wp_index >= len(self.waypoints):
            self.finish_mission(curr_x, curr_y)
            return

        # Get target waypoint
        target_x, target_y = self.waypoints[self.current_wp_index]

        # Calculate distance to target
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)

        # Check if waypoint reached
        if dist < self.waypoint_tolerance:
            self.get_logger().info(
                f"Waypoint {self.current_wp_index + 1}/{len(self.waypoints)} "
                f"reached at ({curr_x:.1f}, {curr_y:.1f})"
            )
            self.current_wp_index += 1
            self.total_distance += dist
            self.integral_error = 0.0
            return

        # --- OBSTACLE AVOIDANCE LOGIC ---
        if self.min_obstacle_distance < self.critical_distance:
            # CRITICAL: Stop or reverse
            self.get_logger().warn(
                f"CRITICAL OBSTACLE at {self.min_obstacle_distance:.2f}m - Stopping/Reversing!"
            )
            self.send_thrust(-200.0, -200.0)  # Reverse slowly
            self.avoidance_mode = True
            return

        # Calculate desired heading to waypoint
        target_angle = math.atan2(dy, dx)

        # Calculate heading error
        angle_error = target_angle - self.current_yaw

        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # Obstacle avoidance: modify heading if obstacle detected
        if self.obstacle_detected:
            self.avoidance_mode = True
            # Determine which direction to turn
            if self.left_clear > self.right_clear:
                # More space on left, bias turn left
                avoidance_bias = 0.5  # Positive = turn left
                direction = "LEFT"
            else:
                # More space on right, bias turn right
                avoidance_bias = -0.5  # Negative = turn right
                direction = "RIGHT"

            # Add avoidance bias to angle error
            angle_error += avoidance_bias

            self.get_logger().info(
                f"OBSTACLE {self.min_obstacle_distance:.1f}m - Turning {direction} "
                f"(L:{self.left_clear:.1f}m R:{self.right_clear:.1f}m)",
                throttle_duration_sec=1.0
            )
        else:
            if self.avoidance_mode:
                self.get_logger().info("Path clear - Resuming waypoint navigation")
                self.avoidance_mode = False

        # PID Controller
        self.integral_error += angle_error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error))

        derivative_error = (angle_error - self.previous_error) / self.dt

        turn_power = (
            self.kp * angle_error +
            self.ki * self.integral_error +
            self.kd * derivative_error
        )

        self.previous_error = angle_error

        # Limit turn power
        turn_power = max(-800.0, min(800.0, turn_power))

        # Adaptive speed
        angle_error_deg = abs(math.degrees(angle_error))
        if angle_error_deg > 45:
            speed = self.base_speed * 0.5
        elif angle_error_deg > 20:
            speed = self.base_speed * 0.75
        else:
            speed = self.base_speed

        # Distance-based speed adjustment
        if dist < 5.0:
            speed *= 0.7

        # Obstacle-based speed adjustment
        if self.obstacle_detected:
            speed *= self.obstacle_slow_factor

        # Differential thrust
        left_thrust = speed - turn_power
        right_thrust = speed + turn_power

        # Clamp thrusts
        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))

        # Send commands
        self.send_thrust(left_thrust, right_thrust)

        # Periodic status update
        if self.current_wp_index % 4 == 0 and hasattr(self, '_last_log_time'):
            now = self.get_clock().now()
            if (now - self._last_log_time).nanoseconds / 1e9 > 2.0:
                self.log_status(curr_x, curr_y, target_x, target_y, dist, angle_error)
                self._last_log_time = now
        elif not hasattr(self, '_last_log_time'):
            self._last_log_time = self.get_clock().now()

    def log_status(self, curr_x, curr_y, target_x, target_y, dist, error):
        """Log current navigation status"""
        wp_progress = f"{self.current_wp_index + 1}/{len(self.waypoints)}"
        obs_status = f"OBS:{self.min_obstacle_distance:.1f}m" if self.obstacle_detected else "CLEAR"
        self.get_logger().info(
            f"WP {wp_progress} | "
            f"Pos: ({curr_x:.1f}, {curr_y:.1f}) | "
            f"Target: ({target_x:.1f}, {target_y:.1f}) | "
            f"Dist: {dist:.1f}m | "
            f"Error: {math.degrees(error):.1f}° | "
            f"{obs_status}"
        )

    def finish_mission(self, final_x, final_y):
        """Complete the mission and log statistics"""
        self.state = "FINISHED"
        self.stop_boat()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        elapsed_min = elapsed / 60.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE!")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Final Position: ({final_x:.1f}m, {final_y:.1f}m)")
        self.get_logger().info(f"Total Distance: {self.total_distance:.1f}m")
        self.get_logger().info(f"Mission Time: {elapsed_min:.1f} minutes")
        if elapsed > 0:
            avg_speed = self.total_distance / elapsed
            self.get_logger().info(f"Average Speed: {avg_speed:.2f} m/s")
        self.get_logger().info("=" * 60)

    def send_thrust(self, left, right):
        """Publish thrust commands to both thrusters"""
        msg_l = Float64()
        msg_r = Float64()
        msg_l.data = float(left)
        msg_r.data = float(right)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def stop_boat(self):
        """Stop all thrusters"""
        self.send_thrust(0.0, 0.0)
        self.get_logger().info("Boat stopped")

def main(args=None):
    rclpy.init(args=args)
    node = Vostok1()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission aborted by user")
    finally:
        node.stop_boat()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
