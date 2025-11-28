import rclpy
from rclpy.node import Node
import math
import numpy as np
import struct

from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import Float64

class Vostok1(Node):
    def __init__(self):
        super().__init__('vostok1_node')

        # --- CONFIGURATION PARAMETERS ---
        self.declare_parameter('scan_length', 30.0)
        self.declare_parameter('scan_width', 100.0)
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
        self.declare_parameter('hysteresis_distance', 1.0)  # Exit threshold offset
        self.declare_parameter('reverse_timeout', 3.0)  # Max seconds to reverse

        # Stuck detection parameters
        self.declare_parameter('stuck_timeout', 5.0)  # Seconds without movement to detect stuck
        self.declare_parameter('stuck_threshold', 0.5)  # Minimum movement in meters to not be stuck

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
        self.hysteresis_distance = self.get_parameter('hysteresis_distance').value
        self.reverse_timeout = self.get_parameter('reverse_timeout').value

        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value

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
        self.reverse_start_time = None

        # Stuck detection state
        self.last_position = None
        self.stuck_check_time = None
        self.is_stuck = False
        self.escape_mode = False
        self.escape_start_time = None
        self.consecutive_stuck_count = 0
        self.last_stuck_waypoint = None

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
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data
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
        """Process 3D LIDAR point cloud for obstacle detection"""
        # Extract points from PointCloud2 using proper iteration
        points = []
        
        # PointCloud2 structure: each point has x, y, z (and possibly intensity)
        # Assuming xyz fields are first 12 bytes (3 floats * 4 bytes each)
        point_step = msg.point_step
        row_step = msg.row_step
        data = msg.data
        
        # Process points - sample every 5th point for better coverage
        for i in range(0, len(data) - point_step, point_step * 5):
            try:
                # Extract x, y, z from point cloud data
                x, y, z = struct.unpack_from('fff', data, i)
                
                # Skip invalid points
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    continue
                if math.isinf(x) or math.isinf(y) or math.isinf(z):
                    continue
                
                # Filter by height - obstacles are between -0.2m (water surface) and 3.0m
                if z < -0.2 or z > 3.0:
                    continue
                
                # Calculate horizontal distance
                dist = math.sqrt(x*x + y*y)
                
                # Focus on relevant range: ignore boat itself and very far objects
                if dist < 0.3 or dist > 50.0:
                    continue
                
                # Only consider points in front hemisphere (x > -2.0)
                if x > -2.0:
                    points.append((x, y, z, dist))
                    
            except struct.error:
                continue
            except Exception:
                continue
        
        if not points:
            # No valid points - assume clear but be cautious
            self.min_obstacle_distance = 30.0
            if not self.obstacle_detected:  # Don't immediately clear if we were detecting
                self.front_clear = 30.0
                self.left_clear = 30.0
                self.right_clear = 30.0
            return
        
        # Get minimum distance from all valid points
        distances = [p[3] for p in points]
        self.min_obstacle_distance = min(distances)

        # Hysteresis for obstacle detection (prevents flickering)
        if self.obstacle_detected:
            # Already in obstacle mode - need to be farther to exit
            exit_threshold = self.min_safe_distance + self.hysteresis_distance
            self.obstacle_detected = self.min_obstacle_distance < exit_threshold
        else:
            # Not in obstacle mode - enter if too close
            self.obstacle_detected = self.min_obstacle_distance < self.min_safe_distance

        # Analyze sectors for navigation
        self.analyze_scan_sectors_3d(points)

    def analyze_scan_sectors_3d(self, points):
        """Divide 3D point cloud into sectors to determine best direction"""
        # Divide points into sectors based on angle from boat's perspective
        # Front: -45° to +45°, Left: +45° to +135°, Right: -135° to -45°
        front_points = []
        left_points = []
        right_points = []
        
        for x, y, z, dist in points:
            angle = math.atan2(y, x)  # Angle in radians from boat's forward axis
            
            # Wider front sector for better obstacle detection ahead
            if -math.pi/4 < angle < math.pi/4:  # Front sector ±45°
                front_points.append(dist)
            elif math.pi/4 <= angle <= 3*math.pi/4:  # Left sector 45° to 135°
                left_points.append(dist)
            elif -3*math.pi/4 <= angle < -math.pi/4:  # Right sector -135° to -45°
                right_points.append(dist)
        
        # For each sector, find the minimum distance (closest obstacle)
        # Use 10th percentile for robustness against noise
        max_range = 50.0  # meters
        
        if front_points:
            sorted_front = sorted(front_points)
            # Use 10th percentile or minimum if few points
            if len(sorted_front) > 10:
                self.front_clear = min(max_range, sorted_front[len(sorted_front)//10])
            else:
                self.front_clear = min(max_range, sorted_front[0])
        else:
            self.front_clear = max_range
            
        if left_points:
            sorted_left = sorted(left_points)
            if len(sorted_left) > 10:
                self.left_clear = min(max_range, sorted_left[len(sorted_left)//10])
            else:
                self.left_clear = min(max_range, sorted_left[0])
        else:
            self.left_clear = max_range
            
        if right_points:
            sorted_right = sorted(right_points)
            if len(sorted_right) > 10:
                self.right_clear = min(max_range, sorted_right[len(sorted_right)//10])
            else:
                self.right_clear = min(max_range, sorted_right[0])
        else:
            self.right_clear = max_range

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

        # Initialize stuck detection on first run
        if self.last_position is None:
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

        # Check for stuck condition
        self.check_stuck_condition(curr_x, curr_y)

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

        # --- STUCK ESCAPE LOGIC ---
        if self.is_stuck and self.escape_mode:
            self.execute_escape_maneuver()
            return

        # --- OBSTACLE AVOIDANCE LOGIC ---
        if self.min_obstacle_distance < self.critical_distance:
            # CRITICAL: Stop or reverse
            if self.reverse_start_time is None:
                # Just entered reverse mode
                self.reverse_start_time = self.get_clock().now()
                self.integral_error = 0.0  # Reset integral on mode change
                self.get_logger().warn(
                    f"CRITICAL OBSTACLE at {self.min_obstacle_distance:.2f}m - Reversing!"
                )

            # Check if we've been reversing too long
            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                # Timeout - stop reversing and try turning instead
                self.get_logger().warn(f"Reverse timeout ({elapsed:.1f}s) - Switching to turn mode")
                self.reverse_start_time = None
                # Fall through to normal avoidance
            else:
                # Reverse harder in critical zone
                self.send_thrust(-400.0, -400.0)
                self.avoidance_mode = True
                return
        else:
            # Not in critical zone - reset reverse timer
            self.reverse_start_time = None

        # OBSTACLE AVOIDANCE MODE: Override waypoint navigation
        if self.obstacle_detected:
            # Reset integral error when entering avoidance mode
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.get_logger().info("Entering obstacle avoidance mode - PID reset")

            self.avoidance_mode = True

            # Determine which direction has more clearance
            if self.left_clear > self.right_clear:
                # Turn hard left (90 degrees from current heading)
                avoidance_heading = self.current_yaw + math.pi / 2
                direction = "LEFT"
            else:
                # Turn hard right (90 degrees from current heading)
                avoidance_heading = self.current_yaw - math.pi / 2
                direction = "RIGHT"

            # Calculate error to avoidance heading
            angle_error = avoidance_heading - self.current_yaw

            # Normalize to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2.0 * math.pi
            while angle_error < -math.pi:
                angle_error += 2.0 * math.pi

            self.get_logger().info(
                f"OBSTACLE {self.min_obstacle_distance:.1f}m - Turning {direction} "
                f"(L:{self.left_clear:.1f}m R:{self.right_clear:.1f}m)",
                throttle_duration_sec=1.0
            )
        else:
            # NORMAL WAYPOINT NAVIGATION MODE
            if self.avoidance_mode:
                self.get_logger().info("Path clear - Resuming waypoint navigation")
                self.avoidance_mode = False
                self.integral_error = 0.0
                self.previous_error = 0.0  # Reset PID when exiting avoidance

            # Calculate desired heading to waypoint
            target_angle = math.atan2(dy, dx)

            # Calculate heading error
            angle_error = target_angle - self.current_yaw

            # Normalize to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2.0 * math.pi
            while angle_error < -math.pi:
                angle_error += 2.0 * math.pi

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

    def check_stuck_condition(self, curr_x, curr_y):
        """Detect if boat is stuck and hasn't moved significantly"""
        # Don't check if already in escape mode
        if self.escape_mode:
            return

        # Calculate distance moved since last check
        dx = curr_x - self.last_position[0]
        dy = curr_y - self.last_position[1]
        distance_moved = math.hypot(dx, dy)

        # Check elapsed time
        elapsed = (self.get_clock().now() - self.stuck_check_time).nanoseconds / 1e9

        if elapsed >= self.stuck_timeout:
            # Time to check if stuck
            if distance_moved < self.stuck_threshold:
                if not self.is_stuck:
                    self.is_stuck = True
                    self.escape_mode = True
                    self.escape_start_time = self.get_clock().now()
                    self.integral_error = 0.0  # Reset PID
                    
                    # Track consecutive stuck events at same waypoint
                    if self.last_stuck_waypoint == self.current_wp_index:
                        self.consecutive_stuck_count += 1
                    else:
                        self.consecutive_stuck_count = 1
                        self.last_stuck_waypoint = self.current_wp_index
                    
                    self.get_logger().warn(
                        f"STUCK DETECTED! Moved only {distance_moved:.2f}m in {elapsed:.1f}s. "
                        f"Initiating escape maneuver! (Attempt {self.consecutive_stuck_count})"
                    )
                    
                    # Skip waypoint if stuck too many times
                    if self.consecutive_stuck_count >= 3:
                        self.get_logger().error(
                            f"Stuck {self.consecutive_stuck_count} times at waypoint {self.current_wp_index + 1}. "
                            f"Skipping to next waypoint!"
                        )
                        self.current_wp_index += 1
                        self.consecutive_stuck_count = 0
                        self.is_stuck = False
                        self.escape_mode = False
            else:
                # Moved enough - not stuck
                if self.is_stuck:
                    self.get_logger().info("No longer stuck - resuming normal operation")
                self.is_stuck = False
                self.escape_mode = False

            # Reset tracking
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

    def execute_escape_maneuver(self):
        """Execute escape sequence when stuck"""
        elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9

        # Escape sequence: reverse for 5s, then turn hard for 3s, then try forward
        if elapsed < 5.0:
            # Phase 1: Reverse longer and harder to create more distance
            self.send_thrust(-500.0, -500.0)
            self.get_logger().info(f"Escape Phase 1: Reversing ({elapsed:.1f}s)", throttle_duration_sec=1.0)
        elif elapsed < 8.0:
            # Phase 2: Turn hard (choose direction based on LIDAR clearance)
            # Bias towards the significantly clearer side
            clearance_diff = self.left_clear - self.right_clear
            if abs(clearance_diff) > 2.0:  # Significant difference
                if clearance_diff > 0:
                    self.send_thrust(-400.0, 400.0)  # Turn left
                    direction = "LEFT"
                else:
                    self.send_thrust(400.0, -400.0)  # Turn right
                    direction = "RIGHT"
            else:
                # Similar clearance - turn toward waypoint if possible
                if self.current_wp_index < len(self.waypoints):
                    curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
                    target_x, target_y = self.waypoints[self.current_wp_index]
                    target_bearing = math.atan2(target_y - curr_y, target_x - curr_x)
                    heading_diff = target_bearing - self.current_yaw
                    # Normalize
                    while heading_diff > math.pi:
                        heading_diff -= 2.0 * math.pi
                    while heading_diff < -math.pi:
                        heading_diff += 2.0 * math.pi
                    
                    if heading_diff > 0:
                        self.send_thrust(-400.0, 400.0)  # Turn left
                        direction = "LEFT (toward waypoint)"
                    else:
                        self.send_thrust(400.0, -400.0)  # Turn right
                        direction = "RIGHT (toward waypoint)"
                else:
                    # Default: turn left
                    self.send_thrust(-400.0, 400.0)
                    direction = "LEFT (default)"
            
            self.get_logger().info(
                f"Escape Phase 2: Turning {direction} ({elapsed:.1f}s)",
                throttle_duration_sec=1.0
            )
        elif elapsed < 10.0:
            # Phase 3: Try moving forward slowly
            self.send_thrust(300.0, 300.0)
            self.get_logger().info(f"Escape Phase 3: Forward test ({elapsed:.1f}s)", throttle_duration_sec=1.0)
        else:
            # Escape complete - exit escape mode
            self.get_logger().info("Escape maneuver complete - resuming navigation")
            self.escape_mode = False
            self.is_stuck = False
            self.integral_error = 0.0
            self.previous_error = 0.0
            # Don't reset consecutive counter yet - wait to see if we're truly free
            # Reset position tracking
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

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
