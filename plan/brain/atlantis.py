# new code by referencing to vostok1 and apollo11 created at 01.12.25 
# OV-104 "ATLANTIS" Autonomous Navigation System with Stuck Avoidance System v2.0
# control and planning combined into single node for advanced testing purposes! Don't forget to seperate later!
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import math
import struct
import json

# Messages
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import Float64, String

class Atlantis(Node):
    def __init__(self):
        super().__init__('atlantis_node')

        # --- CONFIGURATION PARAMETERS ---
        self.declare_parameter('scan_length', 30.0) 
        self.declare_parameter('scan_width', 15.0)  
        self.declare_parameter('lanes', 6)
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 3.0)

        # PID Controller gains
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)

        # Obstacle avoidance parameters
        self.declare_parameter('min_safe_distance', 15.0)
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('obstacle_slow_factor', 0.3)
        self.declare_parameter('hysteresis_distance', 2.0)  
        self.declare_parameter('reverse_timeout', 3.0)  # Max seconds to reverse

        # Stuck detection parameters
        self.declare_parameter('stuck_timeout', 5.0)  
        self.declare_parameter('stuck_threshold', 1.0) 
        
        # Smart anti-stuck parameters
        self.declare_parameter('no_go_zone_radius', 8.0) 
        self.declare_parameter('drift_compensation_gain', 0.3) 
        self.declare_parameter('probe_angle', 45.0) 
        self.declare_parameter('detour_distance', 12.0) 

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
        
        # Smart anti-stuck parameters
        self.no_go_zone_radius = self.get_parameter('no_go_zone_radius').value
        self.drift_compensation_gain = self.get_parameter('drift_compensation_gain').value
        self.probe_angle = self.get_parameter('probe_angle').value
        self.detour_distance = self.get_parameter('detour_distance').value

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
        
        # Smart anti-stuck state
        self.no_go_zones = []  
        self.escape_history = []  
        self.drift_vector = (0.0, 0.0) 
        self.position_history = []  
        self.expected_positions = [] 
        self.escape_phase = 0  
        self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0} 
        self.best_escape_direction = None 
        self.detour_waypoint_inserted = False 
        self.last_escape_position = None 
        self.adaptive_escape_duration = 12.0 

        # Statistics
        self.total_distance = 0.0
        self.start_time = None

        # --- SUBSCRIBERS ---
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # Dashboard status publishers
        self.pub_mission_status = self.create_publisher(String, '/atlantis/mission_status', 10)
        self.pub_obstacle_status = self.create_publisher(String, '/atlantis/obstacle_status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/atlantis/anti_stuck_status', 10)
        
        # Parameter configuration publisher 
        self.pub_config = self.create_publisher(String, '/atlantis/config', 10)
        
        # Parameter update subscriber 
        self.create_subscription(String, '/atlantis/set_config', self.config_callback, 10)
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- CONTROL LOOP (20Hz) ---
        self.dt = 0.05 
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(1.0, self.publish_config)
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("OV-104 'ATLANTIS' - Autonomous Navigation System")
        self.get_logger().info("+ Stuck Avoidance System 2.0")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Scan Zone: {self.scan_length}m x {self.scan_width * self.lanes}m")
        self.get_logger().info("Waiting for GPS signal...")
        self.get_logger().info("=" * 60)

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)

        # First GPS fix - initialize mission
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Launch position has been established: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
            self.generate_lawnmower_path()
            self.state = "DRIVING"
            self.get_logger().info("Mission Started.")
            self.get_logger().info("=" * 60)

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
        
        # SAMPLE REDUCTION: Process every 10th point to save CPU
        for i in range(0, len(data) - point_step, point_step * 10):
            try:
                # Extract x, y, z from point cloud data
                x, y, z = struct.unpack_from('fff', data, i)
                
                # Filter bad data
                if math.isnan(x) or math.isinf(x): continue
                
                # Filter by height (Water surface ~ -0.2m to 3.0m high)
                if z < -0.2 or z > 3.0: continue
                
                # Filter by distance (Ignore self < 1m, ignore far > 100m)
                dist = math.sqrt(x*x + y*y)
                if dist < 1.0 or dist > 100.0: continue
                
                # Ignore points BEHIND the lidar (x < 0) to avoid self-detection
                if x < 0.5: continue
                
                points.append((x, y, z, dist))
                    
            except struct.error:
                continue
            except Exception:
                continue
        
        if not points:
            self.min_obstacle_distance = 50.0
            if not self.obstacle_detected:
                self.front_clear = 50.0
                self.left_clear = 50.0
                self.right_clear = 50.0
            return
        
        # Get minimum distance from all valid points
        distances = [p[3] for p in points]
        self.min_obstacle_distance = min(distances)

        # Hysteresis for obstacle detection
        if self.obstacle_detected:
            exit_threshold = self.min_safe_distance + self.hysteresis_distance
            self.obstacle_detected = self.min_obstacle_distance < exit_threshold
        else:
            self.obstacle_detected = self.min_obstacle_distance < self.min_safe_distance

        self.analyze_scan_sectors_3d(points)

    def analyze_scan_sectors_3d(self, points):
        """Divide 3D point cloud into sectors"""
        front_points = []
        left_points = []
        right_points = []
        
        for x, y, z, dist in points:
            angle = math.atan2(y, x) 
            
            if -math.pi/4 < angle < math.pi/4:  # Front ±45°
                front_points.append(dist)
            elif math.pi/4 <= angle <= 3*math.pi/4:  # Left
                left_points.append(dist)
            elif -3*math.pi/4 <= angle < -math.pi/4:  # Right
                right_points.append(dist)
        
        max_range = 100.0
        
        # Helper to get safe min
        def get_sector_min(pts):
            if not pts: return max_range
            pts.sort()
            # Use 10th percentile to ignore noise spikes
            idx = len(pts) // 10
            return min(max_range, pts[idx]) if len(pts) > 10 else pts[0]

        self.front_clear = get_sector_min(front_points)
        self.left_clear = get_sector_min(left_points)
        self.right_clear = get_sector_min(right_points)

    def latlon_to_meters(self, lat, lon):
        R = 6371000.0 
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])
        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return y, x # East, North

    def generate_lawnmower_path(self):
        self.waypoints = []
        for i in range(self.lanes):
            x_end = self.scan_length if (i % 2 == 0) else 0.0
            y_pos = i * self.scan_width
            self.waypoints.append((x_end, y_pos))

            if i < self.lanes - 1:
                next_y = (i + 1) * self.scan_width
                self.waypoints.append((x_end, next_y))

        self.get_logger().info(f"Voyage Plan Generated: {len(self.waypoints)} waypoints")

    def publish_config(self):
        config = {
            'scan_length': self.scan_length,
            'scan_width': self.scan_width,
            'lanes': self.lanes,
            'kp': self.kp,
            'base_speed': self.base_speed,
            'state': self.state
        }
        msg = String()
        msg.data = json.dumps(config)
        self.pub_config.publish(msg)

    def config_callback(self, msg):
        try:
            config = json.loads(msg.data)
            self.get_logger().info("CONFIG UPDATE RECEIVED")
            
            regenerate_path = False
            
            if 'scan_length' in config and config['scan_length'] != self.scan_length:
                self.scan_length = float(config['scan_length'])
                regenerate_path = True
            if 'scan_width' in config and config['scan_width'] != self.scan_width:
                self.scan_width = float(config['scan_width'])
                regenerate_path = True
            if 'restart_mission' in config and config['restart_mission']:
                self.get_logger().info("MISSION RESTART")
                self.current_wp_index = 0
                self.state = "DRIVING"
                self.integral_error = 0.0
                regenerate_path = True
            
            if regenerate_path and self.start_gps is not None:
                self.generate_lawnmower_path()
        except Exception as e:
            self.get_logger().error(f"Config update error: {e}")

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parameter changed: {param.name} = {param.value}")
            if param.name == 'kp': self.kp = param.value
            elif param.name == 'ki': self.ki = param.value
            elif param.name == 'kd': self.kd = param.value
            elif param.name == 'base_speed': self.base_speed = param.value
        return SetParametersResult(successful=True)

    def control_loop(self):
        if self.state != "DRIVING" or self.current_gps is None:
            return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        if self.last_position is None:
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

        # Check Stuck Condition
        self.check_stuck_condition(curr_x, curr_y)

        if self.current_wp_index >= len(self.waypoints):
            self.finish_mission(curr_x, curr_y)
            return

        target_x, target_y = self.waypoints[self.current_wp_index]
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)

        # Check whatever it is reached the waypoint ?
        if dist < self.waypoint_tolerance:
            self.get_logger().info(f"Waypoint {self.current_wp_index + 1} reached.")
            self.current_wp_index += 1
            self.total_distance += dist
            self.integral_error = 0.0
            return

        # --- ESCAPE MANEUVER ---
        if self.is_stuck and self.escape_mode:
            self.execute_escape_maneuver()
            return

        # --- CRITICAL AVOIDANCE ---
        if self.min_obstacle_distance < self.critical_distance:
            if self.reverse_start_time is None:
                self.reverse_start_time = self.get_clock().now()
                self.get_logger().warn("OBSTACLE DETECTED - REVERSING!")

            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                # Timed out, stop reversing
                self.reverse_start_time = None
            else:
                self.send_thrust(-800.0, -800.0)
                self.avoidance_mode = True
                return
        else:
            self.reverse_start_time = None

        # --- OBSTACLE AVOIDANCE STEERING ---
        if self.obstacle_detected:
            if not self.avoidance_mode:
                self.integral_error = 0.0 # Reset PID
            self.avoidance_mode = True

            if self.left_clear > self.right_clear:
                # Turn Left
                target_angle = self.current_yaw + math.pi / 2
                direction = "LEFT"
            else:
                # Turn Right
                target_angle = self.current_yaw - math.pi / 2
                direction = "RIGHT"
                
            self.get_logger().warn(f"Avoiding Obstacle -> Turning {direction}", throttle_duration_sec=2.0)
        else:
            # NORMAL NAVIGATION
            if self.avoidance_mode:
                self.avoidance_mode = False
                self.integral_error = 0.0
            
            # Standard angle to waypoint
            target_angle = math.atan2(dy, dx)

        # Calculate Angle Error
        angle_error = target_angle - self.current_yaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi

        # PID Controller Calculation
        self.integral_error += angle_error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error)) # Anti-windup
        
        derivative_error = (angle_error - self.previous_error) / self.dt
        
        turn_power = (self.kp * angle_error + 
                      self.ki * self.integral_error + 
                      self.kd * derivative_error)
        
        self.previous_error = angle_error

        # Power Limits
        turn_power = max(-800.0, min(800.0, turn_power))
        
        # Adaptive Speed (Slow down for sharp turns)
        speed = self.base_speed
        if abs(math.degrees(angle_error)) > 20:
            speed *= 0.75
        if self.obstacle_detected:
            speed *= self.obstacle_slow_factor

        # Mix Thrust
        left_thrust = speed - turn_power
        right_thrust = speed + turn_power
        
        # Clamp
        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))

        self.send_thrust(left_thrust, right_thrust)
        self.publish_dashboard_status(curr_x, curr_y, target_x, target_y, dist)

    def log_status(self, curr_x, curr_y, target_x, target_y, dist, error):
        obs = "OBS DETECTED" if self.obstacle_detected else "CLEAR"
        self.get_logger().info(
            f"WP {self.current_wp_index + 1}/{len(self.waypoints)} | "
            f"Pos:({curr_x:.1f}, {curr_y:.1f}) | Dist:{dist:.1f}m | {obs}"
        )

    def finish_mission(self, final_x, final_y):
        self.state = "FINISHED"
        self.stop_boat()
        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE! Thruster full stop.")
        self.get_logger().info("=" * 60)

    def send_thrust(self, left, right):
        msg_l = Float64()
        msg_r = Float64()
        msg_l.data = float(left)
        msg_r.data = float(right)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def check_stuck_condition(self, curr_x, curr_y):
        if self.escape_mode: return

        # Drift tracking
        self.position_history.append((curr_x, curr_y, self.get_clock().now()))
        if len(self.position_history) > 100: self.position_history.pop(0)
        self.estimate_drift()

        dx = curr_x - self.last_position[0]
        dy = curr_y - self.last_position[1]
        moved = math.hypot(dx, dy)
        elapsed = (self.get_clock().now() - self.stuck_check_time).nanoseconds / 1e9

        if elapsed >= self.stuck_timeout:
            if moved < self.stuck_threshold:
                if not self.is_stuck:
                    self.is_stuck = True
                    self.escape_mode = True
                    self.escape_start_time = self.get_clock().now()
                    self.escape_phase = 0
                    self.consecutive_stuck_count += 1
                    self.calculate_adaptive_escape_duration()
                    self.get_logger().warn(f"WAMV IS STUCK! (Attempt {self.consecutive_stuck_count})")
            else:
                self.is_stuck = False
                self.consecutive_stuck_count = 0
            
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

    def execute_escape_maneuver(self):
        """Phase-based escape maneuver"""
        elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
        
        probe_end = 2.0
        reverse_end = probe_end + self.adaptive_escape_duration * 0.4
        turn_end = reverse_end + self.adaptive_escape_duration * 0.35
        forward_end = turn_end + self.adaptive_escape_duration * 0.25
        
        drift_L, drift_R = self.calculate_drift_compensation()

        # PHASE 0: PROBE
        if elapsed < probe_end:
            if elapsed < 0.6: # Look Left
                self.send_thrust(-200.0, 200.0)
                self.probe_results['left'] = max(self.probe_results['left'], self.left_clear)
            elif elapsed < 1.2: # Look Right
                self.send_thrust(200.0, -200.0)
                self.probe_results['right'] = max(self.probe_results['right'], self.right_clear)
            else: # Assess
                self.send_thrust(0.0, 0.0)
                if self.best_escape_direction is None:
                    self.best_escape_direction = self.determine_best_escape_direction()
            return

        # PHASE 1: REVERSE
        elif elapsed < reverse_end:
            rev_pow = -700.0
            self.send_thrust(rev_pow + drift_L, rev_pow + drift_R)
            return

        # PHASE 2: TURN
        elif elapsed < turn_end:
            turn = 700.0
            if self.best_escape_direction == 'LEFT':
                self.send_thrust(-turn + drift_L, turn + drift_R)
            else:
                self.send_thrust(turn + drift_L, -turn + drift_R)
            return

        # PHASE 3: FORWARD TEST
        elif elapsed < forward_end:
            self.send_thrust(500.0 + drift_L, 500.0 + drift_R)
            return

        else:
            self.get_logger().info("Escape Manuver Complete. Resuming.")
            self.escape_mode = False
            self.is_stuck = False
            self.last_position = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            self.stuck_check_time = self.get_clock().now()

    def calculate_adaptive_escape_duration(self):
        base = 6.0 + (self.consecutive_stuck_count * 2.0)
        self.adaptive_escape_duration = min(15.0, base)
    
    def determine_best_escape_direction(self):
        # Choose direction with most space
        if self.probe_results['left'] > self.probe_results['right']:
            return 'LEFT'
        return 'RIGHT'

    def estimate_drift(self):
        if len(self.position_history) < 20: return
        recent = self.position_history[-20:]
        dx = recent[-1][0] - recent[0][0]
        dy = recent[-1][1] - recent[0][1]
        dt = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        
        if dt > 0.5:
            self.drift_vector = (dx/dt, dy/dt)

    def calculate_drift_compensation(self):
        drift_mag = math.hypot(self.drift_vector[0], self.drift_vector[1])
        if drift_mag < 0.1: return 0.0, 0.0
        
        drift_angle = math.atan2(self.drift_vector[1], self.drift_vector[0])
        rel_drift = drift_angle - self.current_yaw
        
        # Compensate against drift
        comp = min(150.0, drift_mag * 50.0)
        if math.sin(rel_drift) > 0: # Drifting Left
            return comp, -comp # Push Right
        else:
            return -comp, comp

    def publish_anti_stuck_status(self):
        msg = String()
        data = {
            'is_stuck': self.is_stuck,
            'escape_mode': self.escape_mode,
            'attempts': self.consecutive_stuck_count
        }
        msg.data = json.dumps(data)
        self.pub_anti_stuck.publish(msg)

    def stop_boat(self):
        self.send_thrust(0.0, 0.0)

    def publish_dashboard_status(self, curr_x, curr_y, tx, ty, dist):
        status = "DRIVING"
        if self.escape_mode: status = "ESCAPING"
        elif self.avoidance_mode: status = "AVOIDING"
        elif self.state == "FINISHED": status = "FINISHED"

        data = {
            "state": status,
            "waypoint": self.current_wp_index + 1,
            "dist_to_wp": round(dist, 1),
            "obstacle_dist": round(self.min_obstacle_distance, 1) if self.min_obstacle_distance < 99 else 999.9
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub_mission_status.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Atlantis()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()