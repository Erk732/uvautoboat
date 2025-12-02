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
        
        # Obstacle avoidance parameters
        self.declare_parameter('min_safe_distance', 15.0)
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('obstacle_slow_factor', 0.3)
        self.declare_parameter('hysteresis_distance', 2.0)
        self.declare_parameter('reverse_timeout', 5.0)

        # Stuck detection parameters
        self.declare_parameter('stuck_timeout', 5.0) #try 5 seconds 
        self.declare_parameter('stuck_threshold', 1.0)
        
        # Smart anti-stuck parameters
        self.declare_parameter('no_go_zone_radius', 8.0)
        self.declare_parameter('drift_compensation_gain', 0.3)
        self.declare_parameter('probe_angle', 45.0)
        self.declare_parameter('detour_distance', 12.0)

        # Get parameters
        self.base_speed = self.get_parameter('base_speed').value
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
        self.no_go_zone_radius = self.get_parameter('no_go_zone_radius').value
        self.drift_compensation_gain = self.get_parameter('drift_compensation_gain').value
        self.probe_angle = self.get_parameter('probe_angle').value
        self.detour_distance = self.get_parameter('detour_distance').value

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
        self.avoidance_mode = False
        self.reverse_start_time = None
        
        # Stuck Detection State
        self.last_position = None
        self.stuck_check_time = None
        self.is_stuck = False
        self.escape_mode = False
        self.escape_start_time = None
        self.consecutive_stuck_count = 0
        self.last_stuck_waypoint = None
        self.no_go_zones = []
        self.escape_history = []
        self.drift_vector = (0.0, 0.0)
        self.position_history = []
        self.escape_phase = 0
        self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}
        self.best_escape_direction = None
        self.detour_waypoint_inserted = False
        self.last_escape_position = None
        self.adaptive_escape_duration = 12.0
        
        # Statistics
        self.total_distance = 0.0
        self.start_time = None
        self._last_log_time = self.get_clock().now()

        # --- SUBSCRIBERS ---
        self.create_subscription(Path, '/atlantis/path', self.path_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, rclpy.qos.qos_profile_sensor_data)

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_mission_status = self.create_publisher(String, '/atlantis/mission_status', 10)
        self.pub_obstacle_status = self.create_publisher(String, '/atlantis/obstacle_status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/atlantis/anti_stuck_status', 10)

        # --- TIMER ---
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(0.5, self.publish_anti_stuck_status)
        
        self.get_logger().info("Atlantis Controller Ready - Waiting for GPS and Path...")

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
                self.start_time = self.get_clock().now()

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"Home Position Set: {self.start_gps}")
            if len(self.waypoints) > 0:
                self.state = "DRIVING"
                self.start_time = self.get_clock().now()

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
        
        for i in range(0, len(data) - point_step, point_step * 10):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                if math.isnan(x) or math.isinf(x): continue
                if z < -0.2 or z > 3.0: continue
                dist = math.sqrt(x*x + y*y)
                if dist < 1.0 or dist > 100.0: continue
                if x < 0.5: continue # Only forward points
                points.append((x, y, z, dist))
            except struct.error:
                continue

        if not points:
            self.min_obstacle_distance = 50.0
            if not self.obstacle_detected:
                self.front_clear = 50.0
                self.left_clear = 50.0
                self.right_clear = 50.0
            return

        distances = [p[3] for p in points]
        self.min_obstacle_distance = min(distances)
        
        # Hysteresis
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
            if -math.pi/4 < angle < math.pi/4:
                front_points.append(dist)
            elif math.pi/4 <= angle <= 3*math.pi/4:
                left_points.append(dist)
            elif -3*math.pi/4 <= angle < -math.pi/4:
                right_points.append(dist)
        
        max_range = 100.0
        
        if front_points:
            sorted_front = sorted(front_points)
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
        R = 6371000.0
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])
        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return y, x

    def control_loop(self):
        if self.state != "DRIVING" or self.current_gps is None or not self.waypoints:
            return

        # 1. Get Position
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        # Initialize stuck detection
        if self.last_position is None:
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

        # Check Stuck
        self.check_stuck_condition(curr_x, curr_y)

        # 2. Check Waypoint Reached
        if self.current_wp_index >= len(self.waypoints):
            self.finish_mission(curr_x, curr_y)
            return

        target_x, target_y = self.waypoints[self.current_wp_index]
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)
        
        if dist < self.waypoint_tolerance:
            self.get_logger().info(f"Waypoint {self.current_wp_index + 1}/{len(self.waypoints)} Reached")
            self.current_wp_index += 1
            self.total_distance += dist
            self.integral_error = 0.0
            return

        # --- STUCK ESCAPE ---
        if self.is_stuck and self.escape_mode:
            self.execute_escape_maneuver()
            return

        # --- CRITICAL OBSTACLE ---
        if self.min_obstacle_distance < self.critical_distance:
            if self.reverse_start_time is None:
                self.reverse_start_time = self.get_clock().now()
                self.integral_error = 0.0
                self.get_logger().warn(f"CRITICAL OBSTACLE {self.min_obstacle_distance:.2f}m - Reversing!")
            
            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                self.get_logger().warn("Reverse timeout - Switching to turn mode")
                self.reverse_start_time = None
            else:
                self.send_thrust(-800.0, -800.0)
                self.avoidance_mode = True
                return
        else:
            self.reverse_start_time = None

        # --- NORMAL / AVOIDANCE DRIVING ---
        if self.obstacle_detected:
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.get_logger().info("Obstacle avoidance mode active")
            self.avoidance_mode = True

            if self.left_clear > self.right_clear:
                avoidance_heading = self.current_yaw + math.pi / 2
                direction = "LEFT"
            else:
                avoidance_heading = self.current_yaw - math.pi / 2
                direction = "RIGHT"
            
            angle_error = avoidance_heading - self.current_yaw
            while angle_error > math.pi: angle_error -= 2.0 * math.pi
            while angle_error < -math.pi: angle_error += 2.0 * math.pi
            
            self.get_logger().warn(f"OBSTACLE! Turning {direction}", throttle_duration_sec=1.0)
        else:
            if self.avoidance_mode:
                self.get_logger().info("Path CLEAR - Resuming navigation")
                self.avoidance_mode = False
                self.integral_error = 0.0
            
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_yaw
            while angle_error > math.pi: angle_error -= 2.0 * math.pi
            while angle_error < -math.pi: angle_error += 2.0 * math.pi

        # PID
        self.integral_error += angle_error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error))
        derivative_error = (angle_error - self.previous_error) / self.dt
        
        turn_power = (self.kp * angle_error + self.ki * self.integral_error + self.kd * derivative_error)
        self.previous_error = angle_error
        turn_power = max(-800.0, min(800.0, turn_power))

        # Speed logic
        speed = self.base_speed
        angle_deg = abs(math.degrees(angle_error))
        if angle_deg > 45: speed *= 0.5
        elif angle_deg > 20: speed *= 0.75
        
        if dist < 5.0: speed *= 0.7
        if self.obstacle_detected: speed *= self.obstacle_slow_factor
        
        left_thrust = max(-1000.0, min(1000.0, speed - turn_power))
        right_thrust = max(-1000.0, min(1000.0, speed + turn_power))
        
        self.send_thrust(left_thrust, right_thrust)
        self.publish_dashboard_status(curr_x, curr_y, target_x, target_y, dist)
        
        # Logging
        if hasattr(self, '_last_log_time'):
            now = self.get_clock().now()
            if (now - self._last_log_time).nanoseconds / 1e9 > 2.0:
                self.log_status(curr_x, curr_y, target_x, target_y, dist, angle_error)
                self._last_log_time = now
        else:
            self._last_log_time = self.get_clock().now()

    def check_stuck_condition(self, curr_x, curr_y):
        if self.escape_mode: return

        dx = curr_x - self.last_position[0]
        dy = curr_y - self.last_position[1]
        distance_moved = math.hypot(dx, dy)
        
        self.position_history.append((curr_x, curr_y, self.get_clock().now()))
        if len(self.position_history) > 100: self.position_history.pop(0)
        
        self.estimate_drift()
        
        elapsed = (self.get_clock().now() - self.stuck_check_time).nanoseconds / 1e9
        if elapsed >= self.stuck_timeout: # Time to check
            if distance_moved < self.stuck_threshold:
                if not self.is_stuck:
                    self.is_stuck = True
                    self.escape_mode = True
                    self.escape_start_time = self.get_clock().now()
                    self.escape_phase = 0
                    self.integral_error = 0.0
                    self.last_escape_position = (curr_x, curr_y)
                    self.best_escape_direction = None
                    self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}
                    self.add_no_go_zone(curr_x, curr_y)
                    self.last_position = (curr_x, curr_y)  # Reset reference position
                    self.stuck_check_time = self.get_clock().now()  # Reset timer
                    
                    if self.last_stuck_waypoint == self.current_wp_index:
                        self.consecutive_stuck_count += 1
                    else:
                        self.consecutive_stuck_count = 1
                        self.last_stuck_waypoint = self.current_wp_index
                        
                    self.calculate_adaptive_escape_duration()
                    self.get_logger().warn(f"STUCK DETECTED! Initiating smart escape (Attempt {self.consecutive_stuck_count})")
                    
                    if self.consecutive_stuck_count == 2 and not self.detour_waypoint_inserted:
                        self.insert_detour_waypoint(curr_x, curr_y)
                    
                    if self.consecutive_stuck_count >= 4:
                        self.get_logger().error(f"Stuck too many times ({self.consecutive_stuck_count}) - Skipping Waypoint!")
                        self.current_wp_index += 1
                        self.consecutive_stuck_count = 0
                        self.detour_waypoint_inserted = False
                        self.is_stuck = False
                        self.escape_mode = False
            else:
                if self.is_stuck:
                    self.record_escape_result(success=True)
                    self.get_logger().info("Escape successful - resuming normal operation")
                self.is_stuck = False
                self.escape_mode = False
                self.escape_phase = 0
                
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

    def execute_escape_maneuver(self):
        elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
        
        probe_end = 2.0
        reverse_end = probe_end + self.adaptive_escape_duration * 0.4
        turn_end = reverse_end + self.adaptive_escape_duration * 0.35
        forward_end = turn_end + self.adaptive_escape_duration * 0.25
        
        drift_comp_left, drift_comp_right = self.calculate_drift_compensation()

        if elapsed < probe_end:
            self.escape_phase = 0
            probe_time = elapsed
            if probe_time < 0.6:
                self.send_thrust(-200.0, 200.0)
                self.probe_results['left'] = max(self.probe_results['left'], self.left_clear)
            elif probe_time < 1.2:
                self.send_thrust(200.0, -200.0)
                self.probe_results['right'] = max(self.probe_results['right'], self.right_clear)
            else:
                self.send_thrust(0.0, 0.0)
                self.probe_results['back'] = self.min_obstacle_distance
                if self.best_escape_direction is None:
                    self.best_escape_direction = self.determine_best_escape_direction()
                    self.get_logger().info(f"Probe done. Best direction: {self.best_escape_direction}")
            return

        elif elapsed < reverse_end:
            self.escape_phase = 1
            reverse_power = -700.0 - min(300.0, 100.0 / max(0.5, self.min_obstacle_distance))
            self.send_thrust(reverse_power + drift_comp_left, reverse_power + drift_comp_right)
            return

        elif elapsed < turn_end:
            self.escape_phase = 2
            turn_power = min(900.0, 600.0 + (self.consecutive_stuck_count * 100.0))
            
            if self.best_escape_direction == 'LEFT':
                self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
            elif self.best_escape_direction == 'RIGHT':
                self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
            else:
                bias = self.get_learned_escape_direction()
                if bias == 'LEFT':
                    self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
                else:
                    self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
            return

        elif elapsed < forward_end:
            self.escape_phase = 3
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            if self.is_heading_toward_no_go_zone(curr_x, curr_y):
                self.send_thrust(-400.0, 400.0)
            else:
                self.send_thrust(400.0 + drift_comp_left, 400.0 + drift_comp_right)
            return

        else:
            self.get_logger().info("Escape maneuver complete")
            self.escape_mode = False
            self.is_stuck = False
            self.escape_phase = 0
            self.integral_error = 0.0
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

    # --- HELPER METHODS ---
    def calculate_adaptive_escape_duration(self):
        base_duration = 10.0
        if self.min_obstacle_distance < self.critical_distance: base_duration += 4.0
        elif self.min_obstacle_distance < self.min_safe_distance: base_duration += 2.0
        base_duration += self.consecutive_stuck_count * 2.0
        self.adaptive_escape_duration = min(20.0, base_duration)
        
    def add_no_go_zone(self, x, y):
        for zone in self.no_go_zones:
            if math.hypot(x - zone[0], y - zone[1]) < self.no_go_zone_radius:
                return
        self.no_go_zones.append((x, y, self.no_go_zone_radius))
        if len(self.no_go_zones) > 20: self.no_go_zones.pop(0)

    def is_in_no_go_zone(self, x, y):
        for zone_x, zone_y, radius in self.no_go_zones:
            if math.hypot(x - zone_x, y - zone_y) < radius: return True
        return False
        
    def is_heading_toward_no_go_zone(self, curr_x, curr_y):
        look_ahead = 10.0
        future_x = curr_x + look_ahead * math.cos(self.current_yaw)
        future_y = curr_y + look_ahead * math.sin(self.current_yaw)
        return self.is_in_no_go_zone(future_x, future_y)

    def determine_best_escape_direction(self):
        left, right, back = self.probe_results['left'], self.probe_results['right'], self.probe_results['back']
        threshold = 3.0
        if left > right + threshold and left > back: return 'LEFT'
        elif right > left + threshold and right > back: return 'RIGHT'
        elif back > max(left, right) + threshold: return 'BACK'
        return self.get_learned_escape_direction()

    def estimate_drift(self):
        if len(self.position_history) < 20: return
        recent = self.position_history[-20:]
        total_dx = recent[-1][0] - recent[0][0]
        total_dy = recent[-1][1] - recent[0][1]
        time_diff = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        if time_diff > 0.5:
            alpha = 0.1
            self.drift_vector = (
                alpha * (total_dx/time_diff) + (1-alpha) * self.drift_vector[0],
                alpha * (total_dy/time_diff) + (1-alpha) * self.drift_vector[1]
            )

    def calculate_drift_compensation(self):
        mag = math.hypot(self.drift_vector[0], self.drift_vector[1])
        if mag < 0.1: return 0.0, 0.0
        drift_angle = math.atan2(self.drift_vector[1], self.drift_vector[0])
        rel = drift_angle - self.current_yaw
        while rel > math.pi: rel -= 2.0*math.pi
        while rel < -math.pi: rel += 2.0*math.pi
        comp = min(150.0, mag * self.drift_compensation_gain * 100.0)
        return (comp, -comp) if rel > 0 else (-comp, comp)

    def insert_detour_waypoint(self, curr_x, curr_y):
        detour_angle = self.current_yaw + (math.pi/2 if self.left_clear > self.right_clear else -math.pi/2)
        detour_x = curr_x + self.detour_distance * math.cos(detour_angle)
        detour_y = curr_y + self.detour_distance * math.sin(detour_angle)
        
        if self.is_in_no_go_zone(detour_x, detour_y):
            detour_angle += math.pi
            detour_x = curr_x + self.detour_distance * math.cos(detour_angle)
            detour_y = curr_y + self.detour_distance * math.sin(detour_angle)
            
        self.waypoints.insert(self.current_wp_index, (detour_x, detour_y))
        self.detour_waypoint_inserted = True
        self.get_logger().warn(f"DETOUR INSERTED at ({detour_x:.1f}, {detour_y:.1f})")

    def record_escape_result(self, success):
        if self.last_escape_position is None: return
        self.escape_history.append({
            'position': self.last_escape_position,
            'direction': self.best_escape_direction,
            'success': success
        })
        if len(self.escape_history) > 50: self.escape_history.pop(0)

    def get_learned_escape_direction(self):
        if not self.escape_history: return 'LEFT'
        left_score = sum(1 for r in self.escape_history if r['direction'] == 'LEFT' and r['success'])
        right_score = sum(1 for r in self.escape_history if r['direction'] == 'RIGHT' and r['success'])
        return 'LEFT' if left_score >= right_score else 'RIGHT'

    def send_thrust(self, left, right):
        self.pub_left.publish(Float64(data=float(left)))
        self.pub_right.publish(Float64(data=float(right)))

    def stop_boat(self):
        self.send_thrust(0.0, 0.0)

    def log_status(self, curr_x, curr_y, target_x, target_y, dist, error):
        wp = f"{self.current_wp_index + 1}/{len(self.waypoints)}"
        obs = f"OBS:{self.min_obstacle_distance:.1f}m" if self.obstacle_detected else "CLEAR"
        self.get_logger().info(f"WP {wp} | Pos:({curr_x:.1f},{curr_y:.1f}) | Dist:{dist:.1f}m | Err:{math.degrees(error):.1f}deg | {obs}")

    def finish_mission(self, final_x, final_y):
        self.state = "FINISHED"
        self.stop_boat()
        self.get_logger().info("="*60)
        self.get_logger().info("MISSION COMPLETE!")
        self.get_logger().info(f"Total Distance: {self.total_distance:.1f}m")
        self.get_logger().info("="*60)

    def publish_dashboard_status(self, curr_x, curr_y, target_x, target_y, dist):
        state_key = "STUCK_ESCAPING" if self.escape_mode else \
                    "OBSTACLE_AVOIDING" if self.avoidance_mode else \
                    "MISSION_COMPLETE" if self.state == "FINISHED" else \
                    "MOVING_TO_WAYPOINT" if self.state == "DRIVING" else self.state
        
        msg = String()
        msg.data = json.dumps({
            "state": state_key,
            "waypoint": self.current_wp_index + 1,
            "total_waypoints": len(self.waypoints),
            "distance_to_waypoint": round(dist, 1)
        })
        self.pub_mission_status.publish(msg)
        
        status_text = f"OBSTACLE {round(self.min_obstacle_distance, 1)}m" if self.min_obstacle_distance < self.min_safe_distance else "CLEAR"
        obs_msg = String()
        obs_msg.data = json.dumps({
            "status": status_text,
            "min_distance": round(self.min_obstacle_distance, 1) if self.min_obstacle_distance != float('inf') else 999.9,
            "front_clear": self.front_clear != float('inf'),
            "left_clear": self.left_clear > self.min_safe_distance,
            "right_clear": self.right_clear > self.min_safe_distance
        })
        self.pub_obstacle_status.publish(obs_msg)
        
    def publish_anti_stuck_status(self):
        msg = String()
        msg.data = json.dumps({
            'is_stuck': self.is_stuck,
            'escape_mode': self.escape_mode,
            'consecutive_attempts': self.consecutive_stuck_count,
            'adaptive_duration': round(self.adaptive_escape_duration, 1)
        })
        self.pub_anti_stuck.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_boat()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()