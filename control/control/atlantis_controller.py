import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import Float64, String, Empty
import math
import struct
import json
import time # Added for safety sleep

class AtlantisController(Node):
    def __init__(self):
        super().__init__('atlantis_controller')

        # --- PARAMETERS ---
        self.declare_parameter('auto_start_mission', True)  # Auto-enable when GPS + path ready
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)
        
        # Obstacle parameters
        self.declare_parameter('min_safe_distance', 10.0)  # Earlier detection
        self.declare_parameter('critical_distance', 3.0)   # Much faster response
        self.declare_parameter('obstacle_slow_factor', 0.2)  # Increased from 0.3 - slow down more
        self.declare_parameter('hysteresis_distance', 2.0)
        self.declare_parameter('reverse_timeout', 8.0)    # Increased from 5s - reverse longer if stuck

        # Stuck detection
        self.declare_parameter('stuck_timeout', 5.0)
        self.declare_parameter('stuck_threshold', 1.0)
        
        # Anti-stuck
        self.declare_parameter('no_go_zone_radius', 8.0)
        self.declare_parameter('drift_compensation_gain', 0.3)
        self.declare_parameter('probe_angle', 45.0)
        self.declare_parameter('detour_distance', 12.0)
        
        # Return-to-path parameters
        self.declare_parameter('path_return_timeout', 10.0)
        self.declare_parameter('path_return_angle_tolerance', 30.0)  # degrees
        self.declare_parameter('path_return_distance_tolerance', 5.0)

        # Get params
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
        
        # Return-to-path
        self.path_return_timeout = self.get_parameter('path_return_timeout').value
        self.path_return_angle_tolerance = self.get_parameter('path_return_angle_tolerance').value
        self.path_return_distance_tolerance = self.get_parameter('path_return_distance_tolerance').value

        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        self.waypoints = [] 
        self.current_wp_index = 0
        self.state = "WAITING_FOR_PATH" 
        
        # PID
        self.previous_error = 0.0
        self.integral_error = 0.0
        
        # Obstacle
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.obstacle_detected = False
        self.avoidance_mode = False
        self.reverse_start_time = None
        # NEW: Avoidance Commitment State
        self.avoidance_commit_time = None 
        self.avoidance_direction = None
        
        # Stuck Detection
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
        
        # Return-to-path tracking
        self.return_to_path_start_pos = None
        self.return_to_path_start_time = None
        self.is_returning_to_path = False
        
        # Statistics & Time
        self.total_distance = 0.0
        self.start_time = None
        self._last_log_time = self.get_clock().now()
        
        # Mission control
        self.mission_enabled = False  # Wait for start command

        # --- SUBSCRIBERS ---
        self.create_subscription(Path, '/atlantis/path', self.path_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, rclpy.qos.qos_profile_sensor_data)
        
        # Dashboard control subscribers
        self.create_subscription(Empty, '/atlantis/start', self.start_callback, 10)
        self.create_subscription(Empty, '/atlantis/stop', self.stop_callback, 10)

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
        new_waypoints = []
        for pose in msg.poses:
            new_waypoints.append((pose.pose.position.x, pose.pose.position.y))
        
        if len(new_waypoints) == 0:
            # Only stop if we had waypoints before (mission abort)
            # Ignore empty paths during startup
            if len(self.waypoints) > 0:
                self.stop_boat()
                self.waypoints = []
                self.state = "STOPPED"
                self.get_logger().warn("Received EMPTY path - STOPPING BOAT.")
            return

        if new_waypoints != self.waypoints:
            self.waypoints = new_waypoints
            self.current_wp_index = 0
            self.get_logger().info(f"Received new plan with {len(self.waypoints)} waypoints")
            # Auto-enable mission if GPS is ready and we have waypoints
            if self.start_gps is not None:
                self.mission_enabled = True
                self.get_logger().info("✅ Mission AUTO-ENABLED (GPS ready + path received)")
            if self.start_gps is not None and self.mission_enabled:
                self.state = "DRIVING"
                self.start_time = self.get_clock().now()

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"Home Position Set: {self.start_gps}")
            # Auto-enable mission if we already have waypoints
            if len(self.waypoints) > 0:
                self.mission_enabled = True
                self.get_logger().info("✅ Mission AUTO-ENABLED (path ready + GPS acquired)")
            if len(self.waypoints) > 0 and self.mission_enabled:
                self.state = "DRIVING"
                self.start_time = self.get_clock().now()

    def start_callback(self, msg):
        """Enable mission from dashboard"""
        self.mission_enabled = True
        if self.start_gps is not None and len(self.waypoints) > 0:
            self.state = "DRIVING"
            self.start_time = self.get_clock().now()
        self.get_logger().info("🚀 Mission ENABLED from dashboard")

    def stop_callback(self, msg):
        """Disable mission from dashboard"""
        self.mission_enabled = False
        self.state = "PAUSED"
        self.stop_boat()
        self.get_logger().info("⏹ Mission PAUSED from dashboard")

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        points = []
        point_step = msg.point_step
        data = msg.data
        
        for i in range(0, len(data) - point_step, point_step * 10):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                if math.isnan(x) or math.isinf(x): continue
                if z < -0.2 or z > 3.0: continue
                dist = math.sqrt(x*x + y*y)
                if dist > 100.0: continue

                # IMPROVED: More sensitive obstacle detection
                # Accept obstacles that are:
                # 1. In front or slightly to sides (forward 120-degree cone)
                # 2. Within detection range (0.3 to 50m)
                # 3. Not below water (z >= -0.2)
                
                # NEW: Detect obstacles starting from 0.3m instead of 1.0m
                if dist < 0.3: continue  # Still filter very close points (noise)
                
                # IMPROVED: Wider forward detection cone (120 degrees)
                # This catches obstacles approaching from more angles
                is_forward_cone = x > -5.0  # Accept wider angle including some backwards
                is_not_too_far_left = y > -20.0   # Wider left detection
                is_not_too_far_right = y < 20.0   # Wider right detection
                
                if not (is_forward_cone and is_not_too_far_left and is_not_too_far_right):
                    continue

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
        
        if self.obstacle_detected:
            exit_threshold = self.min_safe_distance + self.hysteresis_distance
            self.obstacle_detected = self.min_obstacle_distance < exit_threshold
        else:
            self.obstacle_detected = self.min_obstacle_distance < self.min_safe_distance

        self.analyze_scan_sectors_3d(points)

    def analyze_scan_sectors_3d(self, points):
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
        return y, x

    def control_loop(self):
        # Check mission_enabled first
        if not self.mission_enabled:
            if self.state == "DRIVING":
                self.state = "PAUSED"
            return
            
        if self.state not in ["DRIVING"] or self.current_gps is None or not self.waypoints:
            if self.state == "STOPPED": self.stop_boat()
            return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        
        if self.last_position is None:
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

        self.check_stuck_condition(curr_x, curr_y)

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

        # --- OBSTACLE & AVOIDANCE LOGIC ---
        
        # 1. Stuck Escape (Highest Priority)
        if self.is_stuck and self.escape_mode:
            self.execute_escape_maneuver()
            return

        # 2. Critical Reverse (Panic)
        if self.min_obstacle_distance < self.critical_distance:
            if self.reverse_start_time is None:
                self.reverse_start_time = self.get_clock().now()
                self.integral_error = 0.0
                self.get_logger().warn(f"CRITICAL OBSTACLE {self.min_obstacle_distance:.2f}m - Reversing!")
            
            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                self.reverse_start_time = None
            else:
                self.send_thrust(-800.0, -800.0)
                self.avoidance_mode = True
                return
        else:
            self.reverse_start_time = None

        # 3. Standard Avoidance with COMMITMENT
        now = self.get_clock().now()
        
        # Check if we are currently committed to a turn
        is_committed = False
        if self.avoidance_commit_time is not None:
             commit_elapsed = (now - self.avoidance_commit_time).nanoseconds / 1e9
             if commit_elapsed < 1.5: # Commit to turn for 1.5 seconds minimum
                 is_committed = True
             else:
                 self.avoidance_commit_time = None
                 self.avoidance_direction = None

        if self.obstacle_detected or is_committed:
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
            self.avoidance_mode = True

            # Decide direction (or use committed direction)
            if is_committed and self.avoidance_direction:
                direction = self.avoidance_direction
            else:
                # New decision
                if self.left_clear > self.right_clear:
                    direction = "LEFT"
                else:
                    direction = "RIGHT"
                
                # Start commitment
                self.avoidance_commit_time = now
                self.avoidance_direction = direction

            # Execute Turn
            if direction == "LEFT":
                avoidance_heading = self.current_yaw + 1.57
            else:
                avoidance_heading = self.current_yaw - 1.57
            
            angle_error = avoidance_heading - self.current_yaw
            while angle_error > math.pi: angle_error -= 2.0 * math.pi
            while angle_error < -math.pi: angle_error += 2.0 * math.pi
            
            self.get_logger().warn(f"OBSTACLE! Turning {direction} (Locked)", throttle_duration_sec=1.0)
        else:
            # CLEAR PATH
            if self.avoidance_mode:
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

        # Speed
        speed = self.base_speed
        angle_deg = abs(math.degrees(angle_error))
        if angle_deg > 45: speed *= 0.5
        elif angle_deg > 20: speed *= 0.75
        
        if dist < 5.0: speed *= 0.7
        if self.obstacle_detected: speed *= self.obstacle_slow_factor
        
        left_thrust = max(-1000.0, min(1000.0, speed - turn_power))
        right_thrust = max(-1000.0, min(1000.0, speed + turn_power))
        
        # NEW: Check if we can return to original path after avoidance
        if self.avoidance_mode:
            self.check_return_to_path(curr_x, curr_y, (target_x, target_y))
        
        self.send_thrust(left_thrust, right_thrust)
        self.publish_dashboard_status(curr_x, curr_y, target_x, target_y, dist)
        
        # Logging
        if (now - self._last_log_time).nanoseconds / 1e9 > 2.0:
            self.log_status(curr_x, curr_y, target_x, target_y, dist, angle_error)
            self._last_log_time = now

    # --- HELPER METHODS ---
    def check_stuck_condition(self, curr_x, curr_y):
        if self.escape_mode: return
        dx = curr_x - self.last_position[0]
        dy = curr_y - self.last_position[1]
        distance_moved = math.hypot(dx, dy)
        self.position_history.append((curr_x, curr_y, self.get_clock().now()))
        if len(self.position_history) > 100: self.position_history.pop(0)
        self.estimate_drift()
        
        elapsed = (self.get_clock().now() - self.stuck_check_time).nanoseconds / 1e9
        if elapsed >= self.stuck_timeout:
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
                    
                    if self.last_stuck_waypoint == self.current_wp_index: self.consecutive_stuck_count += 1
                    else:
                        self.consecutive_stuck_count = 1
                        self.last_stuck_waypoint = self.current_wp_index
                        
                    self.calculate_adaptive_escape_duration()
                    self.get_logger().warn(f"STUCK DETECTED! Escaping (Attempt {self.consecutive_stuck_count})")
                    
                    if self.consecutive_stuck_count == 2 and not self.detour_waypoint_inserted:
                        self.insert_detour_waypoint(curr_x, curr_y)
                    if self.consecutive_stuck_count >= 4:
                        self.get_logger().error("Stuck too many times - Skipping Waypoint!")
                        self.current_wp_index += 1
                        self.consecutive_stuck_count = 0
                        self.detour_waypoint_inserted = False
                        self.is_stuck = False
                        self.escape_mode = False
            else:
                if self.is_stuck:
                    self.record_escape_result(success=True)
                    self.get_logger().info("Escape successful")
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
                if bias == 'LEFT': self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
                else: self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
            return
        elif elapsed < forward_end:
            self.escape_phase = 3
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            if self.is_heading_toward_no_go_zone(curr_x, curr_y): self.send_thrust(-400.0, 400.0)
            else: self.send_thrust(400.0 + drift_comp_left, 400.0 + drift_comp_right)
            return
        else:
            self.escape_mode = False
            self.is_stuck = False
            self.escape_phase = 0
            self.integral_error = 0.0
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

    def calculate_adaptive_escape_duration(self):
        base_duration = 10.0
        if self.min_obstacle_distance < self.critical_distance: base_duration += 4.0
        elif self.min_obstacle_distance < self.min_safe_distance: base_duration += 2.0
        base_duration += self.consecutive_stuck_count * 2.0
        self.adaptive_escape_duration = min(20.0, base_duration)
        
    def add_no_go_zone(self, x, y):
        for zone in self.no_go_zones:
            if math.hypot(x - zone[0], y - zone[1]) < self.no_go_zone_radius: return
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
        if left > right + 3.0 and left > back: return 'LEFT'
        elif right > left + 3.0 and right > back: return 'RIGHT'
        elif back > max(left, right) + 3.0: return 'BACK'
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
        self.escape_history.append({'position': self.last_escape_position, 'direction': self.best_escape_direction, 'success': success})
        if len(self.escape_history) > 50: self.escape_history.pop(0)

    def get_learned_escape_direction(self):
        if not self.escape_history: return 'LEFT'
        left_score = sum(1 for r in self.escape_history if r['direction'] == 'LEFT' and r['success'])
        right_score = sum(1 for r in self.escape_history if r['direction'] == 'RIGHT' and r['success'])
        return 'LEFT' if left_score >= right_score else 'RIGHT'

    def calculate_path_line_distance(self, point_x, point_y, line_start, line_end):
        """
        Calculate perpendicular distance from point to line between two waypoints
        
        Args:
            point_x, point_y: Point to measure from
            line_start, line_end: Waypoint tuples (x, y)
            
        Returns:
            Distance to line (positive = perpendicular distance)
        """
        x1, y1 = line_start
        x2, y2 = line_end
        
        # Line vector
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            return math.sqrt((point_x - x1)**2 + (point_y - y1)**2)
        
        # Perpendicular distance formula
        numerator = abs(dy * point_x - dx * point_y + x2 * y1 - y2 * x1)
        denominator = math.sqrt(dx**2 + dy**2)
        
        return numerator / denominator if denominator > 0 else float('inf')
    
    def is_path_clear_to_waypoint(self, curr_x, curr_y, target_wp, min_clearance=10.0):
        """
        Check if the direct path to a waypoint is obstacle-free
        
        Args:
            curr_x, curr_y: Current position
            target_wp: Target waypoint (x, y) tuple
            min_clearance: Minimum distance from obstacles
            
        Returns:
            True if path is clear
        """
        if self.min_obstacle_distance < min_clearance:
            return False
        
        target_x, target_y = target_wp
        dist_to_target = math.hypot(target_x - curr_x, target_y - curr_y)
        
        return dist_to_target > 2.0  # Must be moving toward target
    
    def check_return_to_path(self, curr_x, curr_y, target_wp):
        """
        Detect when avoidance maneuver is complete and boat can return to original path
        
        Args:
            curr_x, curr_y: Current position
            target_wp: Original target waypoint
        """
        if not self.avoidance_mode:
            return
        
        if self.return_to_path_start_pos is None:
            self.return_to_path_start_pos = (curr_x, curr_y)
            self.return_to_path_start_time = self.get_clock().now()
            return
        
        # Check if path is clear enough to return
        if self.is_path_clear_to_waypoint(curr_x, curr_y, target_wp, 
                                          min_clearance=self.get_parameter('min_safe_distance').value):
            self.avoidance_mode = False
            self.avoidance_commit_time = None
            self.avoidance_direction = None
            self.return_to_path_start_pos = None
            self.is_returning_to_path = False
            self.get_logger().info("✓ Path clear! Returning to original waypoint...")
        
        # Timeout: give up avoidance after too long
        elapsed = (self.get_clock().now() - self.return_to_path_start_time).nanoseconds / 1e9
        if elapsed > self.path_return_timeout:
            self.get_logger().warn(f"Return-to-path timeout after {elapsed:.1f}s - Continuing to next waypoint")
            self.avoidance_mode = False
            self.avoidance_commit_time = None
            self.avoidance_direction = None
            self.return_to_path_start_pos = None

    def send_thrust(self, left, right):
        # Safety check: ensure rclpy is okay
        if rclpy.ok():
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
                    "PAUSED" if self.state == "PAUSED" else \
                    "MOVING_TO_WAYPOINT" if self.state == "DRIVING" else self.state
        msg = String()
        msg.data = json.dumps({
            "state": state_key,
            "waypoint": self.current_wp_index + 1,
            "total_waypoints": len(self.waypoints),
            "distance_to_waypoint": round(dist, 1),
            "local_x": round(curr_x, 2),
            "local_y": round(curr_y, 2),
            "yaw_deg": round(math.degrees(self.current_yaw), 1),
            "mission_enabled": self.mission_enabled
        })
        self.pub_mission_status.publish(msg)
        status_text = f"OBSTACLE {round(self.min_obstacle_distance, 1)}m" if self.min_obstacle_distance < self.min_safe_distance else "CLEAR"
        obs_msg = String()
        obs_msg.data = json.dumps({"status": status_text, "min_distance": round(self.min_obstacle_distance, 1) if self.min_obstacle_distance != float('inf') else 999.9, "front_clear": self.front_clear != float('inf'), "left_clear": self.left_clear > self.min_safe_distance, "right_clear": self.right_clear > self.min_safe_distance})
        self.pub_obstacle_status.publish(obs_msg)
        
    def publish_anti_stuck_status(self):
        msg = String()
        msg.data = json.dumps({'is_stuck': self.is_stuck, 'escape_mode': self.escape_mode, 'consecutive_attempts': self.consecutive_stuck_count, 'adaptive_duration': round(self.adaptive_escape_duration, 1)})
        self.pub_anti_stuck.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("SHUTDOWN: Stopping boat...")
        
        # Send stop command explicitly
        node.stop_boat()
        
        # Small sleep to ensure message goes out before context dies
        time.sleep(0.1) 
        
    finally:
        # Don't try to use node methods here that require active context if it's already dead
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()