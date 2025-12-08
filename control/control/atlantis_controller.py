import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import Float64, String, Empty
import math
import json
import time

# Relative Import
from .lidar_obstacle_avoidance import LidarObstacleDetector

class AtlantisController(Node):
    def __init__(self):
        super().__init__('atlantis_controller')

        # --- PARAMETERS ---
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 3.0)
        self.declare_parameter('kp', 450.0) # Increased slightly for better heading hold
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 150.0) # Increased for more responsive turning
        
        # --- OBSTACLE PARAMETERS ---
        self.declare_parameter('min_safe_distance', 6.0)
        self.declare_parameter('critical_distance', 3.0)
        self.declare_parameter('obstacle_slow_factor', 0.2) # Speed of the boat during avoidance can be changed for faster/slower movement
        self.declare_parameter('hysteresis_distance', 2.0)   
        self.declare_parameter('reverse_timeout', 10.0)    

        # Stuck detection & Waypoint Skipping
        self.declare_parameter('stuck_timeout', 5.0)
        self.declare_parameter('stuck_threshold', 1.0)
        self.declare_parameter('waypoint_timeout', 60.0)
        self.declare_parameter('no_go_zone_radius', 8.0)
        self.declare_parameter('drift_compensation_gain', 0.3)
        self.declare_parameter('detour_distance', 12.0)
        self.declare_parameter('path_return_timeout', 10.0)
        self.declare_parameter('blocked_skip_time', 15.0) 

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
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        self.no_go_zone_radius = self.get_parameter('no_go_zone_radius').value
        self.drift_compensation_gain = self.get_parameter('drift_compensation_gain').value
        self.detour_distance = self.get_parameter('detour_distance').value
        self.path_return_timeout = self.get_parameter('path_return_timeout').value
        self.blocked_skip_time = self.get_parameter('blocked_skip_time').value

        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        self.waypoints = [] 
        self.current_wp_index = 0
        self.state = "WAITING_FOR_PATH"
        self.path_validation_ok = False
        self.current_path_version = -1
        
        # Waypoint Timing
        self.waypoint_start_time = None
        self.blocked_start_time = None 
        
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
        self.mission_enabled = False
        self.total_distance = 0.0
        self._last_log_time = self.get_clock().now()

        # --- SUBSCRIBERS ---
        self.create_subscription(Path, '/atlantis/path', self.path_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
        self.create_subscription(Empty, '/atlantis/start', self.start_callback, 10)
        self.create_subscription(Empty, '/atlantis/stop', self.stop_callback, 10)

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_mission_status = self.create_publisher(String, '/atlantis/mission_status', 10)
        self.pub_obstacle_status = self.create_publisher(String, '/atlantis/obstacle_status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/atlantis/anti_stuck_status', 10)

        # --- HELPER INIT ---
        self.lidar_detector = LidarObstacleDetector(min_distance=5.0, max_distance=100.0, z_filter_enabled=False)

        # --- TIMER ---
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(0.5, self.publish_anti_stuck_status)
        
        self.get_logger().info("Atlantis Controller Ready - Waiting for GPS and Path...")

    def path_callback(self, msg):
        if self.escape_mode or self.is_stuck: return
        try:
            new_waypoints = []
            for pose in msg.poses:
                new_waypoints.append((pose.pose.position.x, pose.pose.position.y))
            if len(new_waypoints) < 2: return
            if new_waypoints != self.waypoints:
                self.waypoints = new_waypoints
                self.current_wp_index = 0
                self.path_validation_ok = True
                self.current_path_version += 1
                self.waypoint_start_time = self.get_clock().now()
                self.blocked_start_time = None
                if self.start_gps is not None:
                    self.mission_enabled = True
                    self.state = "DRIVING"
        except Exception as e: self.get_logger().error(f"Path callback error: {e}")

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            if len(self.waypoints) > 0:
                self.mission_enabled = True
                self.state = "DRIVING"

    def start_callback(self, msg):
        self.mission_enabled = True
        if self.start_gps is not None and len(self.waypoints) > 0: self.state = "DRIVING"

    def stop_callback(self, msg):
        self.mission_enabled = False
        self.state = "PAUSED"
        self.stop_boat()

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        detected_obstacles = self.lidar_detector.process_pointcloud(msg.data, msg.point_step, sampling_factor=10)
        points = []
        for obs in detected_obstacles:
            is_forward = obs.x > -2.0 and obs.x < 25.0 
            is_width = abs(obs.y) < 15.0 
            if is_forward and is_width:
                points.append((obs.x, obs.y, obs.z, obs.distance))

        if points:
            distances = [p[3] for p in points]
            self.min_obstacle_distance = min(distances)
            self.analyze_scan_sectors_3d(points)
            self.obstacle_detected = self.min_obstacle_distance < self.min_safe_distance
        else:
            self.min_obstacle_distance = 100.0
            self.obstacle_detected = False
            self.front_clear = 100.0; self.left_clear = 100.0; self.right_clear = 100.0

    def analyze_scan_sectors_3d(self, points):
        front_points = []
        left_points = []
        right_points = []
        for x, y, z, dist in points:
            angle = math.atan2(y, x)
            if -0.7 < angle < 0.7: front_points.append(dist)
            elif angle >= 0.7: left_points.append(dist)
            elif angle <= -0.7: right_points.append(dist)
        
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

    def check_waypoint_blocked(self, angle_to_target, dist_to_target):
        while angle_to_target > math.pi: angle_to_target -= 2.0 * math.pi
        while angle_to_target < -math.pi: angle_to_target += 2.0 * math.pi
        
        check_dist = dist_to_target - 2.0 
        if check_dist < 1.0: return False 
        
        sensor_limit = 95.0 
        sector_clearance = 100.0
        
        if -0.7 < angle_to_target < 0.7: sector_clearance = self.front_clear
        elif angle_to_target >= 0.7: sector_clearance = self.left_clear
        elif angle_to_target <= -0.7: sector_clearance = self.right_clear
            
        if sector_clearance < sensor_limit and sector_clearance < check_dist:
            return True
        return False

    def control_loop(self):
        if not self.mission_enabled or not self.waypoints: return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        
        if self.last_position is None:
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

        self.check_stuck_condition(curr_x, curr_y)

        if self.current_wp_index >= len(self.waypoints):
            self.finish_mission(curr_x, curr_y)
            return

        now = self.get_clock().now()
        target_x, target_y = self.waypoints[self.current_wp_index]
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)
        
        # Calculate Target Angle FIRST
        target_angle_raw = math.atan2(dy, dx)
        angle_to_target = target_angle_raw - self.current_yaw
        while angle_to_target > math.pi: angle_to_target -= 2.0 * math.pi
        while angle_to_target < -math.pi: angle_to_target += 2.0 * math.pi

        # --- PATH PRIORITY LOGIC (Your Request) ---
        # 1. Is the direct path to the waypoint clear?
        is_path_blocked = self.check_waypoint_blocked(angle_to_target, dist)
        
        # 2. Check for CRITICAL collision (Emergency Stop < 3m)
        is_critical = self.min_obstacle_distance < self.critical_distance

        # 3. Decision Tree
        if is_critical:
            # SAFETY OVERRIDE: Ignore path, just don't crash
            if self.reverse_start_time is None: self.reverse_start_time = now
            if (now - self.reverse_start_time).nanoseconds/1e9 < self.reverse_timeout:
                self.send_thrust(-800.0, -800.0)
                return
            self.avoidance_mode = True
        
        elif not is_path_blocked:
            # GREEN LIGHT: Path is clear! Go to GPS even if other things are nearby.
            self.avoidance_mode = False
            self.obstacle_detected = False # Override detection flag
            self.blocked_start_time = None
            target_heading = target_angle_raw
            
        else:
            # RED LIGHT: Path is blocked. Enter Avoidance Mode.
            self.avoidance_mode = True
            if self.blocked_start_time is None: self.blocked_start_time = now
            
            # Fast Skip Logic (Pier)
            elapsed = (now - self.blocked_start_time).nanoseconds / 1e9
            threshold = 2.0 if self.min_obstacle_distance < 8.0 else self.blocked_skip_time
            
            if elapsed > threshold:
                self.get_logger().warn(f"⏭️ SKIP WP {self.current_wp_index+1}: Blocked for {elapsed:.1f}s")
                self.current_wp_index += 1
                self.blocked_start_time = None
                return

        # --- EXECUTE MOVEMENT ---
        
        # 1. Stuck Escape (Highest Priority)
        if self.is_stuck and self.escape_mode:
            self.execute_escape_maneuver()
            return

        # 2. Avoidance Steering
        if self.avoidance_mode:
            # Stick to a decision (Commit)
            if self.avoidance_commit_time is None:
                self.avoidance_commit_time = now
                self.avoidance_direction = "LEFT" if self.left_clear > self.right_clear else "RIGHT"
            
            # Reset commit after 3s
            if (now - self.avoidance_commit_time).nanoseconds/1e9 > 3.0:
                self.avoidance_commit_time = None

            turn_dir = 1.0 if self.avoidance_direction == "LEFT" else -1.0
            target_heading = self.current_yaw + (1.57 * turn_dir)
            
        else:
            # 3. GPS Steering (PID)
            target_heading = target_angle_raw
            self.avoidance_commit_time = None # Reset avoidance memory

        # PID Calculation
        angle_error = target_heading - self.current_yaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi

        self.integral_error += angle_error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error))
        derivative = (angle_error - self.previous_error) / self.dt
        turn_power = (self.kp * angle_error + self.ki * self.integral_error + self.kd * derivative)
        self.previous_error = angle_error
        
        # Limit Turn Power
        turn_power = max(-900.0, min(900.0, turn_power))

        # Speed Control
        speed = self.base_speed
        if abs(math.degrees(angle_error)) > 45: speed *= 0.5
        elif abs(math.degrees(angle_error)) > 20: speed *= 0.75
        
        if dist < 5.0: speed *= 0.7
        if self.avoidance_mode: speed *= self.obstacle_slow_factor
        
        if dist < self.waypoint_tolerance:
            self.current_wp_index += 1
            self.total_distance += dist
            self.integral_error = 0.0
            return

        self.send_thrust(speed - turn_power, speed + turn_power)
        
        if (now - self._last_log_time).nanoseconds / 1e9 > 2.0:
            mode = "AVOID" if self.avoidance_mode else "GPS"
            self.get_logger().info(f"WP {self.current_wp_index+1} | Dist:{dist:.1f}m | Mode:{mode}")
            self._last_log_time = now

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
                    self.consecutive_stuck_count += 1
                    self.calculate_adaptive_escape_duration()
                    self.get_logger().warn(f"STUCK DETECTED! Escaping (Attempt {self.consecutive_stuck_count})")
            else:
                self.is_stuck = False
                self.escape_mode = False
                self.consecutive_stuck_count = 0
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
            reverse_power = -700.0 - min(300.0, 100.0 / max(0.5, self.min_obstacle_distance))
            self.send_thrust(reverse_power + drift_comp_left, reverse_power + drift_comp_right)
            return
        elif elapsed < turn_end:
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

    def record_escape_result(self, success):
        if self.last_escape_position is None: return
        self.escape_history.append({'position': self.last_escape_position, 'direction': self.best_escape_direction, 'success': success})
        if len(self.escape_history) > 50: self.escape_history.pop(0)

    def get_learned_escape_direction(self):
        if not self.escape_history: return 'LEFT'
        left_score = sum(1 for r in self.escape_history if r['direction'] == 'LEFT' and r['success'])
        right_score = sum(1 for r in self.escape_history if r['direction'] == 'RIGHT' and r['success'])
        return 'LEFT' if left_score >= right_score else 'RIGHT'

    def send_thrust(self, left, right):
        if rclpy.ok():
            self.pub_left.publish(Float64(data=float(left)))
            self.pub_right.publish(Float64(data=float(right)))

    def stop_boat(self):
        self.send_thrust(0.0, 0.0)

    def log_status(self, curr_x, curr_y, target_x, target_y, dist, error):
        wp = f"{self.current_wp_index + 1}/{len(self.waypoints)}"
        obs = f"OBS:{self.min_obstacle_distance:.1f}m" if self.obstacle_detected else "CLEAR"
        blocked = "BLOCKED" if self.blocked_start_time else ""
        self.get_logger().info(f"WP {wp} | Pos:({curr_x:.1f},{curr_y:.1f}) | Dist:{dist:.1f}m | {obs} {blocked}")

    def finish_mission(self, final_x, final_y):
        if self.state == "FINISHED": # prints only once
            return
        self.state = "FINISHED"
        self.stop_boat()
        self.get_logger().info("MISSION COMPLETE!")

    def publish_dashboard_status(self, curr_x, curr_y, target_x, target_y, dist):
        state_key = "STUCK_ESCAPING" if self.escape_mode else \
                    "OBSTACLE_AVOIDING" if self.avoidance_mode else \
                    "MISSION_COMPLETE" if self.state == "FINISHED" else \
                    "PAUSED" if self.state == "PAUSED" else \
                    "MOVING_TO_WAYPOINT" if self.state == "DRIVING" else self.state
        msg = String()
        msg.data = json.dumps({
            "state": state_key, "waypoint": self.current_wp_index + 1,
            "total_waypoints": len(self.waypoints), "distance_to_waypoint": round(dist, 1),
            "local_x": round(curr_x, 2), "local_y": round(curr_y, 2),
            "yaw_deg": round(math.degrees(self.current_yaw), 1), "mission_enabled": self.mission_enabled
        })
        self.pub_mission_status.publish(msg)
    
    def publish_obstacle_status(self):
        msg = String()
        msg.data = json.dumps({
            "min_distance": round(self.min_obstacle_distance if self.min_obstacle_distance != float('inf') else 999.0, 2),
            "front_clear": self.front_clear >= self.min_safe_distance,
            "left_clear": self.left_clear >= self.min_safe_distance,
            "right_clear": self.right_clear >= self.min_safe_distance
        })
        self.pub_obstacle_status.publish(msg)
        
    def publish_anti_stuck_status(self):
        msg = String()
        msg.data = json.dumps({
            'is_stuck': self.is_stuck,
            'escape_mode': self.escape_mode,
            'escape_phase': self.escape_phase,
            'best_direction': self.best_escape_direction,
            'consecutive_attempts': self.consecutive_stuck_count,
            'no_go_zones': len(self.no_go_zones)
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
        node.get_logger().info("🛑 STOPPING MOTORS...")
        node.stop_boat()
        time.sleep(0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()