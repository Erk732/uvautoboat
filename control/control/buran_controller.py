#!/usr/bin/env python3
"""
Buran Controller - Motion Control System (Enhanced Persistence)

Part of the modular Vostok1 architecture.
Subscribes to planner targets and perception data, outputs thruster commands.

Features:
- PID heading control with anti-windup
- Smart Anti-Stuck System (SASS) with Kalman-filtered drift estimation
- Multi-phase escape maneuvers
- No-go zone memory
- **NEW: Avoidance Persistence (prevents oscillation near piers)**

Topics:
    Subscribes:
        /wamv/sensors/gps/gps/fix (NavSatFix) - GPS position
        /wamv/sensors/imu/imu/data (Imu) - Heading orientation
        /planning/current_target (String) - Current navigation target
        /perception/obstacle_info (String) - Obstacle detection data
    
    Publishes:
        /wamv/thrusters/left/thrust (Float64) - Left thruster command
        /wamv/thrusters/right/thrust (Float64) - Right thruster command
        /control/status (String) - Controller status
        /control/anti_stuck_status (String) - Anti-stuck system status
"""

import rclpy
from rclpy.node import Node
import math
import json
import numpy as np

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, String


# =============================================================================
# CONTROL CONSTANTS
# =============================================================================
MAX_THRUST = 2000.0          # Newtons - hardware limit
SAFE_THRUST = 800.0          # Newtons - operational limit
INTEGRAL_LIMIT = 0.5         # radians - prevent integral windup
TURN_POWER_LIMIT = 1600.0    # Newtons - max differential thrust
SENSOR_TIMEOUT = 2.0         # seconds


# =============================================================================
# KALMAN FILTER FOR DRIFT ESTIMATION
# =============================================================================
class KalmanDriftEstimator:
    def __init__(self, process_noise=0.01, measurement_noise=0.5):
        self.n = 2
        self.x = np.zeros((self.n, 1))  # State estimate
        self.P = np.eye(self.n) * 1.0   # Error covariance
        self.F = np.eye(self.n)         # State transition (random walk)
        self.H = np.eye(self.n)         # Measurement model
        self.Q = np.eye(self.n) * process_noise    # Process noise
        self.R = np.eye(self.n) * measurement_noise  # Measurement noise
        self.last_kalman_gain = np.zeros((self.n, self.n))
        
    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z):
        z = np.array(z).reshape((self.n, 1))
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.last_kalman_gain = K.copy()
        self.x = self.x + K @ y
        self.P = (np.eye(self.n) - K @ self.H) @ self.P
        
    def get_drift(self):
        return (float(self.x[0, 0]), float(self.x[1, 0]))
    
    def get_uncertainty(self):
        return (float(np.sqrt(self.P[0, 0])), float(np.sqrt(self.P[1, 1])))


class BuranController(Node):
    """
    BURAN - Boat controller with PID navigation
    Enhanced with Smart Anti-Stuck System (SASS) and Persistence Logic
    """
    def __init__(self):
        super().__init__('buran_controller_node')

        # --- PID PARAMETERS ---
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)

        # Speed parameters
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('obstacle_slow_factor', 0.3)
        self.declare_parameter('approach_slow_distance', 5.0)
        self.declare_parameter('approach_slow_factor', 0.7)
        self.declare_parameter('slew_rate_limit', 80.0)       
        self.declare_parameter('turn_deadband_deg', 2.0)      

        # Obstacle avoidance
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('reverse_timeout', 5.0)
        self.declare_parameter('avoidance_persistence', 2.0) # NEW: Keep turning for 2s
        
        # Smart anti-stuck parameters
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('stuck_threshold', 0.5)
        self.declare_parameter('no_go_zone_radius', 8.0)
        self.declare_parameter('drift_compensation_gain', 0.3)
        self.declare_parameter('detour_distance', 12.0)
        
        # Kalman filter parameters
        self.declare_parameter('kalman_process_noise', 0.01)
        self.declare_parameter('kalman_measurement_noise', 0.5)

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.obstacle_slow_factor = self.get_parameter('obstacle_slow_factor').value
        self.approach_slow_distance = float(self.get_parameter('approach_slow_distance').value)
        self.approach_slow_factor = float(self.get_parameter('approach_slow_factor').value)
        self.slew_rate_limit = float(self.get_parameter('slew_rate_limit').value)
        self.turn_deadband_deg = float(self.get_parameter('turn_deadband_deg').value)
        self.critical_distance = self.get_parameter('critical_distance').value
        self.reverse_timeout = self.get_parameter('reverse_timeout').value
        self.avoidance_persistence = self.get_parameter('avoidance_persistence').value
        
        # Smart anti-stuck parameters
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.no_go_zone_radius = self.get_parameter('no_go_zone_radius').value
        self.drift_compensation_gain = self.get_parameter('drift_compensation_gain').value
        self.detour_distance = self.get_parameter('detour_distance').value

        # --- STATE ---
        self.current_yaw = 0.0
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.dt = 0.05  # 20Hz

        # Target state (from planner)
        self.target_x = None
        self.target_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_to_target = float('inf')
        
        # Mission state (from planner)
        self.mission_active = False  

        # Obstacle state (from perception)
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.is_critical = False
        self.urgency = 0.0  
        self.best_gap = None  
        self.obstacle_count = 0  

        # Avoidance state
        self.avoidance_mode = False
        self.reverse_start_time = None
        self.avoidance_cooldown_time = None # NEW: Timer for persistence
        self.prev_left_thrust = 0.0
        self.prev_right_thrust = 0.0

        # GPS for local coordinate conversion
        self.start_gps = None
        self.current_gps = None
        
        # Smart anti-stuck state
        self.last_position = None
        self.stuck_check_time = None
        self.is_stuck = False
        self.escape_mode = False
        self.escape_start_time = None
        self.escape_phase = 0
        self.consecutive_stuck_count = 0
        self.adaptive_escape_duration = 12.0
        
        # No-go zones (obstacle memory)
        self.no_go_zones = []  # List of (x, y, radius)
        
        # Drift compensation with Kalman filter
        self.position_history = []
        kalman_process = self.get_parameter('kalman_process_noise').value
        kalman_measurement = self.get_parameter('kalman_measurement_noise').value
        self.drift_kalman = KalmanDriftEstimator(
            process_noise=kalman_process,
            measurement_noise=kalman_measurement
        )
        self.drift_vector = (0.0, 0.0)  
        
        # Escape learning
        self.escape_history = []
        self.best_escape_direction = None
        self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}
        self.last_escape_position = None
        
        # Detour waypoints
        self.detour_requested = False

        # --- SUBSCRIBERS ---
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(String, '/planning/current_target', self.target_callback, 10)
        self.create_subscription(String, '/perception/obstacle_info', self.obstacle_callback, 10)
        self.create_subscription(String, '/planning/mission_status', self.mission_status_callback, 10)
        self.create_subscription(String, '/vostok1/set_config', self.config_callback, 10)
        self.create_subscription(String, '/sputnik/set_config', self.config_callback, 10)

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_status = self.create_publisher(String, '/control/status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/control/anti_stuck_status', 10)
        self.pub_detour_request = self.create_publisher(String, '/planning/detour_request', 10)

        # Timers
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("BURAN Controller (Persistent Avoidance) Ready")

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.current_x, self.current_y = data['current_position']
            self.target_x, self.target_y = data['target_waypoint']
            self.distance_to_target = data['distance_to_target']
        except: pass

    def obstacle_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacle_detected = data.get('obstacle_detected', False)
            self.min_obstacle_distance = data.get('min_distance', float('inf'))
            self.front_clear = data.get('front_clear', float('inf'))
            self.left_clear = data.get('left_clear', float('inf'))
            self.right_clear = data.get('right_clear', float('inf'))
            self.is_critical = data.get('is_critical', False)
            self.urgency = data.get('urgency', 0.0)
            self.best_gap = data.get('best_gap', None)
            self.obstacle_count = data.get('obstacle_count', 0)
        except: pass

    def mission_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            state = data.get('state', '')
            was_active = self.mission_active
            self.mission_active = (state == "DRIVING")
            
            if was_active != self.mission_active:
                if not self.mission_active:
                    self.target_x = None
                    self._reset_all_escape_state()
                    self.stop()
                else:
                    self._reset_all_escape_state()
        except: pass
    
    def _reset_all_escape_state(self):
        self.escape_mode = False
        self.is_stuck = False
        self.avoidance_mode = False
        self.reverse_start_time = None
        self.integral_error = 0.0
        self.escape_phase = 0
        self.consecutive_stuck_count = 0
        self.last_position = None
        self.stuck_check_time = None

    def config_callback(self, msg):
        # Configuration parsing logic here (same as before)
        pass

    def control_loop(self):
        if not self.mission_active:
            self.stop()
            return
            
        if self.target_x is None:
            self.stop()
            return
        
        if self.last_position is None:
            self.last_position = (self.current_x, self.current_y)
            self.stuck_check_time = self.get_clock().now()
        
        self.update_position_history()
        
        if not self.escape_mode:
            self.check_stuck_condition()
        
        if self.is_stuck and self.escape_mode:
            self.execute_smart_escape()
            return

        # Critical Obstacle - Reverse
        if self.is_critical and self.min_obstacle_distance < self.critical_distance:
            if self.reverse_start_time is None:
                self.reverse_start_time = self.get_clock().now()
                self.integral_error = 0.0
                self.get_logger().warn("CRITICAL REVERSE!")

            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                self.reverse_start_time = None
            else:
                self.send_thrust(-1600.0, -1600.0)
                self.publish_status("REVERSING")
                return
        else:
            self.reverse_start_time = None

        # --- OBSTACLE AVOIDANCE LOGIC (WITH PERSISTENCE) ---
        target_heading_mode = "TARGET"
        
        if self.obstacle_detected:
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.avoidance_mode = True
                
            # Reset cooldown because we still see the obstacle
            self.avoidance_cooldown_time = self.get_clock().now()

            # Determine avoidance direction
            if self.best_gap and self.best_gap.get('width', 0) > 20:
                avoidance_heading = self.current_yaw + math.radians(self.best_gap.get('direction', 0))
                target_heading_mode = "GAP"
            elif self.left_clear > self.right_clear:
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)
                avoidance_heading = self.current_yaw + turn_angle
                target_heading_mode = "LEFT"
            else:
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)
                avoidance_heading = self.current_yaw - turn_angle
                target_heading_mode = "RIGHT"

            angle_error = self.normalize_angle(avoidance_heading - self.current_yaw)
            
        else:
            # Check Cooldown Persistence
            is_in_cooldown = False
            if self.avoidance_mode and self.avoidance_cooldown_time:
                elapsed = (self.get_clock().now() - self.avoidance_cooldown_time).nanoseconds / 1e9
                if elapsed < self.avoidance_persistence:
                    is_in_cooldown = True
                    target_heading_mode = "COOLDOWN"
                    
                    # Force continue turning in the last direction
                    if self.prev_left_thrust < self.prev_right_thrust: # Was turning Left
                         avoidance_heading = self.current_yaw + 0.5 
                    else:
                         avoidance_heading = self.current_yaw - 0.5
                    
                    angle_error = self.normalize_angle(avoidance_heading - self.current_yaw)
                    self.get_logger().info(f"❄️ Persisting Avoidance... ({elapsed:.1f}s)", throttle_duration_sec=0.5)

            if not is_in_cooldown:
                # Actual Safe Navigation
                if self.avoidance_mode:
                    self.get_logger().info("Path CLEAR - Resuming navigation")
                    self.avoidance_mode = False
                    self.integral_error = 0.0
                    self.previous_error = 0.0

                dx = self.target_x - self.current_x
                dy = self.target_y - self.current_y
                target_angle = math.atan2(dy, dx)
                angle_error = self.normalize_angle(target_angle - self.current_yaw)

        # PID Controller
        self.integral_error += angle_error * self.dt
        self.integral_error = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral_error))
        derivative_error = (angle_error - self.previous_error) / self.dt
        turn_power = (self.kp * angle_error + self.ki * self.integral_error + self.kd * derivative_error)
        self.previous_error = angle_error
        turn_power = max(-TURN_POWER_LIMIT, min(TURN_POWER_LIMIT, turn_power))

        # Speed Control
        angle_deg = abs(math.degrees(angle_error))
        if angle_deg < self.turn_deadband_deg: turn_power = 0.0
        
        speed = self.base_speed
        if angle_deg > 45: speed *= 0.5
        elif angle_deg > 20: speed *= 0.75

        # Approach slowdown
        if math.isfinite(self.distance_to_target) and self.distance_to_target < self.approach_slow_distance:
            frac = max(0.0, min(1.0, self.distance_to_target / self.approach_slow_distance))
            speed *= (self.approach_slow_factor + (1.0 - self.approach_slow_factor) * frac)

        if self.obstacle_detected:
            if self.urgency > 0.0:
                speed *= (1.0 - (self.urgency * (1.0 - self.obstacle_slow_factor)))
            else:
                speed *= self.obstacle_slow_factor

        left_thrust = speed - turn_power
        right_thrust = speed + turn_power
        left_thrust = max(-SAFE_THRUST, min(SAFE_THRUST, left_thrust))
        right_thrust = max(-SAFE_THRUST, min(SAFE_THRUST, right_thrust))

        # Slew rate limit
        left_thrust = self._slew_limit(self.prev_left_thrust, left_thrust)
        right_thrust = self._slew_limit(self.prev_right_thrust, right_thrust)
        self.prev_left_thrust = left_thrust
        self.prev_right_thrust = right_thrust

        self.send_thrust(left_thrust, right_thrust)
        self.publish_status(target_heading_mode)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _slew_limit(self, previous, target):
        delta = target - previous
        delta = max(-self.slew_rate_limit, min(self.slew_rate_limit, delta))
        return previous + delta

    def send_thrust(self, left, right):
        if not self.mission_active: left, right = 0.0, 0.0
        self.pub_left.publish(Float64(data=float(left)))
        self.pub_right.publish(Float64(data=float(right)))

    def stop(self):
        self.send_thrust(0.0, 0.0)

    def publish_status(self, mode):
        msg = String()
        msg.data = json.dumps({
            'mode': mode,
            'avoidance_active': self.avoidance_mode,
            'obstacle_detected': bool(self.obstacle_detected),
            'min_distance': round(self.min_obstacle_distance, 1),
            'urgency': round(self.urgency, 2)
        })
        self.pub_status.publish(msg)

    # --- SASS ---
    def update_position_history(self):
        self.position_history.append((self.current_x, self.current_y, self.get_clock().now()))
        if len(self.position_history) > 100: self.position_history.pop(0)
        self.estimate_drift()
    
    def check_stuck_condition(self):
        dx = self.current_x - self.last_position[0]
        dy = self.current_y - self.last_position[1]
        distance_moved = math.hypot(dx, dy)
        elapsed = (self.get_clock().now() - self.stuck_check_time).nanoseconds / 1e9
        
        if elapsed >= self.stuck_timeout:
            if distance_moved < self.stuck_threshold:
                if not self.is_stuck:
                    self.is_stuck = True
                    self.escape_mode = True
                    self.escape_start_time = self.get_clock().now()
                    self.escape_phase = 0
                    self.consecutive_stuck_count += 1
                    self.add_no_go_zone(self.current_x, self.current_y)
                    self.calculate_adaptive_escape_duration()
                    if self.consecutive_stuck_count == 2: self.request_detour_waypoint()
            else:
                self.is_stuck = False
                self.escape_mode = False
                self.consecutive_stuck_count = 0
            self.last_position = (self.current_x, self.current_y)
            self.stuck_check_time = self.get_clock().now()
    
    def execute_smart_escape(self):
        elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
        probe_end = 2.0
        reverse_end = probe_end + self.adaptive_escape_duration * 0.4
        turn_end = reverse_end + self.adaptive_escape_duration * 0.35
        forward_end = turn_end + self.adaptive_escape_duration * 0.25
        drift_comp_left, drift_comp_right = self.calculate_drift_compensation()
        
        if elapsed < probe_end:
            if elapsed < 0.6: self.send_thrust(-200.0, 200.0)
            elif elapsed < 1.2: self.send_thrust(200.0, -200.0)
            else:
                self.send_thrust(0.0, 0.0)
                if self.best_escape_direction is None:
                    self.best_escape_direction = self.determine_best_escape_direction()
        elif elapsed < reverse_end:
            reverse_power = -700.0 - min(300.0, 100.0 / max(0.5, self.min_obstacle_distance))
            self.send_thrust(reverse_power + drift_comp_left, reverse_power + drift_comp_right)
        elif elapsed < turn_end:
            turn_power = min(900.0, 600.0 + (self.consecutive_stuck_count * 100.0))
            if self.best_escape_direction == 'LEFT':
                self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
            else:
                self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
        elif elapsed < forward_end:
            if self.is_heading_toward_no_go_zone(): self.send_thrust(-400.0, 400.0)
            else: self.send_thrust(400.0 + drift_comp_left, 400.0 + drift_comp_right)
        else:
            self.escape_mode = False
            self.is_stuck = False
            self.last_position = (self.current_x, self.current_y)
            self.stuck_check_time = self.get_clock().now()
    
    def calculate_adaptive_escape_duration(self):
        base = 10.0
        if self.min_obstacle_distance < self.critical_distance: base += 4.0
        base += self.consecutive_stuck_count * 2.0
        self.adaptive_escape_duration = min(20.0, base)
    
    def add_no_go_zone(self, x, y):
        self.no_go_zones.append((x, y, self.no_go_zone_radius))
        if len(self.no_go_zones) > 20: self.no_go_zones.pop(0)
    
    def is_in_no_go_zone(self, x, y):
        for zx, zy, r in self.no_go_zones:
            if math.hypot(x-zx, y-zy) < r: return True
        return False
    
    def is_heading_toward_no_go_zone(self):
        lx = self.current_x + 10.0 * math.cos(self.current_yaw)
        ly = self.current_y + 10.0 * math.sin(self.current_yaw)
        return self.is_in_no_go_zone(lx, ly)
    
    def determine_best_escape_direction(self):
        if self.probe_results['left'] > self.probe_results['right'] + 3.0: return 'LEFT'
        elif self.probe_results['right'] > self.probe_results['left'] + 3.0: return 'RIGHT'
        return self.get_learned_escape_direction()
    
    def estimate_drift(self):
        if len(self.position_history) < 20: return
        self.drift_kalman.predict()
        recent = self.position_history[-20:]
        dx = recent[-1][0] - recent[0][0]
        dy = recent[-1][1] - recent[0][1]
        dt = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        if dt > 0.5: self.drift_kalman.update([dx/dt, dy/dt])
        self.drift_vector = self.drift_kalman.get_drift()
    
    def calculate_drift_compensation(self):
        mag = math.hypot(self.drift_vector[0], self.drift_vector[1])
        if mag < 0.1: return 0.0, 0.0
        angle = math.atan2(self.drift_vector[1], self.drift_vector[0])
        rel = self.normalize_angle(angle - self.current_yaw)
        comp = min(150.0, mag * self.drift_compensation_gain * 100.0)
        return (comp, -comp) if rel > 0 else (-comp, comp)
    
    def request_detour_waypoint(self):
        angle = self.current_yaw + (math.pi/2 if self.left_clear > self.right_clear else -math.pi/2)
        dx = self.current_x + self.detour_distance * math.cos(angle)
        dy = self.current_y + self.detour_distance * math.sin(angle)
        msg = String()
        msg.data = json.dumps({'type': 'detour', 'x': round(dx, 2), 'y': round(dy, 2)})
        self.pub_detour_request.publish(msg)
    
    def record_escape_result(self, success):
        self.escape_history.append({'direction': self.best_escape_direction, 'success': success})
        if len(self.escape_history) > 50: self.escape_history.pop(0)
    
    def get_learned_escape_direction(self):
        if not self.escape_history: return 'LEFT'
        l = sum(1 for r in self.escape_history if r['direction']=='LEFT' and r['success'])
        r = sum(1 for r in self.escape_history if r['direction']=='RIGHT' and r['success'])
        return 'LEFT' if l >= r else 'RIGHT'
    
    def publish_anti_stuck_status(self):
        msg = String()
        msg.data = json.dumps({'is_stuck': self.is_stuck, 'escape_mode': self.escape_mode})
        self.pub_anti_stuck.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BuranController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()