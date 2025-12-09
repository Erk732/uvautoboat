#!/usr/bin/env python3
"""
Buran Controller - Motion Control System (Fixed Version with A* Integration)

Part of the modular Vostok1 architecture.
Subscribes to planner targets and perception data, outputs thruster commands.

Features:
- PID heading control with anti-windup
- Smart Anti-Stuck System (SASS) with Kalman-filtered drift estimation
- Multi-phase escape maneuvers
- No-go zone memory
- Path-aware navigation (A* integration)
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
MAX_THRUST = 2000.0
SAFE_THRUST = 800.0
INTEGRAL_LIMIT = 0.5
TURN_POWER_LIMIT = 1600.0
SENSOR_TIMEOUT = 2.0


# =============================================================================
# KALMAN FILTER FOR DRIFT ESTIMATION
# =============================================================================
class KalmanDriftEstimator:
    """2D Kalman Filter for estimating water/wind drift."""
    
    def __init__(self, process_noise=0.01, measurement_noise=0.5):
        self.n = 2
        self.x = np.zeros((self.n, 1))
        self.P = np.eye(self.n) * 1.0
        self.F = np.eye(self.n)
        self.H = np.eye(self.n)
        self.Q = np.eye(self.n) * process_noise
        self.R = np.eye(self.n) * measurement_noise
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
    """BURAN - Boat controller with PID navigation and A* path awareness"""
    
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
        self.dt = 0.05

        # Target state
        self.target_x = None
        self.target_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_to_target = float('inf')
        
        # Path awareness (NEW for A* integration)
        self.following_astar_path = False
        
        # Mission state
        self.mission_active = False

        # Obstacle state
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
        self.prev_left_thrust = 0.0
        self.prev_right_thrust = 0.0

        # GPS
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
        
        # No-go zones
        self.no_go_zones = []
        
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

        # Control loop at 20Hz
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("=" * 50)
        self.get_logger().info("BURAN - Systeme de Controle de Mouvement")
        self.get_logger().info("Buran Controller - PID Heading Control")
        self.get_logger().info("+ Smart Anti-Stuck System (SASS) v2.0")
        self.get_logger().info("+ Path-Aware Navigation (A* Integration)")
        self.get_logger().info(f"PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info(f"Speed: {self.base_speed} (max: {self.max_speed})")
        self.get_logger().info("=" * 50)

    def gps_callback(self, msg):
        """Handle GPS updates"""
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)

    def imu_callback(self, msg):
        """Extract yaw from IMU quaternion"""
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg):
        """Receive current navigation target from planner"""
        try:
            data = json.loads(msg.data)
            self.current_x, self.current_y = data['current_position']
            self.target_x, self.target_y = data['target_waypoint']
            self.distance_to_target = data['distance_to_target']
            # NEW: Check if following A* path
            self.following_astar_path = data.get('astar_path', False)
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Invalid target message: {e}")

    def obstacle_callback(self, msg):
        """Receive obstacle information from perception"""
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
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Invalid obstacle message: {e}")

    def mission_status_callback(self, msg):
        """Receive mission status from planner"""
        try:
            data = json.loads(msg.data)
            state = data.get('state', '')
            was_active = self.mission_active
            self.mission_active = (state == "DRIVING")
            
            if was_active and not self.mission_active:
                self.target_x = None
                self.target_y = None
                self._reset_all_escape_state()
                self.stop()
                self.send_thrust(0.0, 0.0)
                self.get_logger().info(f"🛑 Mission inactive - stopping")
            elif not was_active and self.mission_active:
                self._reset_all_escape_state()
                self.get_logger().info(f"✅ Mission active")
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Invalid mission status: {e}")
    
    def _reset_all_escape_state(self):
        """Reset all escape/stuck/avoidance state variables"""
        self.escape_mode = False
        self.is_stuck = False
        self.avoidance_mode = False
        self.reverse_start_time = None
        self.integral_error = 0.0
        self.escape_phase = 0
        self.best_escape_direction = None
        self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}
        self.consecutive_stuck_count = 0
        self.last_position = None
        self.stuck_check_time = None

    def config_callback(self, msg):
        """Handle runtime configuration changes"""
        try:
            config = json.loads(msg.data)
            updated = []
            
            if 'kp' in config:
                self.kp = float(config['kp'])
                updated.append(f"Kp={self.kp}")
            if 'ki' in config:
                self.ki = float(config['ki'])
                updated.append(f"Ki={self.ki}")
            if 'kd' in config:
                self.kd = float(config['kd'])
                updated.append(f"Kd={self.kd}")
            if 'base_speed' in config:
                self.base_speed = float(config['base_speed'])
                updated.append(f"base_speed={self.base_speed}")
                
            if updated:
                self.get_logger().info(f"⚙️ Config updated: {', '.join(updated)}")
                
        except Exception as e:
            self.get_logger().error(f"Config parse error: {e}")

    def control_loop(self):
        """Main control loop"""
        if not self.mission_active:
            self.stop()
            self.send_thrust(0.0, 0.0)
            return
            
        if self.target_x is None or self.target_y is None:
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

        if self.is_critical and self.min_obstacle_distance < self.critical_distance:
            if self.reverse_start_time is None:
                self.reverse_start_time = self.get_clock().now()
                self.integral_error = 0.0
                self.get_logger().warn(f"CRITICAL OBSTACLE {self.min_obstacle_distance:.1f}m - Reversing!")

            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                self.reverse_start_time = None
            else:
                self.send_thrust(-1600.0, -1600.0)
                self.publish_status("REVERSING")
                return
        else:
            self.reverse_start_time = None

        # NEW: Only use reactive avoidance if NOT on A* path
        if self.obstacle_detected and not self.following_astar_path:
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.avoidance_mode = True
                self.get_logger().info("Avoidance mode - PID reset")

            if self.best_gap and self.best_gap.get('width', 0) > 20:
                gap_direction_deg = self.best_gap.get('direction', 0)
                avoidance_heading = self.current_yaw + math.radians(gap_direction_deg)
                direction = f"GAP {gap_direction_deg:.0f}°"
            elif self.left_clear > self.right_clear:
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)
                avoidance_heading = self.current_yaw + turn_angle
                direction = "LEFT"
            else:
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)
                avoidance_heading = self.current_yaw - turn_angle
                direction = "RIGHT"

            angle_error = self.normalize_angle(avoidance_heading - self.current_yaw)
            self.get_logger().warn(
                f"🚨 OBSTACLE {self.min_obstacle_distance:.1f}m - {direction}",
                throttle_duration_sec=1.0
            )
        else:
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

        # Speed calculation
        angle_error_deg = abs(math.degrees(angle_error))
        if angle_error_deg < self.turn_deadband_deg:
            turn_power = 0.0
            angle_error_deg = 0.0

        if angle_error_deg > 45:
            speed = self.base_speed * 0.5
        elif angle_error_deg > 20:
            speed = self.base_speed * 0.75
        else:
            speed = self.base_speed

        if math.isfinite(self.distance_to_target) and self.distance_to_target < self.approach_slow_distance:
            frac = max(0.0, min(1.0, self.distance_to_target / self.approach_slow_distance))
            speed *= (self.approach_slow_factor + (1.0 - self.approach_slow_factor) * frac)

        if self.obstacle_detected:
            if self.urgency > 0.0:
                speed_factor = 1.0 - (self.urgency * (1.0 - self.obstacle_slow_factor))
                speed *= speed_factor
            else:
                speed *= self.obstacle_slow_factor

        left_thrust = speed - turn_power
        right_thrust = speed + turn_power
        left_thrust = max(-SAFE_THRUST, min(SAFE_THRUST, left_thrust))
        right_thrust = max(-SAFE_THRUST, min(SAFE_THRUST, right_thrust))
        left_thrust = self._slew_limit(self.prev_left_thrust, left_thrust)
        right_thrust = self._slew_limit(self.prev_right_thrust, right_thrust)
        self.prev_left_thrust = left_thrust
        self.prev_right_thrust = right_thrust

        self.send_thrust(left_thrust, right_thrust)
        
        # NEW: Show A* mode in status
        mode = "A*_PATH" if self.following_astar_path else ("AVOIDANCE" if self.avoidance_mode else "NAVIGATION")
        self.publish_status(mode)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _slew_limit(self, previous: float, target: float) -> float:
        """Limit thrust change per cycle"""
        delta = target - previous
        delta = max(-self.slew_rate_limit, min(self.slew_rate_limit, delta))
        return previous + delta

    def send_thrust(self, left, right):
        """Publish thruster commands"""
        if not self.mission_active:
            left = 0.0
            right = 0.0
        
        left_msg = Float64()
        left_msg.data = left
        self.pub_left.publish(left_msg)

        right_msg = Float64()
        right_msg.data = right
        self.pub_right.publish(right_msg)

    def stop(self):
        """Stop the boat"""
        self.send_thrust(0.0, 0.0)

    def publish_status(self, mode):
        """Publish controller status"""
        msg = String()
        msg.data = json.dumps({
            'mode': mode,
            'avoidance_active': self.avoidance_mode,
            'obstacle_detected': bool(self.obstacle_detected),
            'obstacle_distance': round(float(self.min_obstacle_distance), 2),
            'current_yaw': round(math.degrees(self.current_yaw), 1),
            'integral_error': round(float(self.integral_error), 4),
            'urgency': round(float(self.urgency), 2),
            'obstacle_count': int(self.obstacle_count),
            'is_critical': bool(self.is_critical),
            'following_astar': bool(self.following_astar_path)
        })
        self.pub_status.publish(msg)

    # ==================== SMART ANTI-STUCK SYSTEM ====================
    
    def update_position_history(self):
        """Update position history for drift estimation"""
        self.position_history.append((self.current_x, self.current_y, self.get_clock().now()))
        if len(self.position_history) > 100:
            self.position_history.pop(0)
        self.estimate_drift()
    
    def check_stuck_condition(self):
        """Detect if boat is stuck"""
        if self.escape_mode:
            return
        
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
                    self.integral_error = 0.0
                    self.last_escape_position = (self.current_x, self.current_y)
                    self.best_escape_direction = None
                    self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}
                    self.consecutive_stuck_count += 1
                    self.add_no_go_zone(self.current_x, self.current_y)
                    self.calculate_adaptive_escape_duration()
                    self.get_logger().warn(f"STUCK! Smart escape initiating (Attempt {self.consecutive_stuck_count})")
                    
                    if self.consecutive_stuck_count >= 4:
                        self.consecutive_stuck_count = 0
                        self.is_stuck = False
                        self.escape_mode = False
            else:
                if self.is_stuck:
                    self.record_escape_result(success=True)
                    self.get_logger().info("✅ Escape successful!")
                self.is_stuck = False
                self.escape_mode = False
                self.escape_phase = 0
            
            self.last_position = (self.current_x, self.current_y)
            self.stuck_check_time = self.get_clock().now()
    
    def execute_smart_escape(self):
        """Execute smart escape maneuver"""
        if not self.mission_active:
            self._reset_all_escape_state()
            self.stop()
            return
        
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
            turn_power = 600.0 + (self.consecutive_stuck_count * 100.0)
            turn_power = min(900.0, turn_power)
            
            if self.best_escape_direction == 'LEFT':
                self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
            else:
                self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
            return
        
        elif elapsed < forward_end:
            self.escape_phase = 3
            if self.is_heading_toward_no_go_zone():
                self.send_thrust(-400.0, 400.0)
            else:
                self.send_thrust(400.0 + drift_comp_left, 400.0 + drift_comp_right)
            return
        
        else:
            self.escape_mode = False
            self.is_stuck = False
            self.escape_phase = 0
            self.integral_error = 0.0
            self.previous_error = 0.0
            self.last_position = (self.current_x, self.current_y)
            self.stuck_check_time = self.get_clock().now()
    
    def calculate_adaptive_escape_duration(self):
        """Calculate escape duration based on situation"""
        base_duration = 10.0
        
        if self.min_obstacle_distance < self.critical_distance:
            base_duration += 4.0
        elif self.min_obstacle_distance < 15.0:
            base_duration += 2.0
        
        base_duration += self.consecutive_stuck_count * 2.0
        self.adaptive_escape_duration = min(20.0, base_duration)
    
    def add_no_go_zone(self, x, y):
        """Add no-go zone around stuck location"""
        for i, zone in enumerate(self.no_go_zones):
            if math.hypot(x - zone[0], y - zone[1]) < self.no_go_zone_radius:
                self.no_go_zones[i] = (zone[0], zone[1], zone[2] + 2.0)
                return
        
        self.no_go_zones.append((x, y, self.no_go_zone_radius))
        
        if len(self.no_go_zones) > 20:
            self.no_go_zones.pop(0)
    
    def is_in_no_go_zone(self, x, y):
        """Check if position is in any no-go zone"""
        for zone_x, zone_y, radius in self.no_go_zones:
            if math.hypot(x - zone_x, y - zone_y) < radius:
                return True
        return False
    
    def is_heading_toward_no_go_zone(self):
        """Check if current heading leads to no-go zone"""
        look_ahead = 10.0
        future_x = self.current_x + look_ahead * math.cos(self.current_yaw)
        future_y = self.current_y + look_ahead * math.sin(self.current_yaw)
        return self.is_in_no_go_zone(future_x, future_y)
    
    def determine_best_escape_direction(self):
        """Determine best escape direction from probe results"""
        left = self.probe_results['left']
        right = self.probe_results['right']
        
        threshold = 3.0
        if left > right + threshold:
            return 'LEFT'
        elif right > left + threshold:
            return 'RIGHT'
        else:
            return self.get_learned_escape_direction()
    
    def estimate_drift(self):
        """Estimate drift using Kalman filter"""
        if len(self.position_history) < 20:
            self.drift_kalman.predict()
            self.drift_vector = self.drift_kalman.get_drift()
            return
        
        self.drift_kalman.predict()
        
        recent = self.position_history[-20:]
        total_dx = recent[-1][0] - recent[0][0]
        total_dy = recent[-1][1] - recent[0][1]
        time_diff = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        
        if time_diff > 0.5:
            measured_drift_x = total_dx / time_diff
            measured_drift_y = total_dy / time_diff
            self.drift_kalman.update([measured_drift_x, measured_drift_y])
        
        self.drift_vector = self.drift_kalman.get_drift()
    
    def calculate_drift_compensation(self):
        """Calculate thrust compensation for drift"""
        drift_magnitude = math.hypot(self.drift_vector[0], self.drift_vector[1])
        
        if drift_magnitude < 0.1:
            return 0.0, 0.0
        
        drift_angle = math.atan2(self.drift_vector[1], self.drift_vector[0])
        relative_drift = self.normalize_angle(drift_angle - self.current_yaw)
        
        compensation = min(150.0, drift_magnitude * self.drift_compensation_gain * 100.0)
        
        if relative_drift > 0:
            return compensation, -compensation
        else:
            return -compensation, compensation
    
    def record_escape_result(self, success):
        """Record escape result for learning"""
        if self.last_escape_position is None:
            return
        
        self.escape_history.append({
            'direction': self.best_escape_direction,
            'success': success
        })
        
        if len(self.escape_history) > 50:
            self.escape_history.pop(0)
    
    def get_learned_escape_direction(self):
        """Get best escape direction from history"""
        if not self.escape_history:
            return 'LEFT'
        
        left_success = sum(1 for r in self.escape_history if r['direction'] == 'LEFT' and r['success'])
        right_success = sum(1 for r in self.escape_history if r['direction'] == 'RIGHT' and r['success'])
        
        return 'LEFT' if left_success >= right_success else 'RIGHT'
    
    def publish_anti_stuck_status(self):
        """Publish anti-stuck system status"""
        drift_uncertainty = self.drift_kalman.get_uncertainty()
        
        msg = String()
        msg.data = json.dumps({
            'is_stuck': self.is_stuck,
            'escape_mode': self.escape_mode,
            'escape_phase': self.escape_phase,
            'consecutive_attempts': self.consecutive_stuck_count,
            'adaptive_duration': round(self.adaptive_escape_duration, 1),
            'no_go_zones': len(self.no_go_zones),
            'drift_vector': [round(self.drift_vector[0], 3), round(self.drift_vector[1], 3)],
            'drift_uncertainty': [round(drift_uncertainty[0], 3), round(drift_uncertainty[1], 3)],
            'drift_kalman_gain': [
                round(float(self.drift_kalman.last_kalman_gain[0, 0]), 3),
                round(float(self.drift_kalman.last_kalman_gain[1, 1]), 3)
            ],
            'escape_history_count': len(self.escape_history),
            'best_direction': self.best_escape_direction,
            'probe_results': {
                'left': round(self.probe_results['left'], 1),
                'right': round(self.probe_results['right'], 1),
                'back': round(self.probe_results['back'], 1)
            },
            'following_astar': self.following_astar_path
        })
        self.pub_anti_stuck.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuranController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()