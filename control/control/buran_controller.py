#!/usr/bin/env python3
"""
Navigation Controller Node - PID Heading Control with Smart Anti-Stuck System

Part of the modular Vostok1 architecture.
Subscribes to planner targets and perception data, outputs thruster commands.
Includes advanced anti-stuck system with:
- Adaptive escape duration
- Multi-direction probing
- No-go zone memory
- Drift/current compensation
- Detour waypoint insertion
- Simple escape learning

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
        /control/anti_stuck_status (String) - Smart anti-stuck system status
"""

import rclpy
from rclpy.node import Node
import math
import json

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, String


class BuranController(Node):
    """
    BURAN (БУРАН) - "Snowstorm" in Russian
    Soviet space shuttle program reference
    Enhanced with Smart Anti-Stuck System (SASS)
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

        # Obstacle avoidance
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('reverse_timeout', 5.0)
        
        # Smart anti-stuck parameters
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('stuck_threshold', 0.5)
        self.declare_parameter('no_go_zone_radius', 8.0)
        self.declare_parameter('drift_compensation_gain', 0.3)
        self.declare_parameter('detour_distance', 12.0)

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.obstacle_slow_factor = self.get_parameter('obstacle_slow_factor').value
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
        self.dt = 0.05  # 20Hz

        # Target state (from planner)
        self.target_x = None
        self.target_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_to_target = float('inf')

        # Obstacle state (from perception)
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.is_critical = False

        # Avoidance state
        self.avoidance_mode = False
        self.reverse_start_time = None

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
        
        # Drift compensation
        self.position_history = []
        self.drift_vector = (0.0, 0.0)
        
        # Escape learning
        self.escape_history = []
        self.best_escape_direction = None
        self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}
        self.last_escape_position = None
        
        # Detour waypoints
        self.detour_requested = False

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
            String,
            '/planning/current_target',
            self.target_callback,
            10
        )
        self.create_subscription(
            String,
            '/perception/obstacle_info',
            self.obstacle_callback,
            10
        )

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_status = self.create_publisher(String, '/control/status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/control/anti_stuck_status', 10)
        
        # Request detour waypoint publisher
        self.pub_detour_request = self.create_publisher(String, '/planning/detour_request', 10)

        # Control loop at 20Hz
        self.create_timer(self.dt, self.control_loop)
        
        # Anti-stuck status publisher at 2Hz
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("=" * 50)
        self.get_logger().info("БУРАН (BURAN) - Система Управления Движением")
        self.get_logger().info("Buran Controller - PID Heading Control")
        self.get_logger().info("+ Smart Anti-Stuck System (SASS) v2.0")
        self.get_logger().info(f"ПИД | PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info(f"Speed: {self.base_speed} (max: {self.max_speed})")
        self.get_logger().info(f"Anti-Stuck: timeout={self.stuck_timeout}s, threshold={self.stuck_threshold}m")
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
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Invalid target message: {e}")

    def obstacle_callback(self, msg):
        """Receive obstacle information from perception"""
        try:
            data = json.loads(msg.data)
            self.obstacle_detected = data['obstacle_detected']
            self.min_obstacle_distance = data['min_distance']
            self.front_clear = data['front_clear']
            self.left_clear = data['left_clear']
            self.right_clear = data['right_clear']
            self.is_critical = data['is_critical']
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Invalid obstacle message: {e}")

    def control_loop(self):
        """Main control loop - PID heading control with obstacle avoidance and smart anti-stuck"""
        # Check if we have a target
        if self.target_x is None or self.target_y is None:
            self.stop()
            return
        
        # Initialize stuck detection on first run
        if self.last_position is None:
            self.last_position = (self.current_x, self.current_y)
            self.stuck_check_time = self.get_clock().now()
        
        # Update position history for drift estimation
        self.update_position_history()
        
        # Check for stuck condition (if not already in escape mode)
        if not self.escape_mode:
            self.check_stuck_condition()
        
        # --- SMART ESCAPE MODE ---
        if self.is_stuck and self.escape_mode:
            self.execute_smart_escape()
            return

        # --- CRITICAL OBSTACLE - REVERSE ---
        if self.is_critical and self.min_obstacle_distance < self.critical_distance:
            if self.reverse_start_time is None:
                self.reverse_start_time = self.get_clock().now()
                self.integral_error = 0.0
                self.get_logger().warn(
                    f"🚨 КРИТИЧЕСКОЕ ПРЕПЯТСТВИЕ {self.min_obstacle_distance:.1f}m - Реверс! | "
                    f"CRITICAL OBSTACLE - Reversing!"
                )

            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                self.get_logger().warn("Reverse timeout - switching to turn mode")
                self.reverse_start_time = None
            else:
                self.send_thrust(-800.0, -800.0)
                self.publish_status("REVERSING")
                return
        else:
            self.reverse_start_time = None

        # --- OBSTACLE AVOIDANCE MODE ---
        if self.obstacle_detected:
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.avoidance_mode = True
                self.get_logger().info("⚠️ Режим обхода препятствий - ПИД сброшен | Avoidance mode - PID reset")

            # Turn towards clearer direction
            if self.left_clear > self.right_clear:
                avoidance_heading = self.current_yaw + math.pi / 2
                direction = "ЛЕВО/LEFT"
            else:
                avoidance_heading = self.current_yaw - math.pi / 2
                direction = "ПРАВО/RIGHT"

            angle_error = self.normalize_angle(avoidance_heading - self.current_yaw)

            self.get_logger().warn(
                f"🚨 ПРЕПЯТСТВИЕ {self.min_obstacle_distance:.1f}m - Поворот {direction} "
                f"(Л:{self.left_clear:.1f}m П:{self.right_clear:.1f}m)",
                throttle_duration_sec=1.0
            )
        else:
            # --- NORMAL WAYPOINT NAVIGATION ---
            if self.avoidance_mode:
                self.get_logger().info("✅ Путь свободен - Продолжение навигации | Path CLEAR - Resuming navigation")
                self.avoidance_mode = False
                self.integral_error = 0.0
                self.previous_error = 0.0

            # Calculate desired heading to waypoint
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)

        # --- PID CONTROL ---
        self.integral_error += angle_error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error))

        derivative_error = (angle_error - self.previous_error) / self.dt

        turn_power = (
            self.kp * angle_error +
            self.ki * self.integral_error +
            self.kd * derivative_error
        )

        self.previous_error = angle_error
        turn_power = max(-800.0, min(800.0, turn_power))

        # --- SPEED CALCULATION ---
        angle_error_deg = abs(math.degrees(angle_error))
        if angle_error_deg > 45:
            speed = self.base_speed * 0.5
        elif angle_error_deg > 20:
            speed = self.base_speed * 0.75
        else:
            speed = self.base_speed

        # Distance-based slowdown
        if self.distance_to_target < 5.0:
            speed *= 0.7

        # Obstacle-based slowdown
        if self.obstacle_detected:
            speed *= self.obstacle_slow_factor

        # --- DIFFERENTIAL THRUST ---
        left_thrust = speed - turn_power
        right_thrust = speed + turn_power

        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))

        self.send_thrust(left_thrust, right_thrust)
        
        # Publish status
        mode = "AVOIDANCE" if self.avoidance_mode else "NAVIGATION"
        self.publish_status(mode)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def send_thrust(self, left, right):
        """Publish thruster commands"""
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
            'obstacle_detected': self.obstacle_detected,
            'obstacle_distance': round(self.min_obstacle_distance, 2),
            'current_yaw': round(math.degrees(self.current_yaw), 1),
            'integral_error': round(self.integral_error, 4)
        })
        self.pub_status.publish(msg)

    # ==================== SMART ANTI-STUCK SYSTEM (SASS) ====================
    
    def update_position_history(self):
        """Update position history for drift estimation"""
        self.position_history.append((self.current_x, self.current_y, self.get_clock().now()))
        if len(self.position_history) > 100:
            self.position_history.pop(0)
        self.estimate_drift()
    
    def check_stuck_condition(self):
        """Detect if boat is stuck - Enhanced with smart detection"""
        if self.escape_mode:
            return
        
        # Calculate distance moved
        dx = self.current_x - self.last_position[0]
        dy = self.current_y - self.last_position[1]
        distance_moved = math.hypot(dx, dy)
        
        # Check elapsed time
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
                    
                    # Add no-go zone
                    self.add_no_go_zone(self.current_x, self.current_y)
                    
                    # Calculate adaptive duration
                    self.calculate_adaptive_escape_duration()
                    
                    self.get_logger().warn(
                        f"🚨 ЗАСТРЯЛ! | STUCK! Smart escape initiating "
                        f"(Attempt {self.consecutive_stuck_count}, Duration: {self.adaptive_escape_duration:.1f}s)"
                    )
                    
                    # Request detour on 2nd attempt
                    if self.consecutive_stuck_count == 2:
                        self.request_detour_waypoint()
                    
                    # Skip after 4 attempts
                    if self.consecutive_stuck_count >= 4:
                        self.get_logger().error(f"Stuck {self.consecutive_stuck_count} times - requesting waypoint skip")
                        self.consecutive_stuck_count = 0
                        self.is_stuck = False
                        self.escape_mode = False
            else:
                if self.is_stuck:
                    self.record_escape_result(success=True)
                    self.get_logger().info("✅ Escape successful! Resuming navigation")
                self.is_stuck = False
                self.escape_mode = False
                self.escape_phase = 0
            
            self.last_position = (self.current_x, self.current_y)
            self.stuck_check_time = self.get_clock().now()
    
    def execute_smart_escape(self):
        """Execute smart escape maneuver with probing and adaptive duration"""
        elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
        
        # Phase boundaries
        probe_end = 2.0
        reverse_end = probe_end + self.adaptive_escape_duration * 0.4
        turn_end = reverse_end + self.adaptive_escape_duration * 0.35
        forward_end = turn_end + self.adaptive_escape_duration * 0.25
        
        # Get drift compensation
        drift_comp_left, drift_comp_right = self.calculate_drift_compensation()
        
        # PHASE 0: Probe
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
                    self.get_logger().info(
                        f"Probe: L:{self.probe_results['left']:.1f}m R:{self.probe_results['right']:.1f}m → {self.best_escape_direction}"
                    )
            
            self.get_logger().info(f"Phase 0: PROBING ({probe_time:.1f}s)", throttle_duration_sec=0.5)
            return
        
        # PHASE 1: Reverse
        elif elapsed < reverse_end:
            self.escape_phase = 1
            reverse_power = -700.0 - min(300.0, 100.0 / max(0.5, self.min_obstacle_distance))
            self.send_thrust(reverse_power + drift_comp_left, reverse_power + drift_comp_right)
            self.get_logger().info(f"Phase 1: REVERSE ({elapsed:.1f}s/{reverse_end:.1f}s)", throttle_duration_sec=1.0)
            return
        
        # PHASE 2: Turn
        elif elapsed < turn_end:
            self.escape_phase = 2
            turn_power = 600.0 + (self.consecutive_stuck_count * 100.0)
            turn_power = min(900.0, turn_power)
            
            if self.best_escape_direction == 'LEFT':
                self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
            else:
                self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
            
            self.get_logger().info(
                f"Phase 2: TURN {self.best_escape_direction} ({elapsed:.1f}s/{turn_end:.1f}s)",
                throttle_duration_sec=1.0
            )
            return
        
        # PHASE 3: Forward test
        elif elapsed < forward_end:
            self.escape_phase = 3
            if self.is_heading_toward_no_go_zone():
                self.get_logger().warn("Forward leads to no-go zone - extra turn")
                self.send_thrust(-400.0, 400.0)
            else:
                self.send_thrust(400.0 + drift_comp_left, 400.0 + drift_comp_right)
            
            self.get_logger().info(f"Phase 3: FORWARD TEST ({elapsed:.1f}s/{forward_end:.1f}s)", throttle_duration_sec=1.0)
            return
        
        # Escape complete
        else:
            self.get_logger().info(f"Smart escape complete ({self.adaptive_escape_duration:.1f}s)")
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
        self.get_logger().info(f"Added no-go zone #{len(self.no_go_zones)} at ({x:.1f}, {y:.1f})")
        
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
        """Estimate drift from position history"""
        if len(self.position_history) < 20:
            return
        
        recent = self.position_history[-20:]
        total_dx = recent[-1][0] - recent[0][0]
        total_dy = recent[-1][1] - recent[0][1]
        time_diff = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        
        if time_diff > 0.5:
            alpha = 0.1
            new_drift_x = total_dx / time_diff
            new_drift_y = total_dy / time_diff
            self.drift_vector = (
                alpha * new_drift_x + (1 - alpha) * self.drift_vector[0],
                alpha * new_drift_y + (1 - alpha) * self.drift_vector[1]
            )
    
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
    
    def request_detour_waypoint(self):
        """Request planner to insert detour waypoint"""
        if self.left_clear > self.right_clear:
            detour_angle = self.current_yaw + math.pi / 2
        else:
            detour_angle = self.current_yaw - math.pi / 2
        
        detour_x = self.current_x + self.detour_distance * math.cos(detour_angle)
        detour_y = self.current_y + self.detour_distance * math.sin(detour_angle)
        
        if self.is_in_no_go_zone(detour_x, detour_y):
            detour_angle += math.pi
            detour_x = self.current_x + self.detour_distance * math.cos(detour_angle)
            detour_y = self.current_y + self.detour_distance * math.sin(detour_angle)
        
        msg = String()
        msg.data = json.dumps({
            'type': 'detour',
            'x': round(detour_x, 2),
            'y': round(detour_y, 2)
        })
        self.pub_detour_request.publish(msg)
        
        self.get_logger().warn(f"Detour waypoint requested at ({detour_x:.1f}, {detour_y:.1f})")
    
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
        """Publish anti-stuck system status for dashboard"""
        msg = String()
        msg.data = json.dumps({
            'is_stuck': self.is_stuck,
            'escape_mode': self.escape_mode,
            'escape_phase': self.escape_phase,
            'consecutive_attempts': self.consecutive_stuck_count,
            'adaptive_duration': round(self.adaptive_escape_duration, 1),
            'no_go_zones': len(self.no_go_zones),
            'drift_vector': [round(self.drift_vector[0], 3), round(self.drift_vector[1], 3)],
            'escape_history_count': len(self.escape_history),
            'best_direction': self.best_escape_direction,
            'probe_results': {
                'left': round(self.probe_results['left'], 1),
                'right': round(self.probe_results['right'], 1),
                'back': round(self.probe_results['back'], 1)
            }
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