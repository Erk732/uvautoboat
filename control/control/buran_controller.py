#!/usr/bin/env python3
"""
Buran Controller - Motion Control System

Part of the modular Vostok1 architecture.
Subscribes to planner targets and perception data, outputs thruster commands.

Features:
- PID heading control with anti-windup
- Smart Anti-Stuck System (SASS) with Kalman-filtered drift estimation
- Multi-phase escape maneuvers
- No-go zone memory

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
MAX_THRUST = 2000.0          # Newtons - hardware limit (v2.1: increased from 1000)
SAFE_THRUST = 800.0          # Newtons - operational limit
INTEGRAL_LIMIT = 0.5         # radians - prevent integral windup
TURN_POWER_LIMIT = 1600.0     # Newtons - max differential thrust (v2.1: increased from 800)
SENSOR_TIMEOUT = 2.0         # seconds


# =============================================================================
# BAYESIAN STATE ESTIMATION FUNDAMENTALS
# =============================================================================
#
# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                        BAYES' THEOREM                                       │
# │                                                                             │
# │   P(State | Data) = P(Data | State) × P(State) / P(Data)                    │
# │                                                                             │
# │   In robotics terms:                                                        │
# │     Posterior   = Likelihood × Prior / Evidence                             │
# │     (new belief)   (sensor)    (old belief)                                 │
# │                                                                             │
# │   Example: "What is the drift/current affecting my boat?"                   │
# │     Prior:      Previous estimate of water current velocity                 │
# │     Likelihood: Observed velocity vs expected velocity (the difference)     │
# │     Posterior:  Updated drift estimate combining prediction + observation   │
# │                                                                             │
# │   This is the foundation of ALL probabilistic robotics!                     │
# └─────────────────────────────────────────────────────────────────────────────┘
#
# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                   KALMAN FILTER = BAYESIAN + GAUSSIAN                       │
# │                                                                             │
# │   For continuous states with Gaussian (bell curve) distributions:           │
# │     - State described by mean μ (best estimate) and variance σ² (uncertainty)│
# │     - Multiplying two Gaussians → another Gaussian (closed-form solution)   │
# │                                                                             │
# │   Predict-Update Cycle:                                                     │
# │     PREDICT: x̂⁻ = F×x̂, P⁻ = F×P×Fᵀ+Q   (uncertainty grows)                │
# │     UPDATE:  K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹        (Kalman Gain)                       │
# │              x̂ = x̂⁻ + K(z-Hx̂⁻)          (correct with measurement)       │
# │              P = (I-KH)P⁻                (uncertainty shrinks)              │
# │                                                                             │
# │   Kalman Gain K: How much to trust the measurement vs prediction            │
# │     K→0: High R (noisy sensor) → trust prediction more                      │
# │     K→1: High Q (unstable model) → trust measurement more                   │
# └─────────────────────────────────────────────────────────────────────────────┘
#
# =============================================================================
# KALMAN FILTER FOR DRIFT ESTIMATION
# =============================================================================
class KalmanDriftEstimator:
    """
    2D Kalman Filter for estimating water/wind drift.
    
    BAYESIAN INTERPRETATION:
        Prior:      Previous drift estimate with uncertainty P
        Likelihood: How well observed velocity matches predicted velocity
        Posterior:  Updated drift estimate after measurement
    
    State: [drift_x, drift_y] - velocity components (m/s)
    
    Tuning:
        High Q, Low R → Trust measurements, respond quickly
        Low Q, High R → Trust model, smooth out noise
    """
    
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
        # Approach slowdown near waypoint
        self.declare_parameter('approach_slow_distance', 5.0)
        self.declare_parameter('approach_slow_factor', 0.7)
        # Smoothness controls
        self.declare_parameter('slew_rate_limit', 80.0)       # Max thrust change per cycle (N)
        self.declare_parameter('turn_deadband_deg', 2.0)      # Ignore tiny heading errors

        # Obstacle avoidance
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('min_safe_distance', 12.0)
        self.declare_parameter('reverse_timeout', 3.0)  # Shorter default reverse timeout
        self.declare_parameter('max_reverse_distance', 25.0)  # Max meters to reverse during escape
        # Softer avoidance steering to avoid sharp pivots
        self.declare_parameter('avoid_diff_gain', 25.0)  # VFH/polar differential steering gain (reduced from 40)
        self.declare_parameter('use_vfh_bias', False)    # Enable/disable VFH/polar steering bias
        
        # Smart anti-stuck parameters
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('stuck_threshold', 0.5)
        self.declare_parameter('no_go_zone_radius', 8.0)
        self.declare_parameter('drift_compensation_gain', 0.3)
        self.declare_parameter('detour_distance', 12.0)
        # Shoreline/bank protection: slow down when very close to obstacles (e.g., banks)
        self.declare_parameter('bank_slow_distance', 6.0)
        self.declare_parameter('bank_slow_factor', 0.25)
        
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
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.reverse_timeout = self.get_parameter('reverse_timeout').value
        self.max_reverse_distance = float(self.get_parameter('max_reverse_distance').value)
        self.avoid_diff_gain = float(self.get_parameter('avoid_diff_gain').value)
        self.use_vfh_bias = bool(self.get_parameter('use_vfh_bias').value)
        
        # Smart anti-stuck parameters
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.no_go_zone_radius = self.get_parameter('no_go_zone_radius').value
        self.drift_compensation_gain = self.get_parameter('drift_compensation_gain').value
        self.detour_distance = self.get_parameter('detour_distance').value
        self.bank_slow_distance = float(self.get_parameter('bank_slow_distance').value)
        self.bank_slow_factor = float(self.get_parameter('bank_slow_factor').value)

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
        self.mission_active = False  # True only when planner is in DRIVING state
        # Highest-priority STOP latch (cleared only by explicit resume/clear)
        self.stop_override = False

        # Obstacle state (from perception)
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.is_critical = False
        # OKO v2.0 enhanced state
        self.urgency = 0.0  # Distance-weighted urgency (0.0-1.0)
        self.best_gap = None  # Best navigation gap direction
        self.obstacle_count = 0  # Number of detected clusters
        # OKO v2.1 VFH/polar state (for differential steering)
        self.vfh_gap = None  # VFH best direction gap info
        self.polar_bias = 0.0  # Polar histogram bias [-1, 1]
        self.force_avoid_active = False  # Global avoidance mode flag

        # Avoidance state
        self.avoidance_mode = False
        self.reverse_start_time = None
        self.reverse_start_pos = None
        self.force_turn_after_reverse = False
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
        self.drift_vector = (0.0, 0.0)  # Updated by Kalman filter
        
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
        
        # Subscribe to mission status to know when to stop
        self.create_subscription(
            String,
            '/planning/mission_status',
            self.mission_status_callback,
            10
        )
        # High-priority mission commands (STOP/RESUME) - modular architecture
        self.create_subscription(
            String,
            '/sputnik/mission_command',
            self.mission_command_callback,
            10
        )
        
        # Subscribe to runtime config updates (PID, speed) - modular architecture
        self.create_subscription(
            String,
            '/sputnik/set_config',
            self.config_callback,
            10
        )

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_status = self.create_publisher(String, '/control/status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/control/anti_stuck_status', 10)
        
        # Request detour waypoint publisher
        self.pub_detour_request = self.create_publisher(String, '/planning/detour_request', 10)
        
        # Request replan from planner (when path is blocked)
        self.pub_replan_request = self.create_publisher(String, '/control/replan_request', 10)
        
        # Request waypoint skip (after multiple stuck attempts)
        self.pub_skip_request = self.create_publisher(String, '/planning/skip_waypoint', 10)

        # Control loop at 20Hz
        self.create_timer(self.dt, self.control_loop)
        
        # Anti-stuck status publisher at 2Hz
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("=" * 50)
        self.get_logger().info("BURAN - Systeme de Controle de Mouvement")
        self.get_logger().info("Buran Controller - PID Heading Control")
        self.get_logger().info("+ Smart Anti-Stuck System (SASS) v2.0")
        self.get_logger().info(f"PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
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
        """Receive obstacle information from perception (OKO v2.0 compatible)"""
        try:
            data = json.loads(msg.data)
            self.obstacle_detected = data.get('obstacle_detected', False)
            self.min_obstacle_distance = data.get('min_distance', float('inf'))
            # OKO v2.0: front_clear, left_clear, right_clear are distances in meters
            self.front_clear = data.get('front_clear', float('inf'))
            self.left_clear = data.get('left_clear', float('inf'))
            self.right_clear = data.get('right_clear', float('inf'))
            self.is_critical = data.get('is_critical', False)
            # OKO v2.0 enhanced fields (backward compatible)
            self.urgency = data.get('urgency', 0.0)
            self.best_gap = data.get('best_gap', None)
            self.obstacle_count = data.get('obstacle_count', 0)
            # OKO v2.1 VFH/polar fields (for fine-tuned steering)
            self.vfh_gap = data.get('vfh_gap', None)
            self.polar_bias = data.get('polar_bias', 0.0)
            self.force_avoid_active = data.get('force_avoid_active', False)
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Invalid obstacle message: {e}")

    def mission_status_callback(self, msg):
        """Receive mission status from planner - stop if not DRIVING"""
        try:
            data = json.loads(msg.data)
            state = data.get('state', '')
            # Only active when planner is in DRIVING state
            was_active = self.mission_active
            self.mission_active = (state == "DRIVING")
            
            # Debug log every status change
            if was_active != self.mission_active:
                self.get_logger().info(f"🔄 Mission status changed: {was_active} → {self.mission_active} (state={state})")
            
            # If mission just became inactive, clear target and stop IMMEDIATELY
            if was_active and not self.mission_active:
                self.target_x = None
                self.target_y = None
                self._reset_all_escape_state()
                # CRITICAL: Stop immediately and multiple times to ensure thrusters cut off
                self.stop()
                self.send_thrust(0.0, 0.0)  # Double-stop - zero thrust explicitly
                self.get_logger().info(f"🛑 Mission IMMEDIATELY inactive (state={state}) - stopping & resetting all states")
            
            # If mission just became active (e.g., go_home, resume), reset escape state for fresh start
            elif not was_active and self.mission_active:
                self._reset_all_escape_state()
                self.get_logger().info(f"✅ Mission active (state={state}) - escape state reset for fresh start")
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Invalid mission status: {e}")

    def mission_command_callback(self, msg):
        """Handle mission commands with highest priority STOP latch"""
        try:
            data = json.loads(msg.data)
            command = data.get('command', '').lower()
        except Exception as e:
            self.get_logger().warn(f"Invalid mission command: {e}")
            return

        if command in ('stop_mission', 'emergency_stop', 'panic_stop'):
            self.stop_override = True
            self.mission_active = False
            self.stop()
            self.send_thrust(0.0, 0.0)
            self.get_logger().warn("🛑 STOP override latched (command). Thrusters cut until resume/clear.")
        elif command in ('force_resume', 'resume_mission', 'clear_stop_override', 'joystick_enable', 'go_home', 'start_mission'):
            if self.stop_override:
                self.get_logger().info(f"Clearing STOP override due to command: {command}")
            self.stop_override = False
    
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
        # Reset stuck detection timing
        self.last_position = None
        self.stuck_check_time = None

    def config_callback(self, msg):
        """Handle runtime configuration changes for PID and speed"""
        try:
            config = json.loads(msg.data)
            updated = []
            
            # PID gains
            if 'kp' in config:
                self.kp = float(config['kp'])
                updated.append(f"Kp={self.kp}")
            if 'ki' in config:
                self.ki = float(config['ki'])
                updated.append(f"Ki={self.ki}")
            if 'kd' in config:
                self.kd = float(config['kd'])
                updated.append(f"Kd={self.kd}")
                
            # Speed parameters
            if 'base_speed' in config:
                self.base_speed = float(config['base_speed'])
                updated.append(f"base_speed={self.base_speed}")
            if 'max_speed' in config:
                self.max_speed = float(config['max_speed'])
                updated.append(f"max_speed={self.max_speed}")
                
            # Obstacle avoidance
            if 'obstacle_slow_factor' in config:
                self.obstacle_slow_factor = float(config['obstacle_slow_factor'])
                updated.append(f"slow_factor={self.obstacle_slow_factor}")
            if 'critical_distance' in config:
                self.critical_distance = float(config['critical_distance'])
                updated.append(f"critical_dist={self.critical_distance}")
            if 'avoid_diff_gain' in config:
                self.avoid_diff_gain = float(config['avoid_diff_gain'])
                updated.append(f"diff_gain={self.avoid_diff_gain}")
            if 'min_safe_distance' in config:
                self.min_safe_distance = float(config['min_safe_distance'])
                updated.append(f"safe_dist={self.min_safe_distance}")
            if 'reverse_timeout' in config:
                self.reverse_timeout = float(config['reverse_timeout'])
                updated.append(f"reverse_timeout={self.reverse_timeout}")
            if 'max_reverse_distance' in config:
                self.max_reverse_distance = float(config['max_reverse_distance'])
                updated.append(f"max_reverse_distance={self.max_reverse_distance}")
            # Bank protection slowdown
            if 'bank_slow_distance' in config:
                self.bank_slow_distance = float(config['bank_slow_distance'])
                updated.append(f"bank_dist={self.bank_slow_distance}")
            if 'bank_slow_factor' in config:
                self.bank_slow_factor = float(config['bank_slow_factor'])
                updated.append(f"bank_factor={self.bank_slow_factor}")
            # Toggle VFH/polar bias
            if 'use_vfh_bias' in config:
                self.use_vfh_bias = bool(config['use_vfh_bias'])
                updated.append(f"use_vfh_bias={self.use_vfh_bias}")

            # Smart Anti-Stuck System (SASS) parameters
            if 'stuck_timeout' in config:
                self.stuck_timeout = float(config['stuck_timeout'])
                updated.append(f"stuck_timeout={self.stuck_timeout}")
            if 'stuck_threshold' in config:
                self.stuck_threshold = float(config['stuck_threshold'])
                updated.append(f"stuck_threshold={self.stuck_threshold}")
            if 'no_go_zone_radius' in config:
                self.no_go_zone_radius = float(config['no_go_zone_radius'])
                updated.append(f"no_go_zone_radius={self.no_go_zone_radius}")
            if 'detour_distance' in config:
                self.detour_distance = float(config['detour_distance'])
                updated.append(f"detour_distance={self.detour_distance}")

            # Control smoothness parameters
            if 'turn_deadband_deg' in config:
                self.turn_deadband_deg = float(config['turn_deadband_deg'])
                updated.append(f"turn_deadband={self.turn_deadband_deg}")
            if 'slew_rate_limit' in config:
                self.slew_rate_limit = float(config['slew_rate_limit'])
                updated.append(f"slew_rate={self.slew_rate_limit}")

            if updated:
                self.get_logger().info(f"⚙️ Config updated: {', '.join(updated)}")
                
        except Exception as e:
            self.get_logger().error(f"Config parse error: {e}")

    def control_loop(self):
        """Main control loop - PID heading control with obstacle avoidance and smart anti-stuck"""
        # Highest priority STOP latch (works even during SASS/panic)
        if self.stop_override:
            self.stop()
            self.send_thrust(0.0, 0.0)
            return

        # Check if mission is active - CRITICAL: Check every control loop to catch stops
        if not self.mission_active:
            self.stop()
            self.send_thrust(0.0, 0.0)  # Ensure thrust is zero if mission became inactive
            return
            
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
            # If we've already hit reverse limits, skip reverse and go to turning
            if self.force_turn_after_reverse:
                self.get_logger().warn("Reverse limit reached - turning instead of further reversing")
            else:
                if self.reverse_start_time is None:
                    self.reverse_start_time = self.get_clock().now()
                    self.reverse_start_pos = (self.current_x, self.current_y)
                    self.integral_error = 0.0
                    self.get_logger().warn(
                        f"CRITICAL OBSTACLE {self.min_obstacle_distance:.1f}m - Reversing (CQB micro-steps)"
                    )

                elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
                # Track reverse distance traveled
                if self.reverse_start_pos is not None:
                    dx = self.current_x - self.reverse_start_pos[0]
                    dy = self.current_y - self.reverse_start_pos[1]
                    reverse_dist = math.hypot(dx, dy)
                else:
                    reverse_dist = 0.0

                # Stop reversing if time or distance exceeded
                if elapsed > self.reverse_timeout or reverse_dist > self.max_reverse_distance:
                    self.get_logger().warn(
                        f"Reverse limit reached (t={elapsed:.1f}s, d={reverse_dist:.1f}m) - switching to turn"
                    )
                    self.reverse_start_time = None
                    self.reverse_start_pos = None
                    self.force_turn_after_reverse = True
                else:
                    # CQB micro-steps: short bursts, then pause
                    cycle = 0.6  # seconds
                    on_time = 0.2
                    phase = elapsed % cycle
                    thrust_cmd = -1200.0 if phase < on_time else 0.0
                    self.send_thrust(thrust_cmd, thrust_cmd)
                    self.publish_status("REVERSING")
                    return

            # After reverse cap, fall through to avoidance turning
            self.reverse_start_time = None
            self.reverse_start_pos = None
            self.force_turn_after_reverse = False
            self.avoidance_mode = True
            if self.reverse_start_time is None:
                self.reverse_start_time = None
        else:
            self.reverse_start_time = None
            self.reverse_start_pos = None
            self.force_turn_after_reverse = False

        # --- OBSTACLE AVOIDANCE MODE ---
        if self.obstacle_detected:
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.avoidance_mode = True
                self.get_logger().info("Avoidance mode - PID reset")
                
                # Request A* replan if obstacle is blocking and urgency is high
                if self.urgency > 0.5 and self.front_clear < self.min_safe_distance:
                    self.request_replan(reason="path_blocked")

            # OKO v2.0: Use best_gap for smarter navigation if available
            if self.best_gap and self.best_gap.get('width', 0) > 20:
                # Navigate towards the best gap direction
                gap_direction_deg = self.best_gap.get('direction', 0)
                gap_width = self.best_gap.get('width', 0)
                avoidance_heading = self.current_yaw + math.radians(gap_direction_deg)
                direction = f"GAP {gap_direction_deg:.0f}° ({gap_width:.0f}° wide)"
            elif self.left_clear > self.right_clear:
                # Fallback: Turn towards clearer side
                # Use urgency to scale turn angle: higher urgency = sharper turn, but keep it mild (30°-60°)
                turn_angle = math.radians(30) + (self.urgency * math.radians(30))  # 30° to 60°
                avoidance_heading = self.current_yaw + turn_angle
                direction = "GAUCHE/LEFT"
            else:
                turn_angle = math.radians(30) + (self.urgency * math.radians(30))  # 30° to 60°
                avoidance_heading = self.current_yaw - turn_angle
                direction = "DROITE/RIGHT"

            angle_error = self.normalize_angle(avoidance_heading - self.current_yaw)

            self.get_logger().warn(
                f"🚨 OBSTACLE {self.min_obstacle_distance:.1f}m (urgency:{self.urgency*100:.0f}%) - "
                f"Virage {direction} (G:{self.left_clear:.1f}m D:{self.right_clear:.1f}m)",
                throttle_duration_sec=1.0
            )
        else:
            # --- NORMAL WAYPOINT NAVIGATION ---
            if self.avoidance_mode:
                self.get_logger().info("Path CLEAR - Resuming navigation")
                self.avoidance_mode = False
                self.integral_error = 0.0
                self.previous_error = 0.0

            # Calculate desired heading to waypoint
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)

        # PID Controller with anti-windup
        self.integral_error += angle_error * self.dt
        self.integral_error = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral_error))

        derivative_error = (angle_error - self.previous_error) / self.dt

        turn_power = (
            self.kp * angle_error +
            self.ki * self.integral_error +
            self.kd * derivative_error
        )

        self.previous_error = angle_error
        
        # Limit turn power
        turn_power = max(-TURN_POWER_LIMIT, min(TURN_POWER_LIMIT, turn_power))

        # --- SPEED CALCULATION ---
        angle_error_deg = abs(math.degrees(angle_error))
        # Small-angle deadband to reduce chatter
        if angle_error_deg < self.turn_deadband_deg:
            turn_power = 0.0
            angle_error_deg = 0.0

        if angle_error_deg > 45:
            speed = self.base_speed * 0.5
        elif angle_error_deg > 20:
            speed = self.base_speed * 0.75
        else:
            speed = self.base_speed

        # Distance-based slowdown (precision near waypoint) - linear ramp
        if math.isfinite(self.distance_to_target) and self.distance_to_target < self.approach_slow_distance:
            frac = max(0.0, min(1.0, self.distance_to_target / self.approach_slow_distance))
            speed *= (self.approach_slow_factor + (1.0 - self.approach_slow_factor) * frac)

        # Obstacle-based slowdown using OKO v2.0 urgency for smoother control
        if self.obstacle_detected:
            # Urgency-based smooth slowdown: higher urgency = more slowdown
            # urgency=0.0 -> full speed, urgency=1.0 -> obstacle_slow_factor
            if self.urgency > 0.0:
                speed_factor = 1.0 - (self.urgency * (1.0 - self.obstacle_slow_factor))
                speed *= speed_factor
            else:
                speed *= self.obstacle_slow_factor

        # Shoreline/bank protection: clamp speed when very close to obstacles
        if math.isfinite(self.min_obstacle_distance) and self.min_obstacle_distance < self.bank_slow_distance:
            # Linear ramp: at 0m → bank_slow_factor, at bank_slow_distance → 1.0
            frac = max(0.0, min(1.0, self.min_obstacle_distance / self.bank_slow_distance))
            speed *= (self.bank_slow_factor + (1.0 - self.bank_slow_factor) * frac)

        # Differential thrust
        left_thrust = speed - turn_power
        right_thrust = speed + turn_power

        # v2.1: Apply VFH/polar bias for fine-tuned steering (from AllInOneStack)
        # Only apply when enabled and avoiding obstacles
        if self.use_vfh_bias and (self.obstacle_detected or self.force_avoid_active):
            diff_bias = 0.0
            vfh_dir = None

            # Skip steering bias if we're seeing only phantom/self hits (far min distance + low urgency)
            allow_bias = (self.min_obstacle_distance <= self.min_safe_distance) or (self.urgency > 0.1) or self.is_critical

            if allow_bias:
                # 1. Left/right clearance bias (steer toward clearer side)
                if self.left_clear < float('inf') and self.right_clear < float('inf'):
                    norm = max(self.left_clear + self.right_clear, 1.0)
                    diff_bias += (self.right_clear - self.left_clear) / norm * self.avoid_diff_gain

                # 2. VFH steering bias (toward best gap direction) - only if not extreme
                if self.vfh_gap is not None:
                    vfh_direction_deg = self.vfh_gap.get('direction', 0.0)
                    if abs(vfh_direction_deg) <= 90.0:  # ignore wild spins
                        vfh_dir = vfh_direction_deg
                        vfh_normalized = max(-1.0, min(1.0, vfh_direction_deg / 45.0))
                        diff_bias += vfh_normalized * self.avoid_diff_gain

                # 3. Polar histogram bias (free space comparison) gated by urgency
                # polar_bias: +1 = left wide open, -1 = right wide open
                if self.force_avoid_active and self.urgency > 0.2:
                    diff_bias += self.polar_bias * self.avoid_diff_gain

                # Clamp bias to avoid over-steering
                diff_bias = max(-self.avoid_diff_gain, min(self.avoid_diff_gain, diff_bias))

                # Apply differential bias: positive bias = turn left
                left_thrust -= diff_bias
                right_thrust += diff_bias

            # Debug VFH/polar application (throttled)
            self.get_logger().info(
                f"VFH steer={vfh_dir if vfh_dir is not None else 'none'} deg | "
                f"polar_bias={self.polar_bias:.2f} | diff_bias={diff_bias:.1f} | "
                f"urgency={self.urgency:.2f} | min={self.min_obstacle_distance:.1f}",
                throttle_duration_sec=1.0
            )

        # Clamp to safe limits
        left_thrust = max(-SAFE_THRUST, min(SAFE_THRUST, left_thrust))
        right_thrust = max(-SAFE_THRUST, min(SAFE_THRUST, right_thrust))

        # Slew-rate limit to avoid sudden reversals/jerk
        left_thrust = self._slew_limit(self.prev_left_thrust, left_thrust)
        right_thrust = self._slew_limit(self.prev_right_thrust, right_thrust)
        self.prev_left_thrust = left_thrust
        self.prev_right_thrust = right_thrust

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

    def _slew_limit(self, previous: float, target: float) -> float:
        """
        Limit thrust change per cycle to avoid abrupt reversals and jerk.
        """
        delta = target - previous
        delta = max(-self.slew_rate_limit, min(self.slew_rate_limit, delta))
        return previous + delta

    def send_thrust(self, left, right):
        """Publish thruster commands - with mission safety check"""
        # SAFETY: If mission is not active, force zero thrust
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
        """Publish controller status with OKO v2.0 enhanced info"""
        msg = String()
        msg.data = json.dumps({
            'mode': mode,
            'stop_override': self.stop_override,
            'avoidance_active': self.avoidance_mode,
            'obstacle_detected': bool(self.obstacle_detected),
            'obstacle_distance': round(float(self.min_obstacle_distance), 2),
            'current_yaw': round(math.degrees(self.current_yaw), 1),
            'integral_error': round(float(self.integral_error), 4),
            # OKO v2.0 enhanced fields
            'urgency': round(float(self.urgency), 2),
            'obstacle_count': int(self.obstacle_count),
            'is_critical': bool(self.is_critical)
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
                        f"STUCK! Smart escape initiating "
                        f"(Attempt {self.consecutive_stuck_count}, Duration: {self.adaptive_escape_duration:.1f}s)"
                    )
                    
                    # Request detour on 2nd attempt
                    if self.consecutive_stuck_count == 2:
                        self.request_detour_waypoint()
                    
                    # Request A* replan on 3rd attempt
                    if self.consecutive_stuck_count == 3:
                        self.request_replan(reason="stuck_multiple_times")
                    
                    # Skip waypoint after 4 attempts
                    if self.consecutive_stuck_count >= 4:
                        self.get_logger().error(f"Stuck {self.consecutive_stuck_count} times - requesting waypoint skip")
                        self.request_waypoint_skip()
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
        # SAFETY: Abort escape immediately if mission is no longer active
        if not self.mission_active:
            self._reset_all_escape_state()
            self.stop()
            return
        
        elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
        
        # Phase boundaries
        probe_end = 1.5
        reverse_end = probe_end + self.adaptive_escape_duration * 0.35
        turn_end = reverse_end + self.adaptive_escape_duration * 0.35
        forward_end = turn_end + self.adaptive_escape_duration * 0.30
        
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
            reverse_power = -500.0 - min(200.0, 80.0 / max(0.5, self.min_obstacle_distance))
            self.send_thrust(reverse_power + drift_comp_left, reverse_power + drift_comp_right)
            self.get_logger().info(f"Phase 1: REVERSE ({elapsed:.1f}s/{reverse_end:.1f}s)", throttle_duration_sec=1.0)
            return
        
        # PHASE 2: Turn
        elif elapsed < turn_end:
            self.escape_phase = 2
            turn_power = 500.0 + (self.consecutive_stuck_count * 80.0)
            turn_power = min(650.0, turn_power)
            
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
                self.send_thrust(-300.0, 300.0)
            else:
                self.send_thrust(300.0 + drift_comp_left, 300.0 + drift_comp_right)
            
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
        base_duration = 8.0
        
        if self.min_obstacle_distance < self.critical_distance:
            base_duration += 4.0
        elif self.min_obstacle_distance < 15.0:
            base_duration += 2.0
        
        base_duration += self.consecutive_stuck_count * 2.0
        self.adaptive_escape_duration = min(15.0, base_duration)
    
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
        """Estimate drift using Kalman filter for optimal estimation."""
        if len(self.position_history) < 20:
            # Still run predict to advance uncertainty
            self.drift_kalman.predict()
            self.drift_vector = self.drift_kalman.get_drift()
            return
        
        # Prediction step
        self.drift_kalman.predict()
        
        # Measurement from position history
        recent = self.position_history[-20:]
        total_dx = recent[-1][0] - recent[0][0]
        total_dy = recent[-1][1] - recent[0][1]
        time_diff = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        
        if time_diff > 0.5:
            measured_drift_x = total_dx / time_diff
            measured_drift_y = total_dy / time_diff
            
            # Update step with Kalman filter
            self.drift_kalman.update([measured_drift_x, measured_drift_y])
        
        # Update legacy tuple for backward compatibility
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
    
    def request_replan(self, reason="obstacle"):
        """Request A* replan from the planner"""
        msg = String()
        msg.data = json.dumps({
            'type': 'replan',
            'reason': reason,
            'current_position': [round(self.current_x, 2), round(self.current_y, 2)],
            'obstacle_distance': round(self.min_obstacle_distance, 2),
            'no_go_zones': [(round(z[0], 1), round(z[1], 1), round(z[2], 1)) for z in self.no_go_zones[-5:]]  # Send last 5 zones
        })
        self.pub_replan_request.publish(msg)
        self.get_logger().info(f"🔄 Replan requested: {reason}")
    
    def request_waypoint_skip(self):
        """Request planner to skip current waypoint after too many stuck attempts"""
        msg = String()
        msg.data = json.dumps({
            'type': 'skip',
            'current_position': [round(self.current_x, 2), round(self.current_y, 2)],
            'reason': f'stuck_{self.consecutive_stuck_count}_times',
            'no_go_zones': [(round(z[0], 1), round(z[1], 1), round(z[2], 1)) for z in self.no_go_zones]
        })
        self.pub_skip_request.publish(msg)
        self.get_logger().warn(f"⏭️ Waypoint skip requested after {self.consecutive_stuck_count} stuck attempts")
    
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
