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
- Path-aware navigation (A* path following vs reactive avoidance)

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
        /control/replan_request (String) - Request replanning from planner
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
# ┌─────────────────────────────────────────────────────────────────────────┐
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
# └─────────────────────────────────────────────────────────────────────────┘
#
# ┌─────────────────────────────────────────────────────────────────────────┐
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
# └─────────────────────────────────────────────────────────────────────────┘
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
    Enhanced with Smart Anti-Stuck System (SASS) and path-aware navigation
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
        self.declare_parameter('reverse_timeout', 5.0)
        self.declare_parameter('path_blocked_threshold', 8.0)  # Distance to consider path blocked
        
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
        self.path_blocked_threshold = self.get_parameter('path_blocked_threshold').value
        
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
        
        # Path awareness
        self.following_astar_path = False  # True when following A* planned path
        self.replan_cooldown = 0  # Prevent excessive replan requests
        
        # Mission state (from planner)
        self.mission_active = False  # True only when planner is in DRIVING state

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
        self.obstacle_clusters = []  # Raw cluster data for path checking

        # Avoidance state
        self.avoidance_mode = False
        self.reverse_start_time = None
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
        
        # Subscribe to runtime config updates (PID, speed)
        self.create_subscription(
            String,
            '/vostok1/set_config',
            self.config_callback,
            10
        )
        # Also listen to modular config topic
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
        
        # Request replanning from planner
        self.pub_replan_request = self.create_publisher(String, '/control/replan_request', 10)

        # Control loop at 20Hz
        self.create_timer(self.dt, self.control_loop)
        
        # Anti-stuck status publisher at 2Hz
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("=" * 50)
        self.get_logger().info("BURAN - Systeme de Controle de Mouvement")
        self.get_logger().info("Buran Controller - PID Heading Control")
        self.get_logger().info("+ Smart Anti-Stuck System (SASS) v2.0")
        self.get_logger().info("+ Path-Aware Navigation (A* Integration)")
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
            # Check if we're following an A* path
            self.following_astar_path = data.get('astar_path', False)
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
            self.obstacle_clusters = data.get('clusters', [])
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
                self.get_logger().info(f"📋 Mission status changed: {was_active} → {self.mission_active} (state={state})")
            
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
                
            if updated:
                self.get_logger().info(f"⚙️ Config updated: {', '.join(updated)}")
                
        except Exception as e:
            self.get_logger().error(f"Config parse error: {e}")

    def check_path_blocked(self):
        """Check if direct path to target is blocked by obstacles"""
        if not self.obstacle_clusters or self.target_x is None:
            return False
        
        # Check if any obstacle cluster is near the direct line to target
        for cluster in self.obstacle_clusters:
            # Convert cluster from local to global coordinates
            cx = self.current_x + cluster['x'] * math.cos(self.current_yaw) - cluster['y'] * math.sin(self.current_yaw)
            cy = self.current_y + cluster['x'] * math.sin(self.current_yaw) + cluster['y'] * math.cos(self.current_yaw)
            
            # Calculate distance from obstacle to line segment (current -> target)
            dist = self._point_to_segment_distance(
                (cx, cy),
                (self.current_x, self.current_y),
                (self.target_x, self.target_y)
            )
            
            if dist < self.path_blocked_threshold:
                return True
        
        return False
    
    def _point_to_segment_distance(self, point, seg_start, seg_end):
        """Calculate minimum distance from point to line segment"""
        px, py = point
        sx, sy = seg_start
        ex, ey = seg_end
        
        dx = ex - sx
        dy = ey - sy
        
        if dx == 0 and dy == 0:
            return math.hypot(px - sx, py - sy)
        
        t = max(0, min(1, ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)))
        
        proj_x = sx + t * dx
        proj_y = sy + t * dy
        
        return math.hypot(px - proj_x, py - proj_y)
    
    def request_replanning(self):
        """Request planner to replan around obstacles"""
        if self.replan_cooldown > 0:
            return
        
        self.replan_cooldown = 40  # Wait 2 seconds (40 cycles at 20Hz) before next request
        
        msg = String()
        msg.data = json.dumps({
            'type': 'replan',
            'current_position': [self.current_x, self.current_y],
            'target': [self.target_x, self.target_y],
            'reason': 'path_blocked'
        })
        self.pub_replan_request.publish(msg)
        self.get_logger().warn("🔄 Requesting A* replanning - path blocked!")

    def control_loop(self):
        """Main control loop - PID heading control with obstacle avoidance and smart anti-stuck"""
        # Decrement replan cooldown
        if self.replan_cooldown > 0:
            self.replan_cooldown -= 1
        
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
        
        # --- PATH BLOCKED CHECK (only if not following A* path) ---
        if not self.following_astar_path and self.check_path_blocked():
            self.request_replanning()

        # --- CRITICAL OBSTACLE - REVERSE ---
        if self.is_critical and self.min_obstacle_distance < self.critical_distance:
            if self.reverse_start_time is None:
                self.reverse_start_time = self.get_clock().now()
                self.integral_error = 0.0
                self.get_logger().warn(
                    f"CRITICAL OBSTACLE {self.min_obstacle_distance:.1f}m - Reversing!"
                )

            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed > self.reverse_timeout:
                self.get_logger().warn("Reverse timeout - switching to turn mode")
                self.reverse_start_time = None
            else:
                self.send_thrust(-1600.0, -1600.0)  # Increased reverse thrust (v2.1: was -800)
                self.publish_status("REVERSING")
                return
        else:
            self.reverse_start_time = None

        # --- OBSTACLE AVOIDANCE MODE (only if not following A* path) ---
        if self.obstacle_detected and not self.following_astar_path:
            if not self.avoidance_mode:
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.avoidance_mode = True
                self.get_logger().info("Avoidance mode - PID reset")

            # OKO v2.0: Use best_gap for smarter navigation if available
            if self.best_gap and self.best_gap.get('width', 0) > 20:
                # Navigate towards the best gap direction
                gap_direction_deg = self.best_gap.get('direction', 0)
                gap_width = self.best_gap.get('width', 0)
                avoidance_heading = self.current_yaw + math.radians(gap_direction_deg)
                direction = f"GAP {gap_direction_deg:.0f}° ({gap_width:.0f}° wide)"
            elif self.left_clear > self.right_clear:
                # Fallback: Turn towards clearer side
                # Use urgency to scale turn angle: higher urgency = sharper turn
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)  # 45° to 90°
                avoidance_heading = self.current_yaw + turn_angle
                direction = "GAUCHE/LEFT"
            else:
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)  # 45° to 90°
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
            angle