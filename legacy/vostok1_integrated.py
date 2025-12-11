#!/usr/bin/env python3
"""
Vostok1 - Autonomous Navigation System
Version 2.0 with Smart Anti-Stuck System (SASS)

Features:
- Lawnmower path planning for coverage missions
- PID-based heading control
- LiDAR obstacle detection and avoidance
- Kalman-filtered drift estimation
- Smart anti-stuck with escape maneuvers
- Web dashboard integration
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import math
import numpy as np
import struct
import json

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, String


# =============================================================================
# CONTROL CONSTANTS
# =============================================================================
MAX_THRUST = 2000.0         # Newtons - hardware limit (v2.1: increased from 1000)
SAFE_THRUST = 1600.0         # Newtons - operational limit (v2.1: increased from 800)
SENSOR_TIMEOUT = 2.0        # seconds before sensor marked invalid


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
# │   Example: "Where is my boat?"                                              │
# │     Prior:      Before GPS reading, I think I'm somewhere near (x,y)        │
# │     Likelihood: GPS says I'm at position z with some accuracy               │
# │     Posterior:  Combining both, my best estimate of true position           │
# │                                                                             │
# │   This is the foundation of ALL probabilistic robotics!                     │
# └─────────────────────────────────────────────────────────────────────────────┘
#
# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                   KALMAN FILTER = BAYESIAN + GAUSSIAN                       │
# │                                                                             │
# │   For continuous states with Gaussian (bell curve) distributions:           │
# │     - Instead of P(x), we track μ (mean) and σ² (variance)                  │
# │     - Multiplying two Gaussians gives another Gaussian                      │
# │     - This makes Bayes' theorem computable in closed form!                  │
# │                                                                             │
# │   Why Gaussians?                                                            │
# │     1. Central Limit Theorem: Sum of many small errors → Gaussian           │
# │     2. Mathematically tractable: Products and sums stay Gaussian            │
# │     3. Only need 2 parameters (μ, σ²) to fully describe distribution        │
# │                                                                             │
# │   Kalman Filter is OPTIMAL for linear systems with Gaussian noise!          │
# └─────────────────────────────────────────────────────────────────────────────┘
#
# =============================================================================
# KALMAN FILTER FOR DRIFT ESTIMATION
# =============================================================================
# 
# A Kalman filter is an optimal recursive estimator that combines:
# 1. A prediction model (how we expect the system to evolve)
# 2. Noisy measurements (what we actually observe)
# 
# For drift estimation, our state is [drift_x, drift_y] - the water/wind current
# pushing our boat. This drift is typically slowly varying, so our model assumes
# the drift stays roughly constant between updates (random walk model).
#
# The Kalman filter maintains:
# - x: state estimate (our best guess of drift)
# - P: error covariance (how uncertain we are about x)
#
# Each cycle:
# 1. PREDICT: Project state forward using our model
#    x_pred = F @ x  (F is state transition matrix)
#    P_pred = F @ P @ F.T + Q  (Q is process noise - models how drift changes)
#
# 2. UPDATE: Correct prediction using measurement
#    K = P_pred @ H.T @ inv(H @ P_pred @ H.T + R)  (K is Kalman gain)
#    x = x_pred + K @ (z - H @ x_pred)  (z is measurement)
#    P = (I - K @ H) @ P_pred
#
# Key insight: Kalman gain K balances trust between prediction and measurement.
# High measurement noise R → trust prediction more
# High process noise Q → trust measurement more
# =============================================================================

# =============================================================================
# TRIPLE-REDUNDANCY SENSOR VOTING
# =============================================================================
# 2-of-3 voting for critical navigation decisions

class TripleRedundancyVoter:
    """
    2-of-3 Voting System
    
    Implements triple-redundancy for critical measurements.
    Used for obstacle detection, position validation, and heading consensus.
    
    Voting Logic:
    - All 3 agree -> HIGH confidence, use average
    - 2 of 3 agree -> MEDIUM confidence, use agreeing pair average
    - All 3 disagree -> LOW confidence, use most conservative value
    - Any sensor timeout -> mark as DEGRADED, continue with remaining
    """
    
    def __init__(self, agreement_threshold=0.2, timeout=2.0):
        """
        Args:
            agreement_threshold: Maximum difference for sensors to "agree" (relative)
            timeout: Seconds before sensor marked as failed
        """
        self.agreement_threshold = agreement_threshold
        self.timeout = timeout
        self.channels = [None, None, None]  # Three sensor channels
        self.timestamps = [0.0, 0.0, 0.0]   # Last update time per channel
        self.valid = [False, False, False]   # Channel validity flags
        
    def update_channel(self, channel_id: int, value: float, timestamp: float):
        """Update a sensor channel with new reading"""
        if 0 <= channel_id < 3:
            self.channels[channel_id] = value
            self.timestamps[channel_id] = timestamp
            self.valid[channel_id] = True
            
    def check_timeouts(self, current_time: float):
        """Mark channels as invalid if they haven't updated recently"""
        for i in range(3):
            if current_time - self.timestamps[i] > self.timeout:
                self.valid[i] = False
                
    def vote(self, current_time: float) -> tuple:
        """
        Perform 2-of-3 voting on current sensor readings.
        
        Returns:
            (voted_value, confidence, valid_channels)
            confidence: 'HIGH', 'MEDIUM', 'LOW', or 'FAILED'
        """
        self.check_timeouts(current_time)
        
        # Collect valid readings
        valid_readings = []
        for i in range(3):
            if self.valid[i] and self.channels[i] is not None:
                valid_readings.append((i, self.channels[i]))
        
        n_valid = len(valid_readings)
        
        if n_valid == 0:
            # SYSTEM FAILURE - no valid sensors
            return (float('inf'), 'FAILED', 0)
            
        elif n_valid == 1:
            # DEGRADED MODE - single sensor
            return (valid_readings[0][1], 'LOW', 1)
            
        elif n_valid == 2:
            # Check if the two agree
            v1, v2 = valid_readings[0][1], valid_readings[1][1]
            avg = (v1 + v2) / 2.0
            diff = abs(v1 - v2) / max(avg, 0.001)
            
            if diff <= self.agreement_threshold:
                return (avg, 'MEDIUM', 2)
            else:
                # Disagreement - use more conservative (smaller distance = closer obstacle)
                return (min(v1, v2), 'LOW', 2)
                
        else:  # n_valid == 3
            v1, v2, v3 = [r[1] for r in valid_readings]
            avg = (v1 + v2 + v3) / 3.0
            
            # Check pairwise agreement
            d12 = abs(v1 - v2) / max((v1 + v2) / 2, 0.001)
            d13 = abs(v1 - v3) / max((v1 + v3) / 2, 0.001)
            d23 = abs(v2 - v3) / max((v2 + v3) / 2, 0.001)
            
            agreements = sum([d12 <= self.agreement_threshold,
                             d13 <= self.agreement_threshold,
                             d23 <= self.agreement_threshold])
            
            if agreements >= 2:  # At least 2 pairs agree → HIGH confidence
                return (avg, 'HIGH', 3)
            elif agreements == 1:  # One pair agrees
                # Find the agreeing pair and use their average
                if d12 <= self.agreement_threshold:
                    return ((v1 + v2) / 2, 'MEDIUM', 3)
                elif d13 <= self.agreement_threshold:
                    return ((v1 + v3) / 2, 'MEDIUM', 3)
                else:
                    return ((v2 + v3) / 2, 'MEDIUM', 3)
            else:  # No agreement - use most conservative
                return (min(v1, v2, v3), 'LOW', 3)


class KalmanDriftEstimator:
    """
    Kalman Filter for Drift Estimation
    
    2D Kalman Filter for estimating water/wind drift affecting the boat.
    
    State vector: [drift_x, drift_y]
    These represent the velocity components (m/s) of environmental forces
    pushing the boat off course.
    
    Tuning:
    - Process noise Q: How much we expect drift to change between updates
      (small Q = assume drift is very stable, large Q = drift can change quickly)
    - Measurement noise R: How noisy our velocity/position measurements are
      (small R = trust measurements, large R = trust predictions)
    """
    
    def __init__(self, process_noise=0.01, measurement_noise=0.5):
        """
        Initialize the Kalman filter for drift estimation.
        
        Args:
            process_noise: Variance of drift change per timestep (Q diagonal)
                          Small value (~0.01) = assume drift changes slowly
                          Large value (~0.1) = drift can change quickly
            measurement_noise: Variance of velocity measurements (R diagonal)
                              Small value (~0.1) = GPS/IMU very accurate
                              Large value (~1.0) = sensors quite noisy
        """
        # State dimension: 2 (drift_x, drift_y)
        self.n = 2
        
        # --- STATE ESTIMATE ---
        # x: Current best estimate of drift [drift_x, drift_y]
        # Start with zero drift assumption
        self.x = np.zeros((self.n, 1))
        
        # --- ERROR COVARIANCE ---
        # P: Uncertainty in our state estimate
        # Start with high uncertainty (we don't know initial drift)
        # This 2x2 matrix represents variance in each component and their correlation
        self.P = np.eye(self.n) * 1.0  # Initial uncertainty = 1 m²/s²
        
        # --- STATE TRANSITION MODEL ---
        # F: How state evolves over time
        # For drift, we use a random walk model: drift stays constant + small noise
        # x(k+1) = F @ x(k) + w, where w ~ N(0, Q)
        self.F = np.eye(self.n)  # Identity = drift doesn't change systematically
        
        # --- MEASUREMENT MODEL ---
        # H: How measurements relate to state
        # We directly observe drift through position/velocity errors
        # z = H @ x + v, where v ~ N(0, R)
        self.H = np.eye(self.n)  # We measure drift directly
        
        # --- PROCESS NOISE COVARIANCE ---
        # Q: Uncertainty in our motion model
        # Larger Q = drift can change more between updates = more responsive
        # Smaller Q = drift assumed more stable = smoother estimates
        self.Q = np.eye(self.n) * process_noise
        
        # --- MEASUREMENT NOISE COVARIANCE ---
        # R: Uncertainty in our measurements
        # Larger R = measurements less reliable = filter more sluggish
        # Smaller R = measurements more trusted = filter more responsive
        self.R = np.eye(self.n) * measurement_noise
        
        # Store for diagnostics
        self.last_innovation = np.zeros((self.n, 1))  # Measurement surprise
        self.last_kalman_gain = np.zeros((self.n, self.n))
        
    def predict(self, dt=0.1):
        """
        PREDICTION STEP: Project state and covariance forward in time.
        
        This is the "prior" in Bayesian terms - our belief before seeing data.
        
        Args:
            dt: Time step (currently unused since we assume constant drift,
                but could scale Q for variable-rate updates)
        
        Math:
            x_pred = F @ x     (state prediction)
            P_pred = F @ P @ F.T + Q  (covariance prediction)
        
        The process noise Q gets added, representing our uncertainty about
        how drift might have changed since last update.
        """
        # State prediction: x(k|k-1) = F @ x(k-1|k-1)
        # For random walk: x stays the same (drift is persistent)
        self.x = self.F @ self.x
        
        # Covariance prediction: P(k|k-1) = F @ P(k-1|k-1) @ F.T + Q
        # Uncertainty grows due to process noise
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z):
        """
        UPDATE STEP: Correct prediction using measurement.
        
        This is the "posterior" in Bayesian terms - our belief after seeing data.
        The Kalman gain K optimally weights prediction vs measurement.
        
        Args:
            z: Measurement vector [measured_drift_x, measured_drift_y]
               This comes from comparing expected vs actual position change.
        
        Math:
            Innovation: y = z - H @ x  (measurement surprise)
            Innovation covariance: S = H @ P @ H.T + R
            Kalman gain: K = P @ H.T @ S^(-1)
            State update: x = x + K @ y
            Covariance update: P = (I - K @ H) @ P
        
        Key insight: If measurement noise R is large, K is small, and we
        mostly trust our prediction. If R is small, K is large, and we
        mostly trust the measurement.
        """
        # Ensure z is column vector
        z = np.array(z).reshape((self.n, 1))
        
        # --- INNOVATION (MEASUREMENT RESIDUAL) ---
        # y = z - H @ x: How much measurement differs from prediction
        # Large innovation = big surprise = either real drift change or noise
        y = z - self.H @ self.x
        self.last_innovation = y.copy()
        
        # --- INNOVATION COVARIANCE ---
        # S = H @ P @ H.T + R: Combined uncertainty from prediction and measurement
        S = self.H @ self.P @ self.H.T + self.R
        
        # --- KALMAN GAIN ---
        # K = P @ H.T @ S^(-1): Optimal weighting factor
        # K near 1 = trust measurement, K near 0 = trust prediction
        # Note: np.linalg.solve is numerically more stable than explicit inverse
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.last_kalman_gain = K.copy()
        
        # --- STATE UPDATE ---
        # x = x + K @ y: Correct prediction with weighted innovation
        self.x = self.x + K @ y
        
        # --- COVARIANCE UPDATE ---
        # P = (I - K @ H) @ P: Reduce uncertainty (we gained information)
        I = np.eye(self.n)
        self.P = (I - K @ self.H) @ self.P
        
    def get_drift(self):
        """
        Get current drift estimate as a tuple.
        
        Returns:
            (drift_x, drift_y): Estimated drift velocity in m/s
        """
        return (float(self.x[0, 0]), float(self.x[1, 0]))
    
    def get_uncertainty(self):
        """
        Get current estimation uncertainty (standard deviation).
        
        Returns:
            (std_x, std_y): Standard deviation of drift estimate
            Useful for confidence intervals: true_drift ≈ estimate ± 2*std (95%)
        """
        return (float(np.sqrt(self.P[0, 0])), float(np.sqrt(self.P[1, 1])))
    
    def reset(self, initial_drift=(0.0, 0.0)):
        """
        Reset filter to initial state.
        
        Args:
            initial_drift: Starting drift estimate
        """
        self.x = np.array([[initial_drift[0]], [initial_drift[1]]])
        self.P = np.eye(self.n) * 1.0  # Reset uncertainty
        
    def set_noise_params(self, process_noise=None, measurement_noise=None):
        """
        Tune filter responsiveness.
        
        Args:
            process_noise: Higher = more responsive to changes, noisier output
            measurement_noise: Higher = smoother output, slower response
            
        Tuning guide:
        - If drift estimate is too noisy: increase measurement_noise
        - If drift estimate responds too slowly: increase process_noise
        - If drift estimate overshoots: decrease process_noise
        """
        if process_noise is not None:
            self.Q = np.eye(self.n) * process_noise
        if measurement_noise is not None:
            self.R = np.eye(self.n) * measurement_noise


class Vostok1(Node):
    def __init__(self):
        super().__init__('vostok1_node')

        # --- CONFIGURATION PARAMETERS ---
        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 50.0)
        self.declare_parameter('lanes', 8)
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 1.0)        # Reduced from 2.0 - stricter tolerance
        self.declare_parameter('waypoint_skip_timeout', 20.0)    # Reduced from 45s - faster skip

        # PID Controller gains
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)

        # Obstacle avoidance parameters
        self.declare_parameter('min_safe_distance', 15.0)        # Increased from 12.0
        self.declare_parameter('critical_distance', 10.0)        # Increased from 5.0 for harder stops
        self.declare_parameter('obstacle_slow_factor', 0.3)
        self.declare_parameter('hysteresis_distance', 2.0)  # Exit threshold offset
        self.declare_parameter('reverse_timeout', 5.0)  # Max seconds to reverse
        
        # OKO v2.0 Enhanced LiDAR parameters (tuned for faster response)
        self.declare_parameter('temporal_history_size', 3)  # Reduced: faster response
        self.declare_parameter('temporal_threshold', 2)  # Reduced: 2/3 detections to confirm
        self.declare_parameter('cluster_distance', 2.0)  # Distance threshold for clustering
        self.declare_parameter('min_cluster_size', 3)  # Minimum points for valid cluster
        self.declare_parameter('water_plane_threshold', -2.5)  # Z threshold for water removal
        self.declare_parameter('velocity_history_size', 5)  # Frames for velocity estimation

        # Stuck detection parameters
        self.declare_parameter('stuck_timeout', 3.0)  # Seconds without movement to detect stuck
        self.declare_parameter('stuck_threshold', 0.5)  # Minimum movement in meters to not be stuck
        
        # Smart anti-stuck parameters
        self.declare_parameter('no_go_zone_radius', 10.0)        # Increased from 8.0
        self.declare_parameter('drift_compensation_gain', 0.3)  # How aggressively to compensate for drift
        self.declare_parameter('probe_angle', 45.0)  # Degrees to probe left/right during escape
        self.declare_parameter('detour_distance', 15.0)          # Increased from 12.0
        
        # Kalman filter parameters for drift estimation
        # See KalmanDriftEstimator class for detailed explanation
        self.declare_parameter('kalman_process_noise', 0.01)  # How fast drift can change
        self.declare_parameter('kalman_measurement_noise', 0.5)  # GPS/position sensor noise

        # Get parameters
        self.scan_length = self.get_parameter('scan_length').value
        self.scan_width = self.get_parameter('scan_width').value
        self.lanes = self.get_parameter('lanes').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.waypoint_skip_timeout = self.get_parameter('waypoint_skip_timeout').value

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
        
        # OKO v2.0 Enhanced LiDAR parameters
        self.temporal_history_size = self.get_parameter('temporal_history_size').value
        self.temporal_threshold = self.get_parameter('temporal_threshold').value
        self.cluster_distance = self.get_parameter('cluster_distance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.water_plane_threshold = self.get_parameter('water_plane_threshold').value
        self.velocity_history_size = self.get_parameter('velocity_history_size').value
        
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
        # Mission states: IDLE (waiting), WAYPOINTS_PREVIEW (waypoints defined, waiting for confirmation),
        #                 RUNNING (mission active), PAUSED (mission paused), FINISHED (completed)
        self.state = "IDLE"
        self.mission_armed = False  # User must arm/start the mission
        self.joystick_override = False  # Joystick control active

        # Obstacle avoidance state
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.avoidance_mode = False
        self.reverse_start_time = None
        
        # OKO v2.0 Enhanced detection state
        self.obstacle_history = []  # Temporal filter history
        self.cluster_centroids = []  # Current obstacle clusters
        self.urgency = 0.0  # Distance-weighted urgency (0.0-1.0)
        self.best_gap = None  # Best navigation gap
        self.velocity_history = []  # For obstacle velocity estimation
        self.obstacle_velocity = (0.0, 0.0)  # Estimated obstacle movement
        self.obstacle_count = 0  # Number of detected clusters

        # Stuck detection state
        self.last_position = None
        self.stuck_check_time = None
        self.is_stuck = False
        self.escape_mode = False
        self.escape_start_time = None
        self.consecutive_stuck_count = 0
        self.last_stuck_waypoint = None
        
        # Smart anti-stuck state
        self.no_go_zones = []  # List of (x, y, radius) zones to avoid
        self.escape_history = []  # List of {position, direction, success} for learning
        
        # Waypoint skip tracking (for obstacles blocking waypoint)
        self.waypoint_start_time = None
        self.obstacle_blocking_time = 0.0
        self.last_obstacle_check = None
        self.go_home_mode = False  # Track if we're in return-home mode
        self.home_detour_timeout = 15.0  # Insert detour after this many seconds of blocking in home mode
        
        # --- KALMAN FILTER FOR DRIFT ESTIMATION ---
        # Initialize Kalman filter with tunable noise parameters
        # Process noise: how quickly we expect drift to change (currents, wind shifts)
        # Measurement noise: how noisy our position/velocity measurements are
        kalman_process = self.get_parameter('kalman_process_noise').value
        kalman_measurement = self.get_parameter('kalman_measurement_noise').value
        self.drift_kalman = KalmanDriftEstimator(
            process_noise=kalman_process,
            measurement_noise=kalman_measurement
        )
        # Legacy tuple interface for backward compatibility
        # Now computed from Kalman filter state
        self.drift_vector = (0.0, 0.0)  # Updated by Kalman filter
        
        self.position_history = []  # Recent positions for drift estimation
        self.expected_positions = []  # Expected positions based on commands
        self.escape_phase = 0  # Current phase of escape maneuver
        self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}  # Probe clearances
        self.best_escape_direction = None  # Determined by probing
        self.detour_waypoint_inserted = False  # Track if we inserted a detour
        self.last_escape_position = None  # Where we were when escape started
        self.adaptive_escape_duration = 12.0  # Will be adjusted dynamically

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
        # Use standalone OKO perception node -> subscribe to /perception/obstacle_info
        # (OKO publishes a JSON String with obstacle fields)
        self.create_subscription(
            String,
            '/perception/obstacle_info',
            self.obstacle_callback,
            10
        )

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # Dashboard status publishers
        self.pub_mission_status = self.create_publisher(String, '/vostok1/mission_status', 10)
        self.pub_obstacle_status = self.create_publisher(String, '/vostok1/obstacle_status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/vostok1/anti_stuck_status', 10)
        
        # Waypoint publisher for RViz visualization and web dashboard
        self.pub_waypoints = self.create_publisher(String, '/vostok1/waypoints', 10)
        
        # Parameter configuration publisher (for web dashboard)
        self.pub_config = self.create_publisher(String, '/vostok1/config', 10)
        
        # Parameter update subscriber (from web dashboard)
        self.create_subscription(
            String,
            '/vostok1/set_config',
            self.config_callback,
            10
        )
        
        # Mission command subscriber (start/stop/pause/joystick)
        self.create_subscription(
            String,
            '/vostok1/mission_command',
            self.mission_command_callback,
            10
        )
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- CONTROL LOOP (20Hz for smoother control) ---
        self.dt = 0.05  # 20 Hz
        self.create_timer(self.dt, self.control_loop)
        
        # Publish config at 1Hz
        self.create_timer(1.0, self.publish_config)
        
        # Publish waypoints at 2Hz (for RViz visualizer)
        self.create_timer(0.5, self.publish_waypoints_periodic)
        
        # Publish anti-stuck status at 2Hz
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("PROJET-17 - Navigation Autonome")
        self.get_logger().info("Vostok 1 - Autonomous Navigation System")
        self.get_logger().info("+ Smart Anti-Stuck System (SASS) v2.0")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Scan Area: {self.scan_length}m x {self.scan_width * self.lanes}m")
        self.get_logger().info(f"Lanes: {self.lanes}, Width: {self.scan_width}m")
        self.get_logger().info(f"Speed: {self.base_speed} (max: {self.max_speed})")
        self.get_logger().info(f"PID: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info(f"Obstacle detection: Safe={self.min_safe_distance}m, Critical={self.critical_distance}m")
        self.get_logger().info(f"Anti-stuck: timeout={self.stuck_timeout}s, threshold={self.stuck_threshold}m")
        self.get_logger().info("Waiting for GPS signal and user command...")
        self.get_logger().info("=" * 60)

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)

        # First GPS fix - set start position but don't start mission
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Base point: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
            self.get_logger().info("GPS ready - awaiting waypoint configuration")
            self.get_logger().info("Use web dashboard to configure and start mission")
            self.get_logger().info("=" * 60)

    def imu_callback(self, msg):
        # Convert quaternion to yaw (heading)
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def obstacle_callback(self, msg):
        """Receive obstacle info published by standalone OKO node (/perception/obstacle_info).

        The OKO node publishes a JSON string matching the structure used across the system
        so integrated nodes (Vostok1/Atlantis) can rely on a single perception source.
        """
        try:
            data = json.loads(msg.data)

            # Support both min_distance and min_dist keys used in older/newer formats
            self.obstacle_detected = data.get('obstacle_detected', False)
            self.min_obstacle_distance = float(data.get('min_distance', data.get('min_dist', float('inf'))))
            self.front_clear = float(data.get('front_clear', data.get('front', self.front_clear)))
            self.left_clear = float(data.get('left_clear', data.get('left', self.left_clear)))
            self.right_clear = float(data.get('right_clear', data.get('right', self.right_clear)))
            self.is_critical = bool(data.get('is_critical', self.min_obstacle_distance < self.critical_distance))
            self.urgency = float(data.get('urgency', data.get('front_urgency', 0.0)))
            self.best_gap = data.get('best_gap', None)
            # clusters may be either 'clusters' list or count via obstacle_count
            if 'clusters' in data:
                self.cluster_centroids = data.get('clusters', [])
                self.obstacle_count = len(self.cluster_centroids)
            else:
                self.obstacle_count = int(data.get('obstacle_count', self.obstacle_count))

        except Exception as e:
            # Bad message — ignore and keep previous state
            self.get_logger().warn(f"Invalid perception message: {e}")
    
    # NOTE: Perception is handled by the standalone 'oko_perception' node which publishes
    # /perception/obstacle_info. Vostok1 now subscribes to that topic and does not run
    # its own PointCloud2 processing here.

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
        
        # Publish waypoints for RViz visualization
        self.publish_waypoints()

    def publish_waypoints(self):
        """Publish waypoints for RViz visualizer"""
        if not self.waypoints:
            return
        waypoint_data = {
            'waypoints': [{'x': wp[0], 'y': wp[1]} for wp in self.waypoints],
            'no_go_zones': [(z[0], z[1], z[2]) for z in self.no_go_zones] if self.no_go_zones else []
        }
        msg = String()
        msg.data = json.dumps(waypoint_data)
        self.pub_waypoints.publish(msg)
        
    def publish_waypoints_periodic(self):
        """Periodic waypoint publish (called by timer)"""
        self.publish_waypoints()

    def publish_config(self):
        """Publish current configuration for web dashboard"""
        config = {
            'scan_length': self.scan_length,
            'scan_width': self.scan_width,
            'lanes': self.lanes,
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'base_speed': self.base_speed,
            'max_speed': self.max_speed,
            'min_safe_distance': self.min_safe_distance,
            'waypoint_tolerance': self.waypoint_tolerance,
            'total_waypoints': len(self.waypoints),
            'current_waypoint': self.current_wp_index,
            'state': self.state,
            'mission_armed': self.mission_armed,
            'joystick_override': self.joystick_override,
            'gps_ready': self.start_gps is not None,
            'start_lat': self.start_gps[0] if self.start_gps else None,
            'start_lon': self.start_gps[1] if self.start_gps else None
        }
        msg = String()
        msg.data = json.dumps(config)
        self.pub_config.publish(msg)

    def config_callback(self, msg):
        """Handle configuration updates from web dashboard"""
        try:
            config = json.loads(msg.data)
            self.get_logger().info("=" * 50)
            self.get_logger().info("CONFIG UPDATE")
            
            # Track if path needs regeneration
            regenerate_path = False
            
            # Update path parameters
            if 'scan_length' in config and config['scan_length'] != self.scan_length:
                self.scan_length = float(config['scan_length'])
                self.get_logger().info(f"  Scan Length: {self.scan_length}m")
                regenerate_path = True
                
            if 'scan_width' in config and config['scan_width'] != self.scan_width:
                self.scan_width = float(config['scan_width'])
                self.get_logger().info(f"  Scan Width: {self.scan_width}m")
                regenerate_path = True
                
            if 'lanes' in config and config['lanes'] != self.lanes:
                self.lanes = int(config['lanes'])
                self.get_logger().info(f"  Lanes: {self.lanes}")
                regenerate_path = True
            
            # Update PID parameters (immediate effect)
            if 'kp' in config:
                self.kp = float(config['kp'])
                self.get_logger().info(f"  Kp: {self.kp}")
                
            if 'ki' in config:
                self.ki = float(config['ki'])
                self.integral_error = 0.0  # Reset integral on Ki change
                self.get_logger().info(f"  Ki: {self.ki}")
                
            if 'kd' in config:
                self.kd = float(config['kd'])
                self.get_logger().info(f"  Kd: {self.kd}")
            
            # Update speed parameters
            if 'base_speed' in config:
                self.base_speed = float(config['base_speed'])
                self.get_logger().info(f"  Base Speed: {self.base_speed}")
                
            if 'max_speed' in config:
                self.max_speed = float(config['max_speed'])
                self.get_logger().info(f"  Max Speed: {self.max_speed}")
            
            # Update safety parameters
            if 'min_safe_distance' in config:
                self.min_safe_distance = float(config['min_safe_distance'])
                self.get_logger().info(f"  Safe Distance: {self.min_safe_distance}m")
            
            # Handle mission restart (legacy support)
            if 'restart_mission' in config and config['restart_mission']:
                self.get_logger().info("MISSION RESTART")
                self.current_wp_index = 0
                self.state = "RUNNING"
                self.mission_armed = True
                self.integral_error = 0.0
                self.previous_error = 0.0
                regenerate_path = True
            
            # Regenerate waypoints if path parameters changed
            if regenerate_path and self.start_gps is not None:
                self.generate_lawnmower_path()
                self.state = "WAYPOINTS_PREVIEW"  # Show preview after regenerating
                self.get_logger().info(f"Path regenerated: {len(self.waypoints)} waypoints")
            
            self.get_logger().info("=" * 50)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Config parse error: {e}")
        except Exception as e:
            self.get_logger().error(f"Config update error: {e}")

    def parameter_callback(self, params):
        """Handle ROS 2 parameter changes"""
        for param in params:
            self.get_logger().info(f"Parameter changed: {param.name} = {param.value}")
            
            if param.name == 'kp':
                self.kp = param.value
            elif param.name == 'ki':
                self.ki = param.value
                self.integral_error = 0.0
            elif param.name == 'kd':
                self.kd = param.value
            elif param.name == 'scan_length':
                self.scan_length = param.value
            elif param.name == 'scan_width':
                self.scan_width = param.value
            elif param.name == 'lanes':
                self.lanes = param.value
            elif param.name == 'base_speed':
                self.base_speed = param.value
            elif param.name == 'max_speed':
                self.max_speed = param.value
            elif param.name == 'min_safe_distance':
                self.min_safe_distance = param.value
                
        return SetParametersResult(successful=True)

    def mission_command_callback(self, msg):
        """Handle mission commands from web dashboard"""
        try:
            cmd = json.loads(msg.data)
            command = cmd.get('command', '')
            
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"MISSION COMMAND: {command}")
            
            if command == 'generate_waypoints':
                # Generate/regenerate waypoints for preview
                if self.start_gps is not None:
                    self.generate_lawnmower_path()
                    self.state = "WAYPOINTS_PREVIEW"
                    self.mission_armed = False
                    self.get_logger().info(f"Waypoints generated: {len(self.waypoints)} points")
                    self.get_logger().info("State: WAYPOINTS_PREVIEW")
                else:
                    self.get_logger().warn("GPS not available - cannot generate waypoints")
                    
            elif command == 'confirm_waypoints':
                # User confirmed waypoints, ready to start
                if self.waypoints:
                    # If already running or paused, keep current state to avoid disabling thrusters
                    if self.state in ["RUNNING", "PAUSED"]:
                        self.get_logger().info("Waypoints already active - confirm ignored during mission")
                    else:
                        self.state = "READY"
                        self.mission_armed = False
                        self.get_logger().info("Waypoints CONFIRMED - Ready to start")
                else:
                    self.get_logger().warn("No waypoints to confirm")
                    
            elif command == 'cancel_waypoints':
                # Cancel waypoints, go back to idle
                self.waypoints = []
                self.current_wp_index = 0
                self.state = "IDLE"
                self.mission_armed = False
                self.get_logger().info("Waypoints CANCELLED")
                
            elif command == 'start_mission':
                # Start the mission (user gives permission)
                # Also allow starting from FINISHED state (restart with existing waypoints)
                if self.waypoints and self.state in ["READY", "PAUSED", "WAYPOINTS_PREVIEW", "FINISHED"]:
                    self.current_wp_index = 0 if self.state == "FINISHED" else self.current_wp_index
                    self.state = "RUNNING"
                    self.mission_armed = True
                    self.joystick_override = False
                    self.go_home_mode = False  # Reset home mode for normal mission
                    self.integral_error = 0.0
                    self.previous_error = 0.0
                    self.get_logger().info("MISSION STARTED!")
                else:
                    self.get_logger().warn(f"Cannot start - state={self.state}, waypoints={len(self.waypoints)}")
                    
            elif command == 'reset_mission':
                # Reset mission to IDLE state, clear waypoints
                self.waypoints = []
                self.current_wp_index = 0
                self.state = "IDLE"
                self.mission_armed = False
                # Reset all control state
                self.integral_error = 0.0
                self.previous_error = 0.0
                self.escape_mode = False
                self.is_stuck = False
                self.avoidance_mode = False
                self.reverse_start_time = None
                self.consecutive_stuck_count = 0
                self.stop_thrusters()
                self.get_logger().info("MISSION RESET!")
                    
            elif command == 'stop_mission':
                # Emergency stop - stop motors and pause
                self.state = "PAUSED"
                self.mission_armed = False
                # Reset escape/stuck mode on stop
                self.escape_mode = False
                self.is_stuck = False
                self.avoidance_mode = False
                self.reverse_start_time = None
                self.stop_thrusters()
                self.get_logger().info("MISSION STOPPED!")
                
            elif command == 'resume_mission':
                # Resume paused mission
                if self.state == "PAUSED" and self.waypoints:
                    self.state = "RUNNING"
                    self.mission_armed = True
                    self.get_logger().info("MISSION RESUMED!")
                    
            elif command == 'joystick_enable':
                # Enable joystick override mode
                self.joystick_override = True
                self.mission_armed = False
                self.state = "JOYSTICK"
                self.stop_thrusters()
                self.get_logger().info("JOYSTICK MODE ENABLED")
                self.get_logger().info("Run: ros2 launch vrx_gz usv_joy_teleop.launch.py")
                
            elif command == 'joystick_disable':
                # Disable joystick, return to previous state
                self.joystick_override = False
                if self.waypoints:
                    self.state = "PAUSED"
                else:
                    self.state = "IDLE"
                self.get_logger().info("JOYSTICK MODE DISABLED")
            
            elif command == 'go_home':
                # Navigate back to spawn point (one-click return home)
                if self.start_gps is not None:
                    # Clear current waypoints and set home as only waypoint
                    home_x, home_y = self.latlon_to_meters(self.start_gps[0], self.start_gps[1])
                    self.waypoints = [(home_x, home_y)]
                    self.current_wp_index = 0
                    self.state = "RUNNING"
                    self.mission_armed = True
                    self.joystick_override = False
                    self.integral_error = 0.0
                    self.previous_error = 0.0
                    self.go_home_mode = True  # Enable home mode for smarter obstacle handling
                    self.obstacle_blocking_time = 0.0
                    self.detour_waypoint_inserted = False
                    # Clear anti-stuck state for fresh return
                    self.no_go_zones = []
                    self.escape_mode = False
                    self.escape_phase = 0
                    self.get_logger().info("GOING HOME!")
                    self.get_logger().info(f"   Destination: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
                    self.get_logger().info(f"   Local position: ({home_x:.1f}m, {home_y:.1f}m)")
                else:
                    self.get_logger().warn("Cannot go home - no spawn point recorded")
                
            self.get_logger().info("=" * 50)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Mission command parse error: {e}")
        except Exception as e:
            self.get_logger().error(f"Mission command error: {e}")
    
    def stop_thrusters(self):
        """Emergency stop - set both thrusters to zero"""
        self.pub_left.publish(Float64(data=0.0))
        self.pub_right.publish(Float64(data=0.0))

    def control_loop(self):
        # Don't run control if not in RUNNING state or joystick override
        if self.state != "RUNNING" or self.joystick_override:
            if self.state == "PAUSED":
                self.stop_thrusters()  # Keep thrusters off when paused
            return
            
        if self.current_gps is None:
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
            self.advance_to_next_waypoint()
            self.total_distance += dist
            self.integral_error = 0.0
            return
        else:
            # Check if we should skip waypoint due to persistent obstacle
            self.check_waypoint_skip(curr_x, curr_y, dist)

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
                    f"CRITICAL OBSTACLE {self.min_obstacle_distance:.2f}m - Reversing!"
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
                self.send_thrust(-1600.0, -1600.0)  # Critical zone reverse
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
                self.get_logger().info("Obstacle avoidance mode - PID reset")

            self.avoidance_mode = True

            # OKO v2.0: Use best_gap for smarter navigation if available
            if self.best_gap and self.best_gap.get('width', 0) > 20:
                # Navigate towards the best gap direction
                gap_direction_deg = self.best_gap.get('direction', 0)
                gap_width = self.best_gap.get('width', 0)
                avoidance_heading = self.current_yaw + math.radians(gap_direction_deg)
                direction = f"GAP {gap_direction_deg:.0f}° ({gap_width:.0f}° wide)"
            elif self.left_clear > self.right_clear:
                # Fallback: Turn towards clearer side with urgency-scaled angle
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)  # 45° to 90°
                avoidance_heading = self.current_yaw + turn_angle
                direction = "LEFT"
            else:
                turn_angle = math.pi / 4 + (self.urgency * math.pi / 4)  # 45° to 90°
                avoidance_heading = self.current_yaw - turn_angle
                direction = "RIGHT"

            # Calculate error to avoidance heading
            angle_error = avoidance_heading - self.current_yaw

            # Normalize to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2.0 * math.pi
            while angle_error < -math.pi:
                angle_error += 2.0 * math.pi

            self.get_logger().warn(
                f"OBSTACLE! {self.min_obstacle_distance:.1f}m (urgency:{self.urgency*100:.0f}%) - "
                f"Turn {direction} (L:{self.left_clear:.1f}m R:{self.right_clear:.1f}m)",
                throttle_duration_sec=1.0
            )
        else:
            # NORMAL WAYPOINT NAVIGATION MODE
            if self.avoidance_mode:
                self.get_logger().info("Path CLEAR - Resuming navigation")
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
        turn_power = max(-1600.0, min(1600.0, turn_power))

        # Adaptive speed based on heading error
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
        left_thrust = max(-2000.0, min(2000.0, left_thrust))
        right_thrust = max(-2000.0, min(2000.0, right_thrust))

        # Send commands
        self.send_thrust(left_thrust, right_thrust)
        
        # Publish dashboard status
        self.publish_dashboard_status(curr_x, curr_y, target_x, target_y, dist)

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
        
        # Obstacle status
        if self.obstacle_detected:
            obs_status = f"OBS:{self.min_obstacle_distance:.1f}m"
        else:
            obs_status = "CLEAR"

        self.get_logger().info(
            f"WP {wp_progress} | "
            f"Pos: ({curr_x:.1f}, {curr_y:.1f}) | "
            f"Target: ({target_x:.1f}, {target_y:.1f}) | "
            f"Dist: {dist:.1f}m | "
            f"Error: {math.degrees(error):.1f}° | "
            f"{obs_status}"
        )

    def advance_to_next_waypoint(self):
        """Move to next waypoint and reset skip tracking"""
        self.current_wp_index += 1
        self.waypoint_start_time = None
        self.obstacle_blocking_time = 0.0
        self.last_obstacle_check = None
        self.detour_waypoint_inserted = False  # Reset for next waypoint

    def check_waypoint_skip(self, curr_x, curr_y, dist):
        """
        Check if we should skip waypoint due to persistent obstacle blocking.
        In go_home_mode: Insert detour waypoints instead of skipping.
        In normal mode: Skip to next waypoint after timeout or immediate cluster detection.
        """
        now = self.get_clock().now()
        
        # Initialize waypoint start time
        if self.waypoint_start_time is None:
            self.waypoint_start_time = now
            self.obstacle_blocking_time = 0.0
            self.last_obstacle_check = now
            return
        
        # Track obstacle blocking time if we're getting close to waypoint
        if dist < 30.0 and self.obstacle_detected:
            if self.last_obstacle_check is not None:
                dt = (now - self.last_obstacle_check).nanoseconds / 1e9
                self.obstacle_blocking_time += dt
        else:
            # Reset blocking time if no obstacle detected at close range
            if dist < 30.0:
                self.obstacle_blocking_time = 0.0
        
        self.last_obstacle_check = now
        
        # GO HOME MODE: Insert detours instead of skipping
        if self.go_home_mode:
            # Insert detour after shorter timeout (15s) to avoid circling
            if self.obstacle_blocking_time >= self.home_detour_timeout and not self.detour_waypoint_inserted:
                self.get_logger().warn(
                    f"🏠 HOME MODE: Obstacle blocking for {self.obstacle_blocking_time:.0f}s - Inserting detour"
                )
                self.insert_detour_waypoint(curr_x, curr_y)
                self.obstacle_blocking_time = 0.0  # Reset timer after inserting detour
            return  # Don't skip in home mode
        
        # NORMAL MODE: Check if we should skip
        # Skip condition 1: Timeout exceeded (faster response)
        timeout_exceeded = self.obstacle_blocking_time >= self.waypoint_skip_timeout
        
        # Skip condition 2: Obstacle detected AND we're in a tight cluster
        in_obstacle_cluster = dist < 8.0 and self.obstacle_detected
        
        if timeout_exceeded or in_obstacle_cluster:
            wp_num = self.current_wp_index + 1
            total_wp = len(self.waypoints)
            target_x, target_y = self.waypoints[self.current_wp_index]
            
            reason = "timeout" if timeout_exceeded else "obstacle_cluster"
            self.get_logger().warn(
                f"⏭️ SKIP WP {wp_num}/{total_wp} | Reason: {reason} | "
                f"Distance: {dist:.1f}m, Blocking: {self.obstacle_blocking_time:.1f}s "
                f"(target: ({target_x:.1f}, {target_y:.1f}))"
            )
            self.advance_to_next_waypoint()

    def finish_mission(self, final_x, final_y):
        """Complete the mission and log statistics"""
        arrived_home = self.go_home_mode
        self.state = "FINISHED"
        self.mission_armed = False
        self.stop_boat()
        self.go_home_mode = False  # Reset home mode after logging

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        elapsed_min = elapsed / 60.0

        self.get_logger().info("=" * 60)
        if arrived_home:
            self.get_logger().info("ARRIVED HOME!")
            # After returning home, clear waypoints and return to IDLE to allow fresh missions
            self.waypoints = []
            self.current_wp_index = 0
            self.state = "IDLE"
        else:
            self.get_logger().info("MISSION COMPLETE!")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Final position: ({final_x:.1f}m, {final_y:.1f}m)")
        self.get_logger().info(f"Total distance: {self.total_distance:.1f}m")
        self.get_logger().info(f"Mission duration: {elapsed_min:.1f} minutes")
        if elapsed > 0:
            avg_speed = self.total_distance / elapsed
            self.get_logger().info(f"Average speed: {avg_speed:.2f} m/s")
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
        """Detect if boat is stuck and hasn't moved significantly - ENHANCED with smart detection"""
        # Don't check if already in escape mode
        if self.escape_mode:
            return

        # Calculate distance moved since last check
        dx = curr_x - self.last_position[0]
        dy = curr_y - self.last_position[1]
        distance_moved = math.hypot(dx, dy)
        
        # Update position history for drift estimation
        self.position_history.append((curr_x, curr_y, self.get_clock().now()))
        if len(self.position_history) > 100:  # Keep last 100 samples (~5 seconds at 20Hz)
            self.position_history.pop(0)
        
        # Estimate drift from position history
        self.estimate_drift()

        # Check elapsed time
        elapsed = (self.get_clock().now() - self.stuck_check_time).nanoseconds / 1e9

        if elapsed >= self.stuck_timeout:
            # Time to check if stuck
            if distance_moved < self.stuck_threshold:
                if not self.is_stuck:
                    self.is_stuck = True
                    self.escape_mode = True
                    self.escape_start_time = self.get_clock().now()
                    self.escape_phase = 0  # Start with probing phase
                    self.integral_error = 0.0  # Reset PID
                    self.last_escape_position = (curr_x, curr_y)
                    self.best_escape_direction = None
                    self.probe_results = {'left': 0.0, 'right': 0.0, 'back': 0.0}
                    
                    # Add this location to no-go zones
                    self.add_no_go_zone(curr_x, curr_y)
                    
                    # Track consecutive stuck events at same waypoint
                    if self.last_stuck_waypoint == self.current_wp_index:
                        self.consecutive_stuck_count += 1
                    else:
                        self.consecutive_stuck_count = 1
                        self.last_stuck_waypoint = self.current_wp_index
                    
                    # Calculate adaptive escape duration based on situation
                    self.calculate_adaptive_escape_duration()
                    
                    self.get_logger().warn(
                        f"BOAT STUCK! Only {distance_moved:.2f}m moved in {elapsed:.1f}s | "
                        f"STUCK DETECTED! Smart escape initiating! (Tentative {self.consecutive_stuck_count}, "
                        f"Adaptive duration: {self.adaptive_escape_duration:.1f}s)"
                    )
                    
                    # Progressive escalation: after 2nd attempt, try inserting detour waypoint
                    if self.consecutive_stuck_count == 2 and not self.detour_waypoint_inserted:
                        self.insert_detour_waypoint(curr_x, curr_y)
                    
                    # Skip waypoint if stuck too many times (escalated to 4 with smart system)
                    if self.consecutive_stuck_count >= 4:
                        self.get_logger().error(
                            f"Stuck {self.consecutive_stuck_count} times at waypoint {self.current_wp_index + 1} - Skipping!"
                        )
                        self.current_wp_index += 1
                        self.consecutive_stuck_count = 0
                        self.detour_waypoint_inserted = False
                        self.is_stuck = False
                        self.escape_mode = False
            else:
                # Moved enough - not stuck
                if self.is_stuck:
                    # Record successful escape for learning
                    self.record_escape_result(success=True)
                    self.get_logger().info("No longer stuck - resuming normal operation (escape successful!)")
                self.is_stuck = False
                self.escape_mode = False
                self.escape_phase = 0

            # Reset tracking
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()

    def execute_escape_maneuver(self):
        """Execute SMART escape sequence with probing, adaptive duration, and drift compensation"""
        elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
        
        # Calculate phase boundaries based on adaptive duration
        # Phase 0: Probe (2s) - scan left/right/back to find best direction
        # Phase 1: Reverse (adaptive, ~40% of duration)
        # Phase 2: Turn (adaptive, ~35% of duration) 
        # Phase 3: Forward test with drift compensation (~25% of duration)
        probe_end = 2.0
        reverse_end = probe_end + self.adaptive_escape_duration * 0.4
        turn_end = reverse_end + self.adaptive_escape_duration * 0.35
        forward_end = turn_end + self.adaptive_escape_duration * 0.25
        
        # Apply drift compensation to thrust values
        drift_comp_left, drift_comp_right = self.calculate_drift_compensation()

        # PHASE 0: Multi-direction probe to find best escape route
        if elapsed < probe_end:
            self.escape_phase = 0
            probe_time = elapsed
            
            # Probe sequence: 0-0.6s look left, 0.6-1.2s look right, 1.2-2.0s assess back
            if probe_time < 0.6:
                # Probe left - gentle left turn while recording clearance
                self.send_thrust(-200.0, 200.0)
                self.probe_results['left'] = max(self.probe_results['left'], self.left_clear)
            elif probe_time < 1.2:
                # Probe right - gentle right turn while recording clearance
                self.send_thrust(200.0, -200.0)
                self.probe_results['right'] = max(self.probe_results['right'], self.right_clear)
            else:
                # Assess backward clearance (use front_clear as proxy when facing away)
                self.send_thrust(0.0, 0.0)  # Stop to stabilize
                self.probe_results['back'] = self.min_obstacle_distance  # Current front becomes back reference
                
                # Determine best escape direction from probe results
                if self.best_escape_direction is None:
                    self.best_escape_direction = self.determine_best_escape_direction()
                    self.get_logger().info(
                        f"Probe complete! L:{self.probe_results['left']:.1f}m R:{self.probe_results['right']:.1f}m "
                        f"B:{self.probe_results['back']:.1f}m → Best: {self.best_escape_direction}"
                    )
            
            self.get_logger().info(
                f"Escape Phase 0: PROBING ({probe_time:.1f}s) - Finding best escape route",
                throttle_duration_sec=0.5
            )
            return

        # PHASE 1: Reverse with adaptive power based on obstacle proximity
        elif elapsed < reverse_end:
            self.escape_phase = 1
            # More aggressive reverse if obstacle very close
            reverse_power = -700.0 - min(300.0, 100.0 / max(0.5, self.min_obstacle_distance))
            # Apply drift compensation
            self.send_thrust(reverse_power + drift_comp_left, reverse_power + drift_comp_right)
            self.get_logger().info(
                f"Escape Phase 1: Reversing SMART ({elapsed:.1f}s/{reverse_end:.1f}s) "
                f"Power: {reverse_power:.0f} + drift comp",
                throttle_duration_sec=1.0
            )
            return

        # PHASE 2: Turn using learned best direction
        elif elapsed < turn_end:
            self.escape_phase = 2
            turn_power = 600.0 + (self.consecutive_stuck_count * 100.0)  # Escalate turn power
            turn_power = min(900.0, turn_power)  # Cap at 900
            
            if self.best_escape_direction == 'LEFT':
                self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
                direction = "LEFT"
            elif self.best_escape_direction == 'RIGHT':
                self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
                direction = "RIGHT"
            else:  # BACK or unknown - use historical learning
                learned_direction = self.get_learned_escape_direction()
                if learned_direction == 'LEFT':
                    self.send_thrust(-turn_power + drift_comp_left, turn_power + drift_comp_right)
                    direction = "LEFT (learned)"
                else:
                    self.send_thrust(turn_power + drift_comp_left, -turn_power + drift_comp_right)
                    direction = "RIGHT (learned)"
            
            self.get_logger().info(
                f"Escape Phase 2: Turning {direction} ({elapsed:.1f}s/{turn_end:.1f}s) "
                f"Power: {turn_power:.0f} (escalation: {self.consecutive_stuck_count})",
                throttle_duration_sec=1.0
            )
            return

        # PHASE 3: Forward test with drift compensation
        elif elapsed < forward_end:
            self.escape_phase = 3
            # Check if we're heading toward a no-go zone
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            if self.is_heading_toward_no_go_zone(curr_x, curr_y):
                # Abort forward, turn more
                self.get_logger().warn("Forward path leads to no-go zone! Extra turn...")
                self.send_thrust(-400.0, 400.0)
            else:
                forward_power = 400.0 + drift_comp_left, 400.0 + drift_comp_right
                self.send_thrust(forward_power[0], forward_power[1])
            
            self.get_logger().info(
                f"Escape Phase 3: Forward test with drift compensation ({elapsed:.1f}s/{forward_end:.1f}s)",
                throttle_duration_sec=1.0
            )
            return

        else:
            # Escape complete - exit escape mode
            self.get_logger().info(
                f"Smart escape maneuver complete ({self.adaptive_escape_duration:.1f}s) - resuming navigation"
            )
            self.escape_mode = False
            self.is_stuck = False
            self.escape_phase = 0
            self.integral_error = 0.0
            self.previous_error = 0.0
            # Reset position tracking
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
            self.last_position = (curr_x, curr_y)
            self.stuck_check_time = self.get_clock().now()
    
    # ==================== SMART ANTI-STUCK HELPER METHODS ====================
    
    def calculate_adaptive_escape_duration(self):
        """Calculate escape duration based on situation severity"""
        base_duration = 10.0
        
        # Increase duration if obstacle very close
        if self.min_obstacle_distance < self.critical_distance:
            base_duration += 4.0
        elif self.min_obstacle_distance < self.min_safe_distance:
            base_duration += 2.0
        
        # Increase duration for consecutive stuck events (progressive escalation)
        base_duration += self.consecutive_stuck_count * 2.0
        
        # Cap at reasonable maximum
        self.adaptive_escape_duration = min(20.0, base_duration)
        
        self.get_logger().info(
            f"Adaptive escape duration: {self.adaptive_escape_duration:.1f}s "
            f"(obstacle: {self.min_obstacle_distance:.1f}m, attempts: {self.consecutive_stuck_count})"
        )
    
    def add_no_go_zone(self, x, y):
        """Add a no-go zone around a stuck location"""
        # Check if already have a zone nearby
        for zone in self.no_go_zones:
            if math.hypot(x - zone[0], y - zone[1]) < self.no_go_zone_radius:
                # Expand existing zone
                zone_idx = self.no_go_zones.index(zone)
                self.no_go_zones[zone_idx] = (zone[0], zone[1], zone[2] + 2.0)
                self.get_logger().info(f"Expanded no-go zone at ({zone[0]:.1f}, {zone[1]:.1f}) radius: {zone[2] + 2.0:.1f}m")
                return
        
        # Add new zone
        self.no_go_zones.append((x, y, self.no_go_zone_radius))
        self.get_logger().info(f"Added no-go zone #{len(self.no_go_zones)} at ({x:.1f}, {y:.1f}) radius: {self.no_go_zone_radius}m")
        
        # Limit number of zones to prevent memory issues
        if len(self.no_go_zones) > 20:
            self.no_go_zones.pop(0)  # Remove oldest
    
    def is_in_no_go_zone(self, x, y):
        """Check if a position is within any no-go zone"""
        for zone_x, zone_y, radius in self.no_go_zones:
            if math.hypot(x - zone_x, y - zone_y) < radius:
                return True
        return False
    
    def is_heading_toward_no_go_zone(self, curr_x, curr_y):
        """Check if current heading will lead into a no-go zone"""
        # Project position 10m forward along current heading
        look_ahead = 10.0
        future_x = curr_x + look_ahead * math.cos(self.current_yaw)
        future_y = curr_y + look_ahead * math.sin(self.current_yaw)
        return self.is_in_no_go_zone(future_x, future_y)
    
    def determine_best_escape_direction(self):
        """Determine best escape direction from probe results and history"""
        left = self.probe_results['left']
        right = self.probe_results['right']
        back = self.probe_results['back']
        
        # Check escape history for this area
        learned_bias = self.get_learned_escape_direction()
        
        # Significant difference threshold
        threshold = 3.0
        
        if left > right + threshold and left > back:
            return 'LEFT'
        elif right > left + threshold and right > back:
            return 'RIGHT'
        elif back > max(left, right) + threshold:
            return 'BACK'
        else:
            # No clear winner - use learning or default
            return learned_bias if learned_bias else 'LEFT'
    
    def estimate_drift(self):
        """
        Estimate current/wind drift using Kalman filter.
        
        The Kalman filter optimally combines:
        1. Our prediction (drift stays constant - random walk model)
        2. Our measurement (observed position change vs expected)
        
        This is superior to simple EMA because:
        - It properly handles measurement uncertainty
        - It tracks estimation confidence (covariance)
        - It provides optimal weighting via Kalman gain
        - It naturally handles variable update rates
        """
        if len(self.position_history) < 20:  # Need enough samples
            # Still run predict step to advance time (uncertainty grows)
            self.drift_kalman.predict()
            self.drift_vector = self.drift_kalman.get_drift()
            return
        
        # --- PREDICTION STEP ---
        # Advance our belief: drift stays same but uncertainty grows
        self.drift_kalman.predict()
        
        # --- MEASUREMENT STEP ---
        # Calculate observed drift from recent position changes
        # Compare actual movement to what we expected (based on thrust commands)
        recent = self.position_history[-20:]
        
        # Calculate average velocity over recent history
        total_dx = recent[-1][0] - recent[0][0]
        total_dy = recent[-1][1] - recent[0][1]
        time_diff = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        
        if time_diff > 0.5:  # At least 0.5 seconds of data
            # Measured drift = observed velocity
            # (In a full implementation, we'd subtract commanded velocity)
            measured_drift_x = total_dx / time_diff
            measured_drift_y = total_dy / time_diff
            
            # --- UPDATE STEP ---
            # Correct prediction with measurement
            # Kalman gain automatically weights based on uncertainties
            self.drift_kalman.update([measured_drift_x, measured_drift_y])
        
        # Update legacy tuple for backward compatibility
        self.drift_vector = self.drift_kalman.get_drift()
        
        # Optional: Log Kalman filter diagnostics
        # uncertainty = self.drift_kalman.get_uncertainty()
        # self.get_logger().debug(f'Drift: {self.drift_vector}, Uncertainty: {uncertainty}')
    
    def calculate_drift_compensation(self):
        """Calculate thrust compensation for estimated drift"""
        drift_magnitude = math.hypot(self.drift_vector[0], self.drift_vector[1])
        
        if drift_magnitude < 0.1:  # Negligible drift
            return 0.0, 0.0
        
        # Calculate drift direction relative to boat heading
        drift_angle = math.atan2(self.drift_vector[1], self.drift_vector[0])
        relative_drift = drift_angle - self.current_yaw
        
        # Normalize
        while relative_drift > math.pi:
            relative_drift -= 2.0 * math.pi
        while relative_drift < -math.pi:
            relative_drift += 2.0 * math.pi
        
        # Compensate: if drifting left, add right thrust bias
        compensation = drift_magnitude * self.drift_compensation_gain * 100.0  # Scale to thrust
        compensation = min(150.0, compensation)  # Cap compensation
        
        if relative_drift > 0:  # Drifting left
            return compensation, -compensation  # Bias right
        else:  # Drifting right
            return -compensation, compensation  # Bias left
    
    def insert_detour_waypoint(self, curr_x, curr_y):
        """Insert a detour waypoint perpendicular to obstacle - OKO v2.0 enhanced"""
        # OKO v2.0: Use best_gap for smarter detour direction
        if self.best_gap and self.best_gap.get('width', 0) > 30:
            # Use best gap direction for detour
            gap_direction_deg = self.best_gap.get('direction', 0)
            detour_angle = self.current_yaw + math.radians(gap_direction_deg)
            direction = f"GAP {gap_direction_deg:.0f}°"
        elif self.left_clear > self.right_clear:
            # Detour left
            detour_angle = self.current_yaw + math.pi / 2
            direction = "LEFT"
        else:
            # Detour right
            detour_angle = self.current_yaw - math.pi / 2
            direction = "RIGHT"
        
        # Calculate detour position
        detour_x = curr_x + self.detour_distance * math.cos(detour_angle)
        detour_y = curr_y + self.detour_distance * math.sin(detour_angle)
        
        # Check detour doesn't lead into no-go zone
        if self.is_in_no_go_zone(detour_x, detour_y):
            # Try opposite direction
            detour_angle += math.pi
            detour_x = curr_x + self.detour_distance * math.cos(detour_angle)
            detour_y = curr_y + self.detour_distance * math.sin(detour_angle)
        
        # Insert detour waypoint before current target
        self.waypoints.insert(self.current_wp_index, (detour_x, detour_y))
        self.detour_waypoint_inserted = True
        
        self.get_logger().warn(
            f"DETOUR! Inserting detour waypoint {direction} at ({detour_x:.1f}, {detour_y:.1f})"
        )
    
    def record_escape_result(self, success):
        """Record escape attempt result for learning"""
        if self.last_escape_position is None:
            return
        
        record = {
            'position': self.last_escape_position,
            'direction': self.best_escape_direction,
            'success': success,
            'consecutive_count': self.consecutive_stuck_count,
            'obstacle_distance': self.min_obstacle_distance
        }
        
        self.escape_history.append(record)
        
        # Limit history size
        if len(self.escape_history) > 50:
            self.escape_history.pop(0)
        
        self.get_logger().info(
            f"Escape {'SUCCESS' if success else 'FAILED'} recorded. "
            f"Direction: {self.best_escape_direction}, History: {len(self.escape_history)} records"
        )
    
    def get_learned_escape_direction(self):
        """Get best escape direction from historical data (simple learning)"""
        if not self.escape_history:
            return 'LEFT'  # Default
        
        # Count successful escapes by direction
        left_success = sum(1 for r in self.escape_history if r['direction'] == 'LEFT' and r['success'])
        right_success = sum(1 for r in self.escape_history if r['direction'] == 'RIGHT' and r['success'])
        
        # Weight recent successes more heavily
        recent = self.escape_history[-10:] if len(self.escape_history) >= 10 else self.escape_history
        left_recent = sum(1 for r in recent if r['direction'] == 'LEFT' and r['success'])
        right_recent = sum(1 for r in recent if r['direction'] == 'RIGHT' and r['success'])
        
        # Combined score (recent weighted 2x)
        left_score = left_success + left_recent * 2
        right_score = right_success + right_recent * 2
        
        if left_score > right_score:
            return 'LEFT'
        elif right_score > left_score:
            return 'RIGHT'
        else:
            return 'LEFT'  # Default tie-breaker
    
    def publish_anti_stuck_status(self):
        """Publish anti-stuck system status for web dashboard"""
        # Get Kalman filter uncertainty for diagnostics
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
            # New: Kalman filter diagnostics
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

    def stop_boat(self):
        """Stop all thrusters"""
        self.send_thrust(0.0, 0.0)
        self.get_logger().info("Boat stopped")

    def publish_dashboard_status(self, curr_x, curr_y, target_x, target_y, dist):
        """Publish status data for web dashboard"""
        import json
        
        # Mission status translations
        state_translations = {
            "STUCK_ESCAPING": "STUCK - ESCAPING",
            "OBSTACLE_AVOIDING": "OBSTACLE - AVOIDING",
            "MISSION_COMPLETE": "MISSION COMPLETE",
            "MOVING_TO_WAYPOINT": "EN ROUTE",
            "FINISHED": "FINISHED",
            "RUNNING": "RUNNING",
            "IDLE": "IDLE",
            "WAYPOINTS_PREVIEW": "PREVIEW",
            "READY": "READY",
            "PAUSED": "PAUSED",
            "JOYSTICK": "JOYSTICK"
        }
        
        if self.escape_mode:
            state_key = "STUCK_ESCAPING"
        elif self.avoidance_mode:
            state_key = "OBSTACLE_AVOIDING"
        elif self.state == "FINISHED":
            state_key = "MISSION_COMPLETE"
        elif self.state == "RUNNING":
            state_key = "MOVING_TO_WAYPOINT"
        else:
            state_key = self.state
            
        state_display = state_translations.get(state_key, state_key)
            
        mission_data = {
            "state": state_display,
            "waypoint": self.current_wp_index + 1,
            "total_waypoints": len(self.waypoints),
            "distance_to_waypoint": round(dist, 1)
        }
        
        mission_msg = String()
        mission_msg.data = json.dumps(mission_data)
        self.pub_mission_status.publish(mission_msg)
        
        # Obstacle status with bilingual messages - OKO v2.0 Enhanced
        obstacle_detected = self.min_obstacle_distance < self.min_safe_distance
        
        if obstacle_detected:
            status_text = f"OBSTACLE at {round(self.min_obstacle_distance, 1)}m"
        else:
            status_text = "PATH CLEAR" 
        
        obstacle_data = {
            "status": status_text,
            "min_distance": round(self.min_obstacle_distance, 1) if self.min_obstacle_distance != float('inf') else 999.9,
            "front_clear": bool(self.front_clear != float('inf')),
            "left_clear": bool(self.left_clear > self.min_safe_distance),
            "right_clear": bool(self.right_clear > self.min_safe_distance),
            "front_distance": round(self.front_clear, 1) if self.front_clear != float('inf') else 999.9,
            "left_distance": round(self.left_clear, 1) if self.left_clear != float('inf') else 999.9,
            "right_distance": round(self.right_clear, 1) if self.right_clear != float('inf') else 999.9,
            # OKO v2.0 Enhanced fields
            "obstacle_detected": bool(self.obstacle_detected),
            "is_critical": bool(self.min_obstacle_distance < self.critical_distance),
            "urgency": round(float(self.urgency), 2),
            "obstacle_count": int(self.obstacle_count),
            "clusters": [
                {"x": round(float(c[0]), 1), "y": round(float(c[1]), 1), "distance": round(float(c[2]), 1)}
                for c in self.cluster_centroids[:5]  # Limit to 5 clusters
            ],
            "best_gap": self.best_gap if self.best_gap else {"direction": 0.0, "width": 360.0, "distance": 100.0},
            "velocity_estimate": {
                "vx": round(float(self.obstacle_velocity[0]), 2),
                "vy": round(float(self.obstacle_velocity[1]), 2)
            }
        }
        
        obstacle_msg = String()
        obstacle_msg.data = json.dumps(obstacle_data)
        self.pub_obstacle_status.publish(obstacle_msg)

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
