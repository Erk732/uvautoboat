import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import math
import numpy as np
import struct
import json

from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import Float64, String

class Vostok1(Node):
    def __init__(self):
        super().__init__('vostok1_node')

        # --- CONFIGURATION PARAMETERS ---
        self.declare_parameter('scan_length', 15.0)
        self.declare_parameter('scan_width', 30.0)
        self.declare_parameter('lanes', 10)
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 2.0)

        # PID Controller gains
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)

        # Obstacle avoidance parameters
        self.declare_parameter('min_safe_distance', 15.0)
        self.declare_parameter('critical_distance', 5.0)
        self.declare_parameter('obstacle_slow_factor', 0.3)
        self.declare_parameter('hysteresis_distance', 2.0)  # Exit threshold offset
        self.declare_parameter('reverse_timeout', 5.0)  # Max seconds to reverse

        # Stuck detection parameters
        self.declare_parameter('stuck_timeout', 3.0)  # Seconds without movement to detect stuck
        self.declare_parameter('stuck_threshold', 0.5)  # Minimum movement in meters to not be stuck
        
        # Smart anti-stuck parameters
        self.declare_parameter('no_go_zone_radius', 8.0)  # Radius of no-go zones around stuck locations
        self.declare_parameter('drift_compensation_gain', 0.3)  # How aggressively to compensate for drift
        self.declare_parameter('probe_angle', 45.0)  # Degrees to probe left/right during escape
        self.declare_parameter('detour_distance', 12.0)  # Distance for detour waypoints

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
        self.no_go_zones = []  # List of (x, y, radius) zones to avoid
        self.escape_history = []  # List of {position, direction, success} for learning
        self.drift_vector = (0.0, 0.0)  # Estimated current/wind drift
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
        self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # Dashboard status publishers
        self.pub_mission_status = self.create_publisher(String, '/vostok1/mission_status', 10)
        self.pub_obstacle_status = self.create_publisher(String, '/vostok1/obstacle_status', 10)
        self.pub_anti_stuck = self.create_publisher(String, '/vostok1/anti_stuck_status', 10)
        
        # Parameter configuration publisher (for web dashboard)
        self.pub_config = self.create_publisher(String, '/vostok1/config', 10)
        
        # Parameter update subscriber (from web dashboard)
        self.create_subscription(
            String,
            '/vostok1/set_config',
            self.config_callback,
            10
        )
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- CONTROL LOOP (20Hz for smoother control) ---
        self.dt = 0.05  # 20 Hz
        self.create_timer(self.dt, self.control_loop)
        
        # Publish config at 1Hz
        self.create_timer(1.0, self.publish_config)
        
        # Publish anti-stuck status at 2Hz
        self.create_timer(0.5, self.publish_anti_stuck_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("ПРОЕКТ-17 (Proekt-17) - Автономная Навигация")
        self.get_logger().info("Vostok 1 - Autonomous Navigation System")
        self.get_logger().info("+ Smart Anti-Stuck System (SASS) v2.0")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Зона сканирования: {self.scan_length}m × {self.scan_width * self.lanes}m")
        self.get_logger().info(f"Полосы: {self.lanes}, Ширина: {self.scan_width}m")
        self.get_logger().info(f"Скорость: {self.base_speed} (макс: {self.max_speed})")
        self.get_logger().info(f"ПИД: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info(f"Обнаружение препятствий: Безопасно={self.min_safe_distance}m, Критично={self.critical_distance}m")
        self.get_logger().info(f"Анти-застревание: timeout={self.stuck_timeout}s, threshold={self.stuck_threshold}m")
        self.get_logger().info("Ожидание сигнала GPS...")
        self.get_logger().info("=" * 60)

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)

        # First GPS fix - initialize mission
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Базовая точка: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
            self.generate_lawnmower_path()
            self.state = "DRIVING"
            self.get_logger().info("МИССИЯ НАЧАТА! (Mission Started!)")
            self.get_logger().info("=" * 60)

    def imu_callback(self, msg):
        # Convert quaternion to yaw (heading)
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        """Process 3D LIDAR point cloud for obstacle detection"""
        # Extract points from PointCloud2 using proper iteration
        points = []
        
        # PointCloud2 structure: each point has x, y, z (and possibly intensity)
        # Assuming xyz fields are first 12 bytes (3 floats * 4 bytes each)
        point_step = msg.point_step
        row_step = msg.row_step
        data = msg.data
        
        # Process points - sample every 10th point (LIDAR has 3750 samples now)
        for i in range(0, len(data) - point_step, point_step * 10):
            try:
                # Extract x, y, z from point cloud data
                x, y, z = struct.unpack_from('fff', data, i)
                
                # Skip invalid points
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    continue
                if math.isinf(x) or math.isinf(y) or math.isinf(z):
                    continue
                
                # Filter by height - obstacles are between -0.2m (water surface) and 3.0m
                if z < -0.2 or z > 3.0:
                    continue
                
                # Calculate horizontal distance
                dist = math.sqrt(x*x + y*y)
                
                # Focus on relevant range: ignore boat itself and very far objects
                # Stricter filtering: must be at least 1.0m away AND in front (x > 0)
                if dist < 1.0 or dist > 100.0:
                    continue
                
                # Only consider points in front of the boat (positive x direction)
                # This prevents seeing the boat's own structure at wide angles
                if x < 0.5:
                    continue
                
                points.append((x, y, z, dist))
                    
            except struct.error:
                continue
            except Exception:
                continue
        
        if not points:
            # No valid points - assume clear but be cautious
            self.min_obstacle_distance = 50.0
            if not self.obstacle_detected:  # Don't immediately clear if we were detecting
                self.front_clear = 50.0
                self.left_clear = 50.0
                self.right_clear = 50.0
            return
        
        # Get minimum distance from all valid points
        distances = [p[3] for p in points]
        self.min_obstacle_distance = min(distances)

        # Hysteresis for obstacle detection (prevents flickering)
        if self.obstacle_detected:
            # Already in obstacle mode - need to be farther to exit
            exit_threshold = self.min_safe_distance + self.hysteresis_distance
            self.obstacle_detected = self.min_obstacle_distance < exit_threshold
        else:
            # Not in obstacle mode - enter if too close
            self.obstacle_detected = self.min_obstacle_distance < self.min_safe_distance

        # Analyze sectors for navigation
        self.analyze_scan_sectors_3d(points)

    def analyze_scan_sectors_3d(self, points):
        """Divide 3D point cloud into sectors to determine best direction"""
        # Divide points into sectors based on angle from boat's perspective
        # Front: -45° to +45°, Left: +45° to +135°, Right: -135° to -45°
        front_points = []
        left_points = []
        right_points = []
        
        for x, y, z, dist in points:
            angle = math.atan2(y, x)  # Angle in radians from boat's forward axis
            
            # Wider front sector for better obstacle detection ahead
            if -math.pi/4 < angle < math.pi/4:  # Front sector ±45°
                front_points.append(dist)
            elif math.pi/4 <= angle <= 3*math.pi/4:  # Left sector 45° to 135°
                left_points.append(dist)
            elif -3*math.pi/4 <= angle < -math.pi/4:  # Right sector -135° to -45°
                right_points.append(dist)
        
        # For each sector, find the minimum distance (closest obstacle)
        # Use 10th percentile for robustness against noise
        max_range = 100.0  # meters - matches LIDAR max_range
        
        if front_points:
            sorted_front = sorted(front_points)
            # Use 10th percentile or minimum if few points
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
            'state': self.state
        }
        msg = String()
        msg.data = json.dumps(config)
        self.pub_config.publish(msg)

    def config_callback(self, msg):
        """Handle configuration updates from web dashboard"""
        try:
            config = json.loads(msg.data)
            self.get_logger().info("=" * 50)
            self.get_logger().info("ОБНОВЛЕНИЕ КОНФИГУРАЦИИ | CONFIG UPDATE")
            
            # Track if path needs regeneration
            regenerate_path = False
            
            # Update path parameters
            if 'scan_length' in config and config['scan_length'] != self.scan_length:
                self.scan_length = float(config['scan_length'])
                self.get_logger().info(f"  Длина скана | Scan Length: {self.scan_length}m")
                regenerate_path = True
                
            if 'scan_width' in config and config['scan_width'] != self.scan_width:
                self.scan_width = float(config['scan_width'])
                self.get_logger().info(f"  Ширина скана | Scan Width: {self.scan_width}m")
                regenerate_path = True
                
            if 'lanes' in config and config['lanes'] != self.lanes:
                self.lanes = int(config['lanes'])
                self.get_logger().info(f"  Полосы | Lanes: {self.lanes}")
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
                self.get_logger().info(f"  Базовая скорость | Base Speed: {self.base_speed}")
                
            if 'max_speed' in config:
                self.max_speed = float(config['max_speed'])
                self.get_logger().info(f"  Макс. скорость | Max Speed: {self.max_speed}")
            
            # Update safety parameters
            if 'min_safe_distance' in config:
                self.min_safe_distance = float(config['min_safe_distance'])
                self.get_logger().info(f"  Безопасная дистанция | Safe Distance: {self.min_safe_distance}m")
            
            # Handle mission restart
            if 'restart_mission' in config and config['restart_mission']:
                self.get_logger().info("ПЕРЕЗАПУСК МИССИИ | MISSION RESTART")
                self.current_wp_index = 0
                self.state = "DRIVING"
                self.integral_error = 0.0
                self.previous_error = 0.0
                regenerate_path = True
            
            # Regenerate waypoints if path parameters changed
            if regenerate_path and self.start_gps is not None:
                self.generate_lawnmower_path()
                self.get_logger().info(f"Маршрут перестроен | Path regenerated: {len(self.waypoints)} waypoints")
            
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

    def control_loop(self):
        if self.state != "DRIVING" or self.current_gps is None:
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
                f"Точка {self.current_wp_index + 1}/{len(self.waypoints)} "
                f"достигнута | Waypoint reached at ({curr_x:.1f}, {curr_y:.1f})"
            )
            self.current_wp_index += 1
            self.total_distance += dist
            self.integral_error = 0.0
            return

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
                    f"КРИТИЧЕСКОЕ ПРЕПЯТСТВИЕ {self.min_obstacle_distance:.2f}m - Реверс! | CRITICAL OBSTACLE - Reversing!"
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
                self.send_thrust(-800.0, -800.0) # change the reverse thrust?
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
                self.get_logger().info("Режим обхода препятствий - Сброс ПИД | Obstacle avoidance mode - PID reset")

            self.avoidance_mode = True

            # Determine which direction has more clearance
            if self.left_clear > self.right_clear:
                # Turn hard left (90 degrees from current heading)
                avoidance_heading = self.current_yaw + math.pi / 2
                direction = "LEFT"
            else:
                # Turn hard right (90 degrees from current heading)
                avoidance_heading = self.current_yaw - math.pi / 2
                direction = "RIGHT"

            # Calculate error to avoidance heading
            angle_error = avoidance_heading - self.current_yaw

            # Normalize to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2.0 * math.pi
            while angle_error < -math.pi:
                angle_error += 2.0 * math.pi

            self.get_logger().warn(
                f"🚨 ПРЕПЯТСТВИЕ! {self.min_obstacle_distance:.1f}m - Поворот {direction} | "
                f"OBSTACLE DETECTED! Turning {direction} (Left:{self.left_clear:.1f}m Right:{self.right_clear:.1f}m)",
                throttle_duration_sec=1.0
            )
        else:
            # NORMAL WAYPOINT NAVIGATION MODE
            if self.avoidance_mode:
                self.get_logger().info("✅ Путь свободен - Возврат к навигации | Path CLEAR - Resuming navigation")
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
        turn_power = max(-800.0, min(800.0, turn_power))

        # Adaptive speed
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
        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))

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
        
        # Bilingual obstacle status
        if self.obstacle_detected:
            obs_status = f"ПРЕПЯТСТВИЕ:{self.min_obstacle_distance:.1f}m | OBS:{self.min_obstacle_distance:.1f}m"
        else:
            obs_status = "СВОБОДНО | CLEAR"
        
        self.get_logger().info(
            f"ТМ {wp_progress} | "  # ТМ = Точка Маршрута (Waypoint)
            f"Поз: ({curr_x:.1f}, {curr_y:.1f}) | "  # Поз = Позиция (Position)
            f"Цель: ({target_x:.1f}, {target_y:.1f}) | "  # Цель = Target
            f"Дист: {dist:.1f}m | "  # Дист = Дистанция (Distance)
            f"Ошибка: {math.degrees(error):.1f}° | "  # Ошибка = Error
            f"{obs_status}"
        )

    def finish_mission(self, final_x, final_y):
        """Complete the mission and log statistics"""
        self.state = "FINISHED"
        self.stop_boat()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        elapsed_min = elapsed / 60.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("МИССИЯ ЗАВЕРШЕНА! (MISSION COMPLETE!)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Конечная позиция: ({final_x:.1f}m, {final_y:.1f}m)")
        self.get_logger().info(f"Общая дистанция: {self.total_distance:.1f}m")
        self.get_logger().info(f"Время миссии: {elapsed_min:.1f} минут")
        if elapsed > 0:
            avg_speed = self.total_distance / elapsed
            self.get_logger().info(f"Средняя скорость: {avg_speed:.2f} м/с")
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
                        f"ЛОДКА ЗАСТРЯЛА! Пройдено только {distance_moved:.2f}m за {elapsed:.1f}s | "
                        f"STUCK DETECTED! Smart escape initiating! (Попытка {self.consecutive_stuck_count}, "
                        f"Adaptive duration: {self.adaptive_escape_duration:.1f}s)"
                    )
                    
                    # Progressive escalation: after 2nd attempt, try inserting detour waypoint
                    if self.consecutive_stuck_count == 2 and not self.detour_waypoint_inserted:
                        self.insert_detour_waypoint(curr_x, curr_y)
                    
                    # Skip waypoint if stuck too many times (escalated to 4 with smart system)
                    if self.consecutive_stuck_count >= 4:
                        self.get_logger().error(
                            f"Застряла {self.consecutive_stuck_count} раз на точке {self.current_wp_index + 1} | "
                            f"Stuck {self.consecutive_stuck_count} times - Пропуск точки! Skipping waypoint!"
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
        """Estimate current/wind drift from position history"""
        if len(self.position_history) < 20:  # Need enough samples
            return
        
        # Compare recent movement to expected movement
        # Simple approach: look at movement when thrust was minimal
        recent = self.position_history[-20:]
        
        # Calculate average velocity over recent history
        total_dx = recent[-1][0] - recent[0][0]
        total_dy = recent[-1][1] - recent[0][1]
        time_diff = (recent[-1][2] - recent[0][2]).nanoseconds / 1e9
        
        if time_diff > 0.5:  # At least 0.5 seconds of data
            # Smooth drift estimate with exponential moving average
            alpha = 0.1  # Smoothing factor
            new_drift_x = total_dx / time_diff
            new_drift_y = total_dy / time_diff
            
            self.drift_vector = (
                alpha * new_drift_x + (1 - alpha) * self.drift_vector[0],
                alpha * new_drift_y + (1 - alpha) * self.drift_vector[1]
            )
    
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
        """Insert a detour waypoint perpendicular to obstacle"""
        # Determine detour direction based on clearance
        if self.left_clear > self.right_clear:
            # Detour left
            detour_angle = self.current_yaw + math.pi / 2
        else:
            # Detour right
            detour_angle = self.current_yaw - math.pi / 2
        
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
        
        direction = "LEFT" if self.left_clear > self.right_clear else "RIGHT"
        self.get_logger().warn(
            f"ОБХОДНОЙ МАРШРУТ! Inserting detour waypoint {direction} at ({detour_x:.1f}, {detour_y:.1f})" # We can remove the russian easter egg? 
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

    def stop_boat(self):
        """Stop all thrusters"""
        self.send_thrust(0.0, 0.0)
        self.get_logger().info("Boat stopped")

    def publish_dashboard_status(self, curr_x, curr_y, target_x, target_y, dist):
        """Publish status data for web dashboard"""
        import json
        
        # Mission status with bilingual state messages
        state_translations = {
            "STUCK_ESCAPING": "ЗАСТРЯЛ - МАНЕВР ОСВОБОЖДЕНИЯ | STUCK - ESCAPING", # We can remove the russian easter egg? 
            "OBSTACLE_AVOIDING": "ПРЕПЯТСТВИЕ - ОБХОД | OBSTACLE - AVOIDING", # We can remove the russian easter egg? 
            "MISSION_COMPLETE": "МИССИЯ ЗАВЕРШЕНА | MISSION COMPLETE", # We can remove the russian easter egg? 
            "MOVING_TO_WAYPOINT": "ДВИЖЕНИЕ К ТОЧКЕ | MOVING TO WAYPOINT", # We can remove the russian easter egg? 
            "FINISHED": "ЗАВЕРШЕНО | FINISHED", # We can remove the russian easter egg? 
            "DRIVING": "ДВИЖЕНИЕ | DRIVING", # We can remove the russian easter egg? 
            "INIT": "ИНИЦИАЛИЗАЦИЯ | INITIALIZING" # We can remove the russian easter egg?  
        }
        
        if self.escape_mode:
            state_key = "STUCK_ESCAPING"
        elif self.avoidance_mode:
            state_key = "OBSTACLE_AVOIDING"
        elif self.state == "FINISHED":
            state_key = "MISSION_COMPLETE"
        elif self.state == "DRIVING":
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
        
        # Obstacle status with bilingual messages
        obstacle_detected = self.min_obstacle_distance < self.min_safe_distance
        
        if obstacle_detected:
            status_text = f"🚨 ПРЕПЯТСТВИЕ {round(self.min_obstacle_distance, 1)}m | OBSTACLE DETECTED" # We can remove the russian easter egg? 
        else:
            status_text = "✅ ПУТЬ СВОБОДЕН | PATH CLEAR" # We can remove the russian easter egg? 
        
        obstacle_data = {
            "status": status_text,
            "min_distance": round(self.min_obstacle_distance, 1) if self.min_obstacle_distance != float('inf') else 999.9,
            "front_clear": self.front_clear != float('inf'),
            "left_clear": self.left_clear > self.min_safe_distance,
            "right_clear": self.right_clear > self.min_safe_distance,
            "front_distance": round(self.front_clear, 1) if self.front_clear != float('inf') else 999.9,
            "left_distance": round(self.left_clear, 1) if self.left_clear != float('inf') else 999.9,
            "right_distance": round(self.right_clear, 1) if self.right_clear != float('inf') else 999.9
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
        node.get_logger().info("МИССИЯ ПРЕРВАНА ОПЕРАТОРОМ | Mission aborted by user") # We can remove the russian easter egg? 
    finally:
        node.stop_boat()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
