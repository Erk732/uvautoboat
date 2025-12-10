import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import Float64, String, Empty
import math
import json
import time

# NEW: Relative Import
from .lidar_obstacle_avoidance import LidarObstacleDetector

class AtlantisController(Node):
    def __init__(self):
        super().__init__('atlantis_controller')

        # --- PARAMETERS ---
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('waypoint_tolerance', 8.0) 
        self.declare_parameter('kp', 150.0) 
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)
        
        # Cross-Track Error Gains (Pull back to line)
        self.declare_parameter('xte_gain', 2.5) 
        self.declare_parameter('lookahead_distance', 15.0)

        # --- OBSTACLE PARAMETERS ---
        self.declare_parameter('min_safe_distance', 6.0)   
        self.declare_parameter('critical_distance', 3.0)   
        self.declare_parameter('obstacle_slow_factor', 0.08)  
        self.declare_parameter('reverse_timeout', 10.0)    
        self.declare_parameter('waypoint_timeout', 60.0)

        # Get params
        self.base_speed = self.get_parameter('base_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.xte_gain = self.get_parameter('xte_gain').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.obstacle_slow_factor = self.get_parameter('obstacle_slow_factor').value
        self.reverse_timeout = self.get_parameter('reverse_timeout').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value

        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        self.waypoints = [] 
        self.current_wp_index = 0
        self.state = "WAITING_FOR_PATH"
        self.path_validation_ok = False
        
        # Path Following State
        self.previous_wp = (0.0, 0.0) # To define the line
        
        # PID
        self.previous_error = 0.0
        self.integral_error = 0.0
        
        # Obstacle
        self.min_obstacle_distance = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.obstacle_detected = False
        self.avoidance_mode = False
        self.reverse_start_time = None
        
        # Timers
        self.waypoint_start_time = None
        self.mission_enabled = False
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

        # Lidar (Fixed settings)
        self.lidar_detector = LidarObstacleDetector(
            min_distance=1.0, 
            max_distance=100.0, 
            z_filter_enabled=True,
            min_height=-0.5, 
            max_height=3.0
        )

        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Atlantis Controller Ready (Line Following Mode)")

    def path_callback(self, msg):
        try:
            new_waypoints = []
            for pose in msg.poses:
                new_waypoints.append((pose.pose.position.x, pose.pose.position.y))
            
            if len(new_waypoints) < 2: return
            
            if new_waypoints != self.waypoints:
                self.waypoints = new_waypoints
                self.current_wp_index = 0
                self.path_validation_ok = True
                
                # Set initial previous point as current location or start of path
                if self.start_gps:
                    cx, cy = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
                    self.previous_wp = (cx, cy)
                else:
                    self.previous_wp = self.waypoints[0]
                
                self.waypoint_start_time = self.get_clock().now()
                if self.start_gps:
                    self.mission_enabled = True
                    self.state = "DRIVING"
        except Exception as e:
            self.get_logger().error(f"Path callback error: {e}")

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            if self.waypoints:
                self.mission_enabled = True
                self.state = "DRIVING"

    def start_callback(self, msg):
        self.mission_enabled = True
        self.state = "DRIVING"

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
        # ... (Same logic as before, detecting front/left/right clearance)
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
            
            front_obs = [p[3] for p in points if abs(p[1]) < 2.0 and p[0] > 2.5]
            front_dist = min(front_obs) if front_obs else 100.0
            
            # Simple Hysteresis
            if self.obstacle_detected:
                 self.obstacle_detected = front_dist < (self.min_safe_distance + 2.0)
            else:
                 self.obstacle_detected = front_dist < self.min_safe_distance

            # Sector analysis
            left_p = [p[3] for p in points if p[1] > 2.0]
            right_p = [p[3] for p in points if p[1] < -2.0]
            self.left_clear = min(left_p) if left_p else 100.0
            self.right_clear = min(right_p) if right_p else 100.0
        else:
            self.min_obstacle_distance = 50.0
            self.obstacle_detected = False
            self.left_clear = 100.0
            self.right_clear = 100.0

    def latlon_to_meters(self, lat, lon):
        R = 6371000.0
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])
        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return y, x

    # --- THE CORE LOGIC CHANGE ---
    def calculate_steering_with_crosstrack(self, curr_x, curr_y, target_x, target_y):
        # 1. Vector from Previous Waypoint (Start of line) to Target
        line_dx = target_x - self.previous_wp[0]
        line_dy = target_y - self.previous_wp[1]
        line_len = math.hypot(line_dx, line_dy)
        
        if line_len < 0.1: return math.atan2(line_dy, line_dx) # Fallback

        # Normalize line vector
        u_x = line_dx / line_len
        u_y = line_dy / line_len

        # 2. Vector from Previous Waypoint to Boat
        boat_dx = curr_x - self.previous_wp[0]
        boat_dy = curr_y - self.previous_wp[1]

        # 3. Cross Track Error (Distance from line)
        # 2D cross product gives distance
        cross_track_error = (boat_dx * u_y) - (boat_dy * u_x)
        
        # 4. Lookahead Point on the line
        # Project boat position onto line
        along_track_dist = (boat_dx * u_x) + (boat_dy * u_y)
        
        # Target point is lookahead_dist meters ahead of our projection
        lookahead_x = self.previous_wp[0] + (along_track_dist + self.lookahead_dist) * u_x
        lookahead_y = self.previous_wp[1] + (along_track_dist + self.lookahead_dist) * u_y
        
        # 5. Calculate Heading to Lookahead
        head_dx = lookahead_x - curr_x
        head_dy = lookahead_y - curr_y
        desired_heading = math.atan2(head_dy, head_dx)

        # 6. Apply Aggressive Correction if error is large
        # If we are to the right (error positive), we want to steer left.
        # But math.atan2 handles the geometry, so we rely on the Lookahead point dragging us back.
        # We can boost the return angle by adding a term proportional to error
        correction = math.atan(-cross_track_error * self.xte_gain / self.lookahead_dist)
        
        # Final Target Heading
        return desired_heading # Simpler "Pure Pursuit" effectively handles returning to path

    def control_loop(self):
        if not self.mission_enabled or not self.waypoints:
            self.stop_boat()
            return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        now = self.get_clock().now()

        # Check Waypoint Arrival
        target_x, target_y = self.waypoints[self.current_wp_index]
        dist = math.hypot(target_x - curr_x, target_y - curr_y)
        
        if dist < self.waypoint_tolerance:
            self.previous_wp = self.waypoints[self.current_wp_index] # Update line start
            self.current_wp_index += 1
            self.waypoint_start_time = now 
            self.integral_error = 0.0
            if self.current_wp_index >= len(self.waypoints):
                self.finish_mission()
                return
            return

        # 1. OBSTACLE AVOIDANCE (The "A*" Substitute)
        if self.min_obstacle_distance < self.critical_distance:
             # PANIC REVERSE
             self.send_thrust(-800.0, -800.0)
             return

        if self.obstacle_detected:
            self.avoidance_mode = True
            # Simple Logic: Turn towards the clearest side
            # This pushes us OFF the path, which is fine
            if self.left_clear > self.right_clear:
                target_angle = self.current_yaw + 1.2 # Turn Left
            else:
                target_angle = self.current_yaw - 1.2 # Turn Right
        else:
            # 2. PATH FOLLOWING (Return to Line)
            self.avoidance_mode = False
            target_angle = self.calculate_steering_with_crosstrack(curr_x, curr_y, target_x, target_y)

        # PID Control
        angle_error = target_angle - self.current_yaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi

        self.integral_error += angle_error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error))
        
        # Turn power
        turn_power = (self.kp * angle_error) + (self.ki * self.integral_error)
        turn_power = max(-900.0, min(900.0, turn_power))

        # Speed (Slow down in turns)
        speed = self.base_speed
        if abs(math.degrees(angle_error)) > 20: speed *= 0.6
        if self.avoidance_mode: speed = 400.0

        left_thrust = max(-1000.0, min(1000.0, speed - turn_power))
        right_thrust = max(-1000.0, min(1000.0, speed + turn_power))
        
        self.send_thrust(left_thrust, right_thrust)
        
        if (now - self._last_log_time).nanoseconds / 1e9 > 2.0:
            self.get_logger().info(f"WP{self.current_wp_index} | Dist:{dist:.1f}m | XTE Mode | Obs:{self.obstacle_detected}")
            self._last_log_time = now

    def finish_mission(self):
        self.state = "FINISHED"
        self.stop_boat()
        self.get_logger().info("MISSION COMPLETE")

    def send_thrust(self, left, right):
        if rclpy.ok():
            self.pub_left.publish(Float64(data=float(left)))
            self.pub_right.publish(Float64(data=float(right)))
    
    def stop_boat(self):
        self.send_thrust(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()