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
        self.declare_parameter('waypoint_tolerance', 8.0) 
        self.declare_parameter('kp', 150.0) 
        self.declare_parameter('ki', 20.0)
        self.declare_parameter('kd', 100.0)
        
        # Cross-Track Error Gains
        self.declare_parameter('xte_gain', 2.5) 
        self.declare_parameter('lookahead_distance', 15.0)

        # --- OBSTACLE PARAMETERS ---
        self.declare_parameter('min_safe_distance', 6.0)   
        self.declare_parameter('critical_distance', 3.0)   
        
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

        # STATE 
        self.start_gps = None
        self.current_gps = None
        self.current_yaw = 0.0
        self.waypoints = [] 
        self.current_wp_index = 0
        
        # Path Following State
        self.previous_wp = (0.0, 0.0)
        
        # PID
        self.integral_error = 0.0
        
        # STATE MACHINE
        # "FOLLOW_PATH": Normal sürüş
        # "OBSTACLE_STOP": Engel gördü, durdu, analiz ediyor
        # "OBSTACLE_ESCAPE": Karar verdi, kaçıyor
        self.mode = "FOLLOW_PATH"
        self.escape_start_time = None
        self.escape_direction = 0.0
        
        # Obstacle Data
        self.min_obstacle_distance = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.obstacle_detected = False
        
        self.mission_enabled = False
        self._last_log_time = self.get_clock().now()

        # SUBSCRIBERS 
        self.create_subscription(Path, '/atlantis/path', self.path_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
        self.create_subscription(Empty, '/atlantis/start', self.start_callback, 10)
        self.create_subscription(Empty, '/atlantis/stop', self.stop_callback, 10)

        # PUBLISHERS
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.pub_mission_status = self.create_publisher(String, '/atlantis/mission_status', 10)

        # LIDAR SETTINGS
        self.lidar_detector = LidarObstacleDetector(
            min_distance=0.5, 
            max_distance=100.0, 
            z_filter_enabled=True,
            min_height=-0.5, 
            max_height=5.0 
        )

        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Atlantis Controller: Stop-and-Sense Modu Aktif")

    def path_callback(self, msg):
        try:
            new_waypoints = []
            for pose in msg.poses:
                new_waypoints.append((pose.pose.position.x, pose.pose.position.y))
            
            if len(new_waypoints) < 2: return
            
            if new_waypoints != self.waypoints:
                self.waypoints = new_waypoints
                self.current_wp_index = 0
                
                if self.start_gps:
                    cx, cy = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
                    self.previous_wp = (cx, cy)
                else:
                    self.previous_wp = self.waypoints[0]
                
                if self.start_gps:
                    self.mission_enabled = True
                    self.mode = "FOLLOW_PATH"
        except Exception as e:
            self.get_logger().error(f"Path callback error: {e}")

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            if self.waypoints:
                self.mission_enabled = True

    def start_callback(self, msg):
        self.mission_enabled = True
        self.mode = "FOLLOW_PATH"

    def stop_callback(self, msg):
        self.mission_enabled = False
        self.stop_boat()

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        detected_obstacles = self.lidar_detector.process_pointcloud(msg.data, msg.point_step, sampling_factor=2)
        
        points = []
        for obs in detected_obstacles:
            # KRİTİK DÜZELTME BURADA
            # obs.z <= 3.0 yaptım. Eski kodda 0.6 idi. 
            # İskele yüksek olduğu için artık algılanacak.
            if (obs.x > -1.5 and obs.x < 30.0 and 
                abs(obs.y) < 20.0 and 
                obs.z <= 3.0): 
                
                points.append((obs.x, obs.y, obs.z, obs.distance))

        if points:
            distances = [p[3] for p in points]
            self.min_obstacle_distance = min(distances)
            
            # Ön taraf kontrolü (daha geniş bir açı)
            front_obs = [p[3] for p in points if abs(p[1]) < 4.0 and p[0] > 0]
            front_dist = min(front_obs) if front_obs else 100.0
            
            # Hysteresis (Titremeyi önlemek için)
            if self.obstacle_detected:
                 self.obstacle_detected = front_dist < (self.min_safe_distance + 2.0)
            else:
                 self.obstacle_detected = front_dist < self.min_safe_distance

            # Sektör Analizi (Kaçış yönü için)
            # Sol taraf (y > 2.0) ve Sağ taraf (y < -2.0)
            left_p = [p[3] for p in points if p[1] > 2.0]
            right_p = [p[3] for p in points if p[1] < -2.0]
            
            # En yakın engelin ne kadar uzakta olduğu (ne kadar büyükse o kadar iyi)
            self.left_clear = min(left_p) if left_p else 50.0
            self.right_clear = min(right_p) if right_p else 50.0
        else:
            self.min_obstacle_distance = 50.0
            self.obstacle_detected = False
            self.left_clear = 50.0
            self.right_clear = 50.0

    def latlon_to_meters(self, lat, lon):
        R = 6371000.0
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])
        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return y, x

    def calculate_steering_with_crosstrack(self, curr_x, curr_y, target_x, target_y):
        line_dx = target_x - self.previous_wp[0]
        line_dy = target_y - self.previous_wp[1]
        line_len = math.hypot(line_dx, line_dy)
        
        if line_len < 0.1: return math.atan2(line_dy, line_dx)

        u_x = line_dx / line_len
        u_y = line_dy / line_len

        boat_dx = curr_x - self.previous_wp[0]
        boat_dy = curr_y - self.previous_wp[1]

        cross_track_error = (boat_dx * u_y) - (boat_dy * u_x)
        along_track_dist = (boat_dx * u_x) + (boat_dy * u_y)
        
        lookahead_x = self.previous_wp[0] + (along_track_dist + self.lookahead_dist) * u_x
        lookahead_y = self.previous_wp[1] + (along_track_dist + self.lookahead_dist) * u_y
        
        return math.atan2(lookahead_y - curr_y, lookahead_x - curr_x)

    # YENİ KONTROL DÖNGÜSÜ 
    def control_loop(self):
        if not self.mission_enabled or not self.waypoints:
            self.stop_boat()
            return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        now = self.get_clock().now()

        # Acil Durum Kontrolü (Çok yakında engel varsa her durumda dur)
        if self.min_obstacle_distance < 2.0 and self.mode != "OBSTACLE_STOP":
             self.stop_boat()
             self.mode = "OBSTACLE_STOP"
             self.escape_start_time = now
             self.get_logger().error("ACİL DURUŞ! Çok yakın engel.")
             return

        # DURUM 1: YOL TAKİBİ 
        if self.mode == "FOLLOW_PATH":
            # Engel gördük mü?
            if self.obstacle_detected:
                self.get_logger().warn("Engel Tespit Edildi -> Analiz Moduna Geçiliyor.")
                self.stop_boat()
                self.mode = "OBSTACLE_STOP"
                self.escape_start_time = now
                return

            # Waypoint Varış Kontrolü
            target_x, target_y = self.waypoints[self.current_wp_index]
            dist = math.hypot(target_x - curr_x, target_y - curr_y)
            
            if dist < self.waypoint_tolerance:
                self.previous_wp = self.waypoints[self.current_wp_index]
                self.current_wp_index += 1
                if self.current_wp_index >= len(self.waypoints):
                    self.finish_mission()
                return

            target_angle = self.calculate_steering_with_crosstrack(curr_x, curr_y, target_x, target_y)
            self.apply_control(target_angle, self.base_speed)

        # DURUM 2: DUR VE ANALİZ ET 
        elif self.mode == "OBSTACLE_STOP":
            self.stop_boat()
            
            # 2 Saniye bekle (Lidar verisinin oturması için)
            time_passed = (now - self.escape_start_time).nanoseconds / 1e9
            
            if time_passed > 2.0:
                self.get_logger().info(f"ANALİZ: Sol Boşluk: {self.left_clear:.1f}m | Sağ Boşluk: {self.right_clear:.1f}m")
                
                # Hangi taraf daha boşsa oraya 90 derece dön
                if self.left_clear > self.right_clear:
                    self.get_logger().info("KARAR: SOLA KAÇILIYOR")
                    self.escape_direction = self.current_yaw + 1.57 # +90 derece
                else:
                    self.get_logger().info("KARAR: SAĞA KAÇILIYOR")
                    self.escape_direction = self.current_yaw - 1.57 # -90 derece
                
                self.mode = "OBSTACLE_ESCAPE"
                self.escape_start_time = now # Süreyi sıfırla

        # DURUM 3: KAÇIŞ MANEVRASI 
        elif self.mode == "OBSTACLE_ESCAPE":
            time_passed = (now - self.escape_start_time).nanoseconds / 1e9
            
            # 5 Saniye boyunca kaçış yönüne git
            if time_passed < 5.0:
                self.apply_control(self.escape_direction, 600.0)
            else:
                self.get_logger().info("OBSTACLE AVOIDED. FOLLOWING THE PATH.")
                self.mode = "FOLLOW_PATH"
                # Not: Eğer engel hala oradaysa, bir sonraki döngüde tekrar yakalanır 
                # ve tekrar kaçar (böylece etrafından dolaşmış olur).

        # Loglama (3 saniyede bir)
        if (now - self._last_log_time).nanoseconds / 1e9 > 3.0:
            status = f"Mod: {self.mode} | WP: {self.current_wp_index}"
            self.get_logger().info(status)
            self._last_log_time = now

    def finish_mission(self):
        self.stop_boat()
        self.get_logger().info("MISSION ACCOMPLISHED!")

    def apply_control(self, target_angle, speed):
        """PID ve Motor sürme işlemini yapan fonksiyon"""
        angle_error = target_angle - self.current_yaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi

        self.integral_error += angle_error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error))
        
        turn_power = (self.kp * angle_error) + (self.ki * self.integral_error)
        turn_power = max(-900.0, min(900.0, turn_power))

        left_thrust = max(-1000.0, min(1000.0, speed - turn_power))
        right_thrust = max(-1000.0, min(1000.0, speed + turn_power))
        
        self.send_thrust(left_thrust, right_thrust)

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