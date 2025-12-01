#!/usr/bin/env python3
# AGAIN ITS ONLY PSEUDO CODE CHANGE WHAT IS NECESSARY!
# YINPU AND LEO PLEASE CHANGE THE CODE AS WHAT YOU NEED WHAT DO YOU WANT TO CHANGE
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import math

# Messages
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class AtlantisController(Node):
    def __init__(self):
        super().__init__('atlantis_controller')

        # --- 1. DECLARE PARAMETERS ---
        self.declare_parameter('kp', 400.0)       # Steering aggression
        self.declare_parameter('base_speed', 600.0) # Cruising speed I used 500 before maybe we can use another value
        
        # --- 2. LOAD INITIAL VALUES ---
        self.kp = self.get_parameter('kp').value
        self.base_speed = self.get_parameter('base_speed').value

        # --- 3. DYNAMIC RECONFIGURE ---
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- STATE ---
        self.target = None # (x, y, mode)
        self.current_gps = None
        self.start_gps = None
        self.current_yaw = 0.0

        # --- PUBS/SUBS ---
        self.create_subscription(Point, '/atlantis/target', self.target_callback, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)

        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.create_timer(0.05, self.control_loop) # 20Hz Control Loop
        self.get_logger().info("ATLANTIS CONTROLLER: Online (Dynamic PID Enabled)")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
                self.get_logger().info(f"Tuned KP to: {self.kp}")
            elif param.name == 'base_speed':
                self.base_speed = param.value
                self.get_logger().info(f"Speed set to: {self.base_speed}")
        return SetParametersResult(successful=True)

    def target_callback(self, msg):
        self.target = (msg.x, msg.y, msg.z)

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if not self.start_gps: self.start_gps = self.current_gps

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def latlon_to_meters(self, lat, lon):
        if not self.start_gps: return 0,0
        R = 6371000.0
        dx = (math.radians(lon - self.start_gps[1]) * R * math.cos(math.radians(self.start_gps[0])))
        dy = (math.radians(lat - self.start_gps[0]) * R)
        return dx, dy

    def control_loop(self):
        if not self.target or not self.current_gps: return

        tx, ty, mode = self.target
        
        # STOP
        if mode == 0.0:
            self.send_thrust(0.0, 0.0)
            return

        # REVERSE
        if mode == -1.0:
            self.send_thrust(-800.0, -800.0)
            return

        # DRIVE (PID)
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        dx = tx - curr_x
        dy = ty - curr_y
        desired_yaw = math.atan2(dy, dx)
        
        error = desired_yaw - self.current_yaw
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi

        # Uses the DYNAMIC 'self.kp' I have no idea if this one works good or no I didn't test it yet 
        turn_effort = error * self.kp
        turn_effort = max(-800.0, min(800.0, turn_effort))

        left = self.base_speed - turn_effort
        right = self.base_speed + turn_effort

        left = max(-1000.0, min(1000.0, left))
        right = max(-1000.0, min(1000.0, right))

        self.send_thrust(left, right)

    def send_thrust(self, l, r):
        self.pub_left.publish(Float64(data=float(l)))
        self.pub_right.publish(Float64(data=float(r)))

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()