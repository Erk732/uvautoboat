#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import json
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, String

class BuranController(Node):
    def __init__(self):
        super().__init__('buran_controller_node')

        # Control Params
        self.declare_parameter('kp', 400.0)
        self.declare_parameter('base_speed', 500.0)
        self.declare_parameter('avoidance_persistence', 2.0) # CRITICAL FIX

        self.kp = self.get_parameter('kp').value
        self.base_speed = self.get_parameter('base_speed').value
        self.avoidance_persistence = self.get_parameter('avoidance_persistence').value

        # State
        self.current_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Obstacle State
        self.obstacle_detected = False
        self.left_clear = 100.0
        self.right_clear = 100.0
        
        # Avoidance State
        self.avoidance_mode = False
        self.avoidance_cooldown_time = None
        self.avoidance_direction = "LEFT" 

        # Pub/Sub
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(String, '/planning/current_target', self.target_callback, 10)
        self.create_subscription(String, '/perception/obstacle_info', self.obstacle_callback, 10)
        
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("BURAN Controller Ready")

    def gps_callback(self, msg): pass # We use planner's current_position feed mostly
    
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
        except: pass

    def obstacle_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacle_detected = data['obstacle_detected']
            self.left_clear = data['left_clear']
            self.right_clear = data['right_clear']
        except: pass

    def control_loop(self):
        if self.target_x is None: return

        # 1. Heading Calculation
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        target_heading = math.atan2(dy, dx)
        
        now = self.get_clock().now()

        # 2. Avoidance Logic
        if self.obstacle_detected:
            self.avoidance_mode = True
            self.avoidance_cooldown_time = now # Reset timer
            
            # Simple decision: Go to clearer side
            if self.left_clear > self.right_clear:
                self.avoidance_direction = "LEFT"
                target_heading = self.current_yaw + 1.2
            else:
                self.avoidance_direction = "RIGHT"
                target_heading = self.current_yaw - 1.2
        
        # 3. Persistence Logic (The Fix)
        elif self.avoidance_mode:
            elapsed = (now - self.avoidance_cooldown_time).nanoseconds / 1e9
            if elapsed < self.avoidance_persistence:
                # Keep turning!
                turn = 0.8 if self.avoidance_direction == "LEFT" else -0.8
                target_heading = self.current_yaw + turn
            else:
                self.avoidance_mode = False # Finally resume path

        # 4. PID & Thrust
        err = target_heading - self.current_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        
        turn_power = self.kp * err
        turn_power = max(-1000.0, min(1000.0, turn_power))
        
        speed = self.base_speed
        if abs(err) > 0.5: speed *= 0.5 # Slow in turns
        
        self.pub_left.publish(Float64(data=speed - turn_power))
        self.pub_right.publish(Float64(data=speed + turn_power))

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