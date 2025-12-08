#!/usr/bin/env python3
"""
Buran Controller - Simple PID Waypoint Following

Simple logic:
1. Receive target from planner
2. PID heading control to reach target
3. Slow down near obstacles (from OKO)
"""

import rclpy
from rclpy.node import Node
import math
import json

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, String


# Thrust limits
MAX_THRUST = 800.0
TURN_POWER_LIMIT = 600.0


class BuranController(Node):
    def __init__(self):
        super().__init__('buran_controller')

        # PID parameters
        self.declare_parameter('kp', 300.0)
        self.declare_parameter('ki', 10.0)
        self.declare_parameter('kd', 80.0)
        self.declare_parameter('base_speed', 400.0)
        self.declare_parameter('obstacle_slow_factor', 0.3)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.obstacle_slow_factor = self.get_parameter('obstacle_slow_factor').value

        # State
        self.current_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_to_target = float('inf')
        
        # PID state
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.dt = 0.05  # 20Hz

        # Obstacle info from OKO
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.urgency = 0.0

        # Mission state
        self.mission_active = False

        # Subscribers
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)
        self.create_subscription(String, '/planning/current_target', self.target_cb, 10)
        self.create_subscription(String, '/planning/mission_status', self.status_cb, 10)
        self.create_subscription(String, '/perception/obstacle_info', self.obstacle_cb, 10)

        # Publishers
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # Control loop at 20Hz
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info("BURAN CONTROLLER")
        self.get_logger().info("Simple PID Waypoint Following")
        self.get_logger().info(f"PID: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info("=" * 50)

    def imu_cb(self, msg):
        q = msg.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 
                                       1 - 2 * (q.y * q.y + q.z * q.z))

    def target_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.current_x, self.current_y = data['current_position']
            self.target_x, self.target_y = data['target_waypoint']
            self.distance_to_target = data['distance_to_target']
        except Exception as e:
            self.get_logger().warn(f"Target parse error: {e}")

    def status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            state = data.get('state', '')
            was_active = self.mission_active
            self.mission_active = (state == "DRIVING")
            
            if was_active and not self.mission_active:
                self.stop()
                self.integral_error = 0.0
                self.previous_error = 0.0
        except:
            pass

    def obstacle_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacle_detected = data.get('obstacle_detected', False)
            self.obstacle_distance = data.get('min_distance', float('inf'))
            self.urgency = data.get('urgency', 0.0)
        except:
            pass

    def control_loop(self):
        """PID control to reach waypoint"""
        if not self.mission_active or self.target_x is None:
            self.stop()
            return

        # Calculate heading error
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        target_heading = math.atan2(dy, dx)
        
        error = self.normalize_angle(target_heading - self.current_yaw)

        # PID
        self.integral_error += error * self.dt
        self.integral_error = max(-0.5, min(0.5, self.integral_error))  # Anti-windup
        
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error

        turn_power = (self.kp * error + 
                      self.ki * self.integral_error + 
                      self.kd * derivative)
        turn_power = max(-TURN_POWER_LIMIT, min(TURN_POWER_LIMIT, turn_power))

        # Speed - slow down for large heading errors
        error_deg = abs(math.degrees(error))
        if error_deg > 45:
            speed = self.base_speed * 0.4
        elif error_deg > 20:
            speed = self.base_speed * 0.7
        else:
            speed = self.base_speed

        # Slow down near obstacles
        if self.obstacle_detected and self.urgency > 0:
            speed *= (1.0 - self.urgency * (1.0 - self.obstacle_slow_factor))

        # Slow down near waypoint
        if self.distance_to_target < 10.0:
            speed *= max(0.4, self.distance_to_target / 10.0)

        # Differential thrust
        left = speed - turn_power
        right = speed + turn_power

        # Clamp
        left = max(-MAX_THRUST, min(MAX_THRUST, left))
        right = max(-MAX_THRUST, min(MAX_THRUST, right))

        self.send_thrust(left, right)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def send_thrust(self, left, right):
        left_msg = Float64()
        left_msg.data = float(left)
        self.pub_left.publish(left_msg)

        right_msg = Float64()
        right_msg.data = float(right)
        self.pub_right.publish(right_msg)

    def stop(self):
        self.send_thrust(0.0, 0.0)


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