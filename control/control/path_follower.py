import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # --- SETTINGS ---
        self.LOOKAHEAD_DIST = 3.5   # How far to look ahead (meters). Larger = smoother, Slower turns.
        self.BASE_SPEED = 500.0     # Base thrust (Max is usually 1000.0 or 1.0 depending on sim)
        self.KP = 300.0             # Proportional Gain (How hard to turn)
        self.GOAL_TOLERANCE = 2.0   # Stop when this close to end (meters)

        # --- SUBSCRIBERS ---
        self.sub_path = self.create_subscription(Path, '/planning/path', self.path_callback, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/wamv/pose', self.pose_callback, 10)

        # --- PUBLISHERS ---
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # --- STATE ---
        self.path = []
        self.current_pose = None
        
        # Control Loop (20Hz - Faster than planner!)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Pure Pursuit Path Follower Started")

    def path_callback(self, msg):
        # Update path whenever A* sends a new one
        self.path = msg.poses
        # self.get_logger().info(f"Received path with {len(self.path)} waypoints")

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_yaw_from_pose(self, pose):
        # Convert Quaternion to Yaw (Heading)
        q = pose.pose.orientation
        # Standard formula for yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # 1. Safety Checks
        if not self.path or not self.current_pose:
            self.stop_boat()
            return

        # 2. Get Boat State
        bx = self.current_pose.pose.position.x
        by = self.current_pose.pose.position.y
        b_yaw = self.get_yaw_from_pose(self.current_pose)

        # 3. Find the Lookahead Point
        target_x, target_y = self.find_lookahead_point(bx, by)

        if target_x is None:
            # We reached the end or lost the path
            dist_to_end = math.hypot(self.path[-1].pose.position.x - bx, 
                                     self.path[-1].pose.position.y - by)
            if dist_to_end < self.GOAL_TOLERANCE:
                self.get_logger().info("Goal Reached!")
                self.path = [] # Clear path
            self.stop_boat()
            return

        # 4. Calculate Heading Error
        desired_yaw = math.atan2(target_y - by, target_x - bx)
        error_yaw = desired_yaw - b_yaw

        # Normalize error to [-pi, pi]
        while error_yaw > math.pi: error_yaw -= 2.0 * math.pi
        while error_yaw < -math.pi: error_yaw += 2.0 * math.pi

        # 5. Calculate Thrust (P-Controller)
        turn_effort = error_yaw * self.KP

        # Differential Thrust
        # If turning LEFT (positive error), Right motor pushes harder
        left_thrust = self.BASE_SPEED - turn_effort
        right_thrust = self.BASE_SPEED + turn_effort

        # 6. Clamp Values (Max thrust usually 1000.0 in VRX)
        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))

        self.publish_thrust(left_thrust, right_thrust)

    def find_lookahead_point(self, bx, by):
        # Find the point on the path that is roughly LOOKAHEAD_DIST away
        chosen_pt = None
        
        # Determine the closest point index first (to avoid going backwards)
        closest_dist = float('inf')
        closest_idx = 0
        
        for i, pt in enumerate(self.path):
            dist = math.hypot(pt.pose.position.x - bx, pt.pose.position.y - by)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        # Now search forward from closest point for the lookahead point
        for i in range(closest_idx, len(self.path)):
            pt = self.path[i]
            dist = math.hypot(pt.pose.position.x - bx, pt.pose.position.y - by)
            
            if dist >= self.LOOKAHEAD_DIST:
                chosen_pt = (pt.pose.position.x, pt.pose.position.y)
                break
        
        # If we didn't find a point far enough (end of path), take the last one
        if chosen_pt is None and len(self.path) > 0:
            return self.path[-1].pose.position.x, self.path[-1].pose.position.y
            
        return chosen_pt

    def publish_thrust(self, left, right):
        l_msg = Float64()
        r_msg = Float64()
        l_msg.data = float(left)
        r_msg.data = float(right)
        self.pub_left.publish(l_msg)
        self.pub_right.publish(r_msg)

    def stop_boat(self):
        self.publish_thrust(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()