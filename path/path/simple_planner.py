import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner_node')
        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/wamv/sensors/odometry', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        # Publisher
        self.pub_path = self.create_publisher(Path, '/planning/path', 10)
        
        self.current_pose = None
        self.goal_pose = None
        self.get_logger().info('Simple Planner Node Started. Waiting for Odom and Goal...')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.get_logger().info(f'Received Goal: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}')
        self.goal_pose = msg.pose
        if self.current_pose is not None:
            self.plan_straight_line()
        else:
            self.get_logger().warn('Cannot plan: No Odometry received yet.')

    def plan_straight_line(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        dx = goal_x - start_x
        dy = goal_y - start_y
        dist = math.sqrt(dx**2 + dy**2)
        
        step_size = 1.0
        num_steps = int(dist / step_size)
        if num_steps < 1: num_steps = 1
        
        for i in range(num_steps + 1):
            ratio = i / float(num_steps)
            px = start_x + ratio * dx
            py = start_y + ratio * dy
            
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = px
            pose_stamped.pose.position.y = py
            path_msg.poses.append(pose_stamped)

        self.pub_path.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} waypoints.')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
