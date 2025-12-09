import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time

class MissionTrigger(Node):
    def __init__(self):
        super().__init__('mission_trigger')
        
        # Publisher for the goal (Where you want the boat to go)
        self.goal_pub = self.create_publisher(PoseStamped, '/planning/goal', 10)
        
        # Publisher for the initial pose (To wake up the planner)
        self.odom_pub = self.create_publisher(Odometry, '/wamv/sensors/odometry', 10)

        self.get_logger().info('Mission Trigger: Waiting 5 seconds for system to warm up...')
        
        # Create a timer to run the mission ONCE after 5 seconds
        self.timer = self.create_timer(5.0, self.trigger_mission)
        self.mission_sent = False

    def trigger_mission(self):
        if self.mission_sent:
            return

        # 1. Force Initial Position (Wake up call for TF)
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'map'
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        self.odom_pub.publish(odom_msg)
        self.get_logger().info('Mission Trigger: Sent Wake-up Odom Signal')

        # 2. Send the Goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        # --- SET YOUR GOAL COORDINATES HERE ---
        goal_msg.pose.position.x = 30.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.position.z = 0.0
        # --------------------------------------

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Mission Trigger: GOAL SENT to x={goal_msg.pose.position.x}!')
        
        self.mission_sent = True
        
        # Stop the timer so it doesn't keep sending
        self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = MissionTrigger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
