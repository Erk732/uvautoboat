import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        self.br = TransformBroadcaster(self)
        
        # --- CRITICAL FIX ---
        # Instead of fake static numbers (0,0), we LISTEN to the simulation 
        # to get the REAL Ground Truth position.
        self.subscription = self.create_subscription(
            Odometry, 
            '/wamv/sensors/position/ground_truth_odometry', 
            self.handle_pose, 
            10
        )
        
        self.get_logger().info("📢 TF Broadcaster Started! Bridging Gazebo -> ROS2")

    def handle_pose(self, msg):
        t = TransformStamped()
        
        # 1. Timestamp (Now)
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 2. Frame Names (The "Name Game" Fix)
        t.header.frame_id = 'world'          # The Global Map
        t.child_frame_id = 'wamv/wamv/base_link'  # The Boat (Standard VRX name)

        # 3. Copy Position from Simulator (Dynamic!)
        # This allows the map to update as the boat moves
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 4. Copy Rotation
        t.transform.rotation = msg.pose.pose.orientation

        # 5. Send it!
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()