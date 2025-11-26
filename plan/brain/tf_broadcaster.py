import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)

        # Subscribe to the simulation's pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/wamv/pose',
            self.listener_callback,
            10)
        self.get_logger().info('TF Broadcaster: Broadcasting world -> wamv/base_link transform from /wamv/pose')

    def listener_callback(self, msg):
        # Create a TransformStamped message from the PoseStamped
        t = TransformStamped()

        # Set the header - use 'world' as the parent frame to match astar_planner expectations
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'wamv/wamv/base_link'

        # Copy position
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        # Copy orientation
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Broadcast it to the global TF system
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
