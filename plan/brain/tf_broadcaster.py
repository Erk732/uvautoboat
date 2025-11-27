import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)

        # Create QoS profile matching VRX's PoseStamped publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the simulation's pose topic with explicit QoS
        # Note: /wamv/pose has multiple message types published by different nodes
        # We specifically want the PoseStamped messages from ros_gz_bridge
        self.subscription = self.create_subscription(
            PoseStamped,
            '/wamv/pose',
            self.listener_callback,
            qos_profile)
        self.get_logger().info('TF Broadcaster: Broadcasting world -> wamv/wamv/base_link transform from /wamv/pose (PoseStamped)')

    def listener_callback(self, msg):
        # Create a TransformStamped message from the PoseStamped
        t = TransformStamped()

        # Set the header - use 'world' as the parent frame to match astar_planner expectations
        # Use message timestamp if available, otherwise use current time
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            # Empty timestamp - use current simulation time
            t.header.stamp = self.get_clock().now().to_msg()
        else:
            # Use the message timestamp to maintain time consistency
            t.header.stamp = msg.header.stamp

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
