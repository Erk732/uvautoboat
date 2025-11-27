import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)

        # Create QoS profile matching VRX's TF publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to VRX's /wamv/pose topic which publishes TFMessage
        # VRX publishes the boat pose as transforms, not PoseStamped
        self.subscription = self.create_subscription(
            TFMessage,
            '/wamv/pose',
            self.listener_callback,
            qos_profile)

        self.get_logger().info('TF Broadcaster: Listening to /wamv/pose (TFMessage) to create world frame')

    def listener_callback(self, msg):
        # VRX publishes transforms with frame_id as the boat frame
        # We need to republish them with 'world' as the parent frame
        for transform in msg.transforms:
            # Create a new transform with 'world' as parent
            t = TransformStamped()

            # Use message timestamp if available, otherwise use current time
            if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
                t.header.stamp = self.get_clock().now().to_msg()
            else:
                t.header.stamp = transform.header.stamp

            # Set parent frame to 'world' instead of original frame
            t.header.frame_id = 'world'

            # Keep the child frame (should be wamv/wamv/base_link or similar)
            t.child_frame_id = transform.child_frame_id

            # Copy the transform data
            t.transform = transform.transform

            # Broadcast to TF
            self.br.sendTransform(t)

            # Log only the base_link transform to avoid spam
            if 'base_link' in transform.child_frame_id:
                self.get_logger().debug(
                    f'Broadcasting: world -> {transform.child_frame_id}',
                    throttle_duration_sec=5.0
                )

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
