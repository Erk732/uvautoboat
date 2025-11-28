import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)

        # Create TF buffer and listener to read existing transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timer to periodically create world -> base_link transform
        # We need to create a static transform from world to the robot's base
        self.timer = self.create_timer(0.05, self.publish_world_transform)  # 20 Hz

        self.get_logger().info('TF Broadcaster: Creating world frame as root of TF tree')

    def publish_world_transform(self):
        """
        Creates a world -> wamv/wamv/base_link transform.
        Since VRX doesn't provide global position, we create a static identity transform.
        The boat's wamv/wamv/base_link becomes our global reference.
        """
        t = TransformStamped()

        # Set timestamp to current time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'wamv/wamv/base_link'

        # Identity transform (world aligns with boat's initial position)
        # In a real deployment, this would come from GPS or other global localization
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # No rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transform
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
