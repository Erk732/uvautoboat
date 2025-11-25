import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)
        
        # Subscribe to the simulation's pose topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/wamv/pose',
            self.listener_callback,
            10)
        self.get_logger().info('TF Broadcaster: Relaying /wamv/pose to /tf')

    def listener_callback(self, msg):
        # The simulation sends a "Transform" message. 
        # We just need to forward it with the right frame names.
        for t in msg.transforms:
            # FORCE the frame ID to 'map' so Rviz accepts it
            t.header.frame_id = 'map'
            t.child_frame_id = 'wamv/base_link'
            
            # Update timestamp to "now" so Rviz doesn't complain it's old
            t.header.stamp = self.get_clock().now().to_msg()
            
            # Broadcast it to the global system
            self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
