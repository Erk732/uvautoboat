import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Parameters with defaults
        self.declare_parameter('forward_cmd', 800.0)
        self.declare_parameter('turn_angle', 0.0)
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('stop_topic', '/wamv/stop')

        # Publishers
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        # Stop subscriber (zeros commands when Bool True arrives)
        stop_topic = self.get_parameter('stop_topic').get_parameter_value().string_value
        self.stop_sub = self.create_subscription(Bool, stop_topic, self.stop_callback, 10)
        self.stopped = False

        period = self.get_parameter('publish_period').get_parameter_value().double_value
        self.timer = self.create_timer(period, self.publish_cmds)
        self.get_logger().info(f'Publishing thrust/steering every {period}s; stop topic: {stop_topic}')

    def stop_callback(self, msg: Bool):
        self.stopped = msg.data
        if self.stopped:
            self.get_logger().warn('Stop command received, zeroing thrust')

    def publish_cmds(self):
        if self.stopped:
            forward_cmd = 0.0
            turn_angle = 0.0
        else:
            forward_cmd = self.get_parameter('forward_cmd').get_parameter_value().double_value
            turn_angle = self.get_parameter('turn_angle').get_parameter_value().double_value

        thrust_l = Float64()
        thrust_l.data = forward_cmd
        thrust_r = Float64()
        thrust_r.data = forward_cmd
        pos_l = Float64()
        pos_l.data = turn_angle
        pos_r = Float64()
        pos_r.data = turn_angle

        self.left_thrust_pub.publish(thrust_l)
        self.right_thrust_pub.publish(thrust_r)
        self.left_pos_pub.publish(pos_l)
        self.right_pos_pub.publish(pos_r)
        self.get_logger().info(f'Thrust L/R: {forward_cmd:.1f}, Pos L/R: {turn_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
