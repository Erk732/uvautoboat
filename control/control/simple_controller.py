import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        # Thrust publishers
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        # Steering angle publishers (radians, positive = left)
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        self.forward_cmd = 800.0    # tweak for your sim
        self.turn_angle = 0.0       # set e.g. 0.3 for gentle turn

        self.timer = self.create_timer(0.5, self.publish_cmds)
        self.get_logger().info('SimpleController publishing thrust and steering')

    def publish_cmds(self):
        thrust_l = Float64()
        thrust_l.data = self.forward_cmd
        thrust_r = Float64()
        thrust_r.data = self.forward_cmd
        pos_l = Float64()
        pos_l.data = self.turn_angle
        pos_r = Float64()
        pos_r.data = self.turn_angle

        self.left_thrust_pub.publish(thrust_l)
        self.right_thrust_pub.publish(thrust_r)
        self.left_pos_pub.publish(pos_l)
        self.right_pos_pub.publish(pos_r)
        self.get_logger().info(f'Thrust L/R: {thrust_l.data:.1f}, Pos L/R: {pos_l.data:.2f}')


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
