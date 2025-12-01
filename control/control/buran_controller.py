import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import math
import tf_transformations


class BuranController(Node):
    def __init__(self):
        super().__init__('buran_controller')

        # ==============================
        #     SUBSCRIBERS
        # ==============================
        self.create_subscription(Path,
                                 '/atlantis/path',
                                 self.path_callback,
                                 10)

        self.create_subscription(PoseStamped,
                                 '/wamv/pose',
                                 self.pose_callback,
                                 20)

        # ==============================
        #     THRUSTER PUBLISHERS
        # ==============================
        self.left_pub = self.create_publisher(
            Float32,
            '/wamv/thrusters/left/thrust',
            10
        )

        self.right_pub = self.create_publisher(
            Float32,
            '/wamv/thrusters/right/thrust',
            10
        )

        # Controller state
        self.path = []
        self.current_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Buran Controller READY. Listening to /atlantis/path")

    # =====================================================
    #                 CALLBACKS
    # =====================================================
    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.current_idx = 0
        self.get_logger().info(f"Received path with {len(self.path)} points")

    def pose_callback(self, msg: PoseStamped):
        pose = msg.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y

        # Extract yaw from quaternion
        q = pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    # =====================================================
    #                 CONTROL LOOP
    # =====================================================
    def control_loop(self):
        if len(self.path) == 0:
            return

        if self.current_idx >= len(self.path):
            self.stop_thrusters()
            self.get_logger().info("Reached final waypoint.")
            return

        target = self.path[self.current_idx].pose.position
        dx = target.x - self.current_x
        dy = target.y - self.current_y
        distance = math.hypot(dx, dy)

        # Move to next waypoint if close enough
        if distance < 1.5:
            self.current_idx += 1
            return

        # Heading error
        desired_yaw = math.atan2(dy, dx)
        yaw_err = desired_yaw - self.current_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))  # normalize

        # Simple proportional control
        fwd_thrust = 50.0           # forward thrust
        turn_thrust = 80.0 * yaw_err  # turn proportional to yaw error

        left = fwd_thrust - turn_thrust
        right = fwd_thrust + turn_thrust

        # Clamp thrust to [-100, 100]
        left = max(min(left, 100.0), -100.0)
        right = max(min(right, 100.0), -100.0)

        self.left_pub.publish(Float32(data=left))
        self.right_pub.publish(Float32(data=right))

    def stop_thrusters(self):
        self.left_pub.publish(Float32(data=0.0))
        self.right_pub.publish(Float32(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = BuranController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
