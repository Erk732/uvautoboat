#!/usr/bin/env python3
"""
Filter PoseStamped messages from a mixed-type topic (e.g., /wamv/pose) and
republish them to a clean PoseStamped-only topic.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseFilter(Node):
    def __init__(self) -> None:
        super().__init__('pose_filter')
        self.declare_parameter('input_topic', '/wamv/pose')
        self.declare_parameter('output_topic', '/wamv/pose_filtered')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.sub = self.create_subscription(
            PoseStamped,
            input_topic,
            self.pose_callback,
            10
        )

        self.logged_once = False

        self.get_logger().info(f'Pose filter relay: {input_topic} -> {output_topic}')

    def pose_callback(self, msg: PoseStamped) -> None:
        # Pass through PoseStamped messages as-is
        self.pub.publish(msg)
        if not self.logged_once:
            p = msg.pose.position
            self.get_logger().info(
                f'Initial pose (first message) frame={msg.header.frame_id}: '
                f'x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}'
            )
            self.logged_once = True


def main(args=None):
    rclpy.init(args=args)
    node = PoseFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pose_filter.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
