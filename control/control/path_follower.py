#!/usr/bin/env python3
# path_follower.py
#
# Simple waypoint-following controller for VRX WAM-V.
#
# Subscribes:
#   - /planned_path : nav_msgs/Path          (from planning group)
#   - /wamv/pose    : geometry_msgs/PoseStamped (boat pose from VRX / bridge)
#
# Publishes:
#   - /wamv/thrusters/left/thrust  : std_msgs/Float64
#   - /wamv/thrusters/right/thrust : std_msgs/Float64
#
# Control idea:
#   - Take current waypoint from the received Path.
#   - Compute distance and heading (yaw) error.
#   - Use a constant forward thrust.
#   - Use a proportional controller on yaw error to create a thrust difference
#     between left and right thrusters.
#   - If we are close enough to the waypoint, switch to the next one.
#
# NOTE:
#   This is a very simple controller intended for testing and integration.
#   You will likely need to tune the gains and thrust limits for your task.

import math
from typing import List

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class ThrusterPathFollower(Node):
    """Waypoint follower that drives left and right thrusters."""

    def __init__(self) -> None:
        super().__init__('thruster_path_follower')

        # Parameters (can be overridden from a launch file)
        self.declare_parameter('forward_thrust', 50.0)
        self.declare_parameter('k_yaw', 30.0)
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('max_thrust', 100.0)

        self.forward_thrust = float(self.get_parameter('forward_thrust').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.max_thrust = float(self.get_parameter('max_thrust').value)

        # Internal state
        self.waypoints: List[PoseStamped] = []
        self.current_index: int = 0
        self.has_path: bool = False

        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw: float = 0.0

        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',       # Must match planning group topic name
            self.path_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/wamv/pose',
            self.pose_callback,
            10
        )

        # Publishers: left and right thruster commands
        self.left_thruster_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/left/thrust',
            10
        )
        self.right_thruster_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/right/thrust',
            10
        )

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('ThrusterPathFollower node started.')

    def path_callback(self, msg: Path) -> None:
        """Store the received path and reset waypoint index."""
        self.waypoints = list(msg.poses)
        self.current_index = 0
        self.has_path = len(self.waypoints) > 0

        self.get_logger().info(
            f'Received path with {len(self.waypoints)} waypoints.'
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        """Update current boat pose."""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        q = msg.pose.orientation
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self) -> None:
        """Main control loop executed at fixed frequency."""
        # If no path or finished all waypoints: stop the boat
        if not self.has_path or self.current_index >= len(self.waypoints):
            self.publish_thrusters(0.0, 0.0)
            return

        # Target waypoint
        target_pose = self.waypoints[self.current_index].pose
        tx = target_pose.position.x
        ty = target_pose.position.y

        # Position error
        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # If close enough to the current waypoint, switch to the next
        if distance < self.waypoint_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_index} / {len(self.waypoints) - 1}'
            )
            self.current_index += 1

            # If that was the last waypoint, stop
            if self.current_index >= len(self.waypoints):
                self.publish_thrusters(0.0, 0.0)
            return

        # Desired heading to the waypoint
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

        # Compute thrust commands
        # Forward component: constant forward thrust
        forward = self.forward_thrust

        # Turning component: proportional to yaw error
        turn = self.k_yaw * yaw_error

        # Left and right thrust
        left_thrust = forward - turn
        right_thrust = forward + turn

        # Clamp thrusts to [-max_thrust, max_thrust]
        left_thrust = max(-self.max_thrust, min(self.max_thrust, left_thrust))
        right_thrust = max(-self.max_thrust, min(self.max_thrust, right_thrust))

        self.publish_thrusters(left_thrust, right_thrust)

    def publish_thrusters(self, left: float, right: float) -> None:
        """Publish thrust commands to both thrusters."""
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left
        right_msg.data = right

        self.left_thruster_pub.publish(left_msg)
        self.right_thruster_pub.publish(right_msg)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)
    node = ThrusterPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
