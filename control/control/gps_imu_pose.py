#!/usr/bin/env python3
"""
Publish a PoseStamped estimated from GPS + IMU for the WAM-V.

Uses the first GPS fix as origin (0,0) in a local ENU plane and yaw from IMU.
This is a lightweight bridge for controllers expecting PoseStamped when the
simulation only provides /wamv/sensors/gps/gps/fix and /wamv/sensors/imu/imu/data.
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ExtrapolationException


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def latlon_to_local_meters(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Convert lat/lon to local ENU meters relative to (lat0, lon0)."""
    R = 6371000.0
    d_lat = math.radians(lat - lat0)
    d_lon = math.radians(lon - lon0)
    lat0_rad = math.radians(lat0)
    x = d_lon * R * math.cos(lat0_rad)
    y = d_lat * R
    return x, y


class GpsImuPose(Node):
    def __init__(self) -> None:
        super().__init__('gps_imu_pose')

        self.declare_parameter('gps_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('imu_topic', '/wamv/sensors/imu/imu/data')
        self.declare_parameter('pose_topic', '/wamv/pose')
        self.declare_parameter('frame_id', 'world')
        # Default TF frame in VRX is wamv/wamv/base_link
        self.declare_parameter('base_link_frame', 'wamv/wamv/base_link')
        self.declare_parameter('publish_rate', 10.0)
        # Use TF directly to publish world pose (ignores GPS/IMU relative origin)
        self.declare_parameter('use_tf_pose', False)
        self.declare_parameter('tf_target_frame', 'world')
        self.declare_parameter('tf_source_frame', 'wamv/wamv/base_link')

        gps_topic = str(self.get_parameter('gps_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_link_frame = str(self.get_parameter('base_link_frame').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.use_tf_pose = bool(self.get_parameter('use_tf_pose').value)
        self.tf_target_frame = str(self.get_parameter('tf_target_frame').value)
        self.tf_source_frame = str(self.get_parameter('tf_source_frame').value)

        self.origin: Optional[Tuple[float, float]] = None
        self.latest_gps: Optional[NavSatFix] = None
        self.latest_imu: Optional[Imu] = None

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        if not self.use_tf_pose:
            self.create_subscription(NavSatFix, gps_topic, self.gps_callback, 10)
            self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        dt = 1.0 / max(publish_rate, 1.0)
        self.create_timer(dt, self.publish_pose)

        if self.use_tf_pose:
            self.get_logger().info(
                f'TF->Pose bridge started. source={self.tf_source_frame}, target={self.tf_target_frame}, output={pose_topic}'
            )
        else:
            self.get_logger().info(
                f'GPS->Pose bridge started. gps={gps_topic}, imu={imu_topic}, output={pose_topic}, frame={self.frame_id}'
            )

    def gps_callback(self, msg: NavSatFix) -> None:
        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'Set GPS origin lat={msg.latitude:.7f}, lon={msg.longitude:.7f} as (0,0).'
            )
        self.latest_gps = msg

    def imu_callback(self, msg: Imu) -> None:
        self.latest_imu = msg

    def publish_pose(self) -> None:
        if self.use_tf_pose:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.tf_target_frame,
                    self.tf_source_frame,
                    rclpy.time.Time())
            except (LookupException, ExtrapolationException):
                return

            pose = PoseStamped()
            pose.header = tf.header
            pose.header.frame_id = self.tf_target_frame
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            self.pose_pub.publish(pose)
            return

        if self.latest_gps is None or self.latest_imu is None or self.origin is None:
            return

        x, y = latlon_to_local_meters(self.origin[0], self.origin[1], self.latest_gps.latitude, self.latest_gps.longitude)
        yaw = yaw_from_quaternion(self.latest_imu.orientation)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # flat-earth yaw only; roll/pitch ignored
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.pose_pub.publish(pose)

        # Broadcast TF world -> base_link for visualization and consumers needing TF
        if self.tf_broadcaster:
            tf_msg = TransformStamped()
            tf_msg.header = pose.header
            tf_msg.child_frame_id = self.base_link_frame
            tf_msg.transform.translation.x = pose.pose.position.x
            tf_msg.transform.translation.y = pose.pose.position.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gps_imu_pose bridge.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
