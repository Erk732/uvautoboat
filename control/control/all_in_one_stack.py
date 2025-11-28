#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    """
    Compute yaw angle from a geometry_msgs/msg/Quaternion.
    Returns yaw in radians within [-pi, pi].
    """
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AllInOneStack(Node):
    """
    Integrated planner + controller + lidar avoidance for WAM-V.

    使用的接口（全部是原项目里定义的那些）：

    Sub:
      - /wamv/pose                        geometry_msgs/PoseStamped
      - /planning/goal                    geometry_msgs/PoseStamped
      - /wamv/sensors/lidars/lidar_wamv/scan  sensor_msgs/LaserScan  (VRX 默认激光雷达, 名字可改参数)

    Pub:
      - /planning/path                    nav_msgs/Path
      - /wamv/thrusters/left/thrust      std_msgs/Float64
      - /wamv/thrusters/right/thrust     std_msgs/Float64
    """

    def __init__(self):
        super().__init__('all_in_one_stack')

        # ---------------- 参数 ----------------
        # 基础前进推力
        self.declare_parameter('forward_thrust', 400.0)
        # 航向 P 控制增益
        self.declare_parameter('kp_yaw', 600.0)
        # waypoint 切换距离
        self.declare_parameter('waypoint_tolerance', 3.0)
        # 最终 goal 的停止距离
        self.declare_parameter('goal_tolerance', 4.0)
        # 控制频率
        self.declare_parameter('control_rate', 20.0)
        # 直线路径离散步长
        self.declare_parameter('path_step', 5.0)

        # Lidar 参数（可后面调）
        self.declare_parameter('lidar_topic', '/wamv/sensors/lidars/lidar_wamv/scan')
        self.declare_parameter('front_angle_deg', 30.0)    # 正前方扇区角度
        self.declare_parameter('side_angle_deg', 60.0)     # 左右侧扇区角度
        self.declare_parameter('obstacle_slow_dist', 15.0) # 开始减速距离
        self.declare_parameter('obstacle_stop_dist', 8.0)  # 硬避障距离
        self.declare_parameter('avoid_turn_thrust', 350.0) # 原地转向推力
        self.declare_parameter('avoid_diff_gain', 40.0)    # 左右距离偏差增益

        self.forward_thrust = float(self.get_parameter('forward_thrust').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.path_step = float(self.get_parameter('path_step').value)

        lidar_topic = str(self.get_parameter('lidar_topic').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.obstacle_slow_dist = float(self.get_parameter('obstacle_slow_dist').value)
        self.obstacle_stop_dist = float(self.get_parameter('obstacle_stop_dist').value)
        self.avoid_turn_thrust = float(self.get_parameter('avoid_turn_thrust').value)
        self.avoid_diff_gain = float(self.get_parameter('avoid_diff_gain').value)

        # 推进器饱和
        self.thrust_min = -1000.0
        self.thrust_max = 1000.0

        # ---------------- 内部状态 ----------------
        self.current_pose: PoseStamped | None = None
        self.path: Path | None = None
        self.current_waypoint_idx: int = 0
        self.active: bool = False      # 是否在跟踪路径
        self.latest_scan: LaserScan | None = None

        # ---------------- Publisher ----------------
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
        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            10
        )

        # ---------------- Subscriber ----------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/wamv/pose',
            self.pose_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/planning/goal',
            self.goal_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            lidar_topic,
            self.scan_callback,
            10
        )

        # ---------------- 定时控制循环 ----------------
        dt = 1.0 / self.control_rate
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info('All-in-one planner+controller stack initialized.')
        self.get_logger().info(f'Lidar topic: {lidar_topic}')

    # =====================================================
    # 回调：位姿、目标、雷达
    # =====================================================

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def goal_callback(self, msg: PoseStamped):
        """
        收到 /planning/goal 后，在本节点内部生成直线路径并发布 /planning/path，
        同时启动控制。
        """
        if self.current_pose is None:
            self.get_logger().warn('No current pose yet, cannot generate path.')
            return

        start = self.current_pose.pose.position
        goal = msg.pose.position

        dx = goal.x - start.x
        dy = goal.y - start.y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self.get_logger().warn('Goal too close to current pose, ignoring.')
            return

        frame_id = msg.header.frame_id or self.current_pose.header.frame_id
        if frame_id == '':
            frame_id = 'world'

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        step = max(self.path_step, 0.5)
        num_steps = max(int(dist / step), 1)

        for i in range(num_steps + 1):
            t = float(i) / float(num_steps)
            px = start.x + t * dx
            py = start.y + t * dy

            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = px
            pose_stamped.pose.position.y = py
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # 不在这里用朝向

            path_msg.poses.append(pose_stamped)

        self.path = path_msg
        self.current_waypoint_idx = 0
        self.active = True

        # 对外发布 /planning/path，保持原架构接口
        self.path_pub.publish(self.path)

        self.get_logger().info(
            f'New goal received ({goal.x:.1f}, {goal.y:.1f}), '
            f'generated straight-line path with {len(self.path.poses)} waypoints.'
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # =====================================================
    # Lidar 分析
    # =====================================================

    def analyze_lidar(self):
        """
        返回 (front_min, left_min, right_min)，单位 m。
        front: [-front_angle, +front_angle]
        left:  [0, +side_angle]
        right: [-side_angle, 0]
        """
        if self.latest_scan is None:
            return None, None, None

        scan = self.latest_scan
        angle = scan.angle_min

        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')

        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r) or r <= 0.0:
                angle += scan.angle_increment
                continue

            if -self.front_angle <= angle <= self.front_angle:
                front_min = min(front_min, r)

            if 0.0 <= angle <= self.side_angle:
                left_min = min(left_min, r)

            if -self.side_angle <= angle <= 0.0:
                right_min = min(right_min, r)

            angle += scan.angle_increment

        if front_min == float('inf'):
            front_min = None
        if left_min == float('inf'):
            left_min = None
        if right_min == float('inf'):
            right_min = None

        return front_min, left_min, right_min

    # =====================================================
    # 主控制循环
    # =====================================================

    def control_loop(self):
        if not self.active:
            # 没有路径要跟踪
            self.publish_thrust(0.0, 0.0)
            return

        if self.current_pose is None or self.path is None:
            self.get_logger().warn_throttle(2.0, 'Waiting for pose and path...')
            self.publish_thrust(0.0, 0.0)
            return

        if self.current_waypoint_idx >= len(self.path.poses):
            self.get_logger().info_throttle(5.0, 'All waypoints reached. Stopping.')
            self.publish_thrust(0.0, 0.0)
            self.active = False
            return

        pose = self.current_pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)

        last_idx = len(self.path.poses) - 1
        target_pose = self.path.poses[self.current_waypoint_idx].pose
        tx = target_pose.position.x
        ty = target_pose.position.y

        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        # waypoint / goal 切换逻辑
        tolerance = self.goal_tolerance if self.current_waypoint_idx == last_idx \
            else self.waypoint_tolerance

        if dist < tolerance:
            if self.current_waypoint_idx == last_idx:
                self.get_logger().info(
                    f'Final goal reached (idx={self.current_waypoint_idx}, dist={dist:.2f} m).'
                )
                self.publish_thrust(0.0, 0.0)
                self.active = False
                return
            else:
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_idx} reached (dist={dist:.2f} m).'
                )
                self.current_waypoint_idx += 1
                return

        # ---------- 基础路径跟踪：航向 P 控制 ----------
        desired_yaw = math.atan2(dy, dx)
        e_yaw = normalize_angle(desired_yaw - yaw)

        T_forward = self.forward_thrust
        T_diff = self.kp_yaw * e_yaw

        left_cmd = T_forward - T_diff
        right_cmd = T_forward + T_diff

        # ---------- 雷达避障修正 ----------
        front_min, left_min, right_min = self.analyze_lidar()

        if front_min is not None:
            # 硬避障：太近，原地转
            if front_min < self.obstacle_stop_dist:
                if left_min is not None and right_min is not None:
                    turn_dir = 1.0 if left_min > right_min else -1.0
                elif left_min is not None:
                    turn_dir = 1.0
                elif right_min is not None:
                    turn_dir = -1.0
                else:
                    turn_dir = 1.0   # 没侧向信息就默认向左

                left_cmd = -turn_dir * self.avoid_turn_thrust
                right_cmd = turn_dir * self.avoid_turn_thrust

                self.get_logger().info_throttle(
                    2.0,
                    f'EMERGENCY AVOID: obstacle at {front_min:.1f} m, '
                    f'turning {"left" if turn_dir > 0 else "right"}.'
                )

            # 软避障：先减速，再略微偏转
            elif front_min < self.obstacle_slow_dist:
                denom = max(self.obstacle_slow_dist - self.obstacle_stop_dist, 0.1)
                scale = (front_min - self.obstacle_stop_dist) / denom
                scale = max(0.2, min(1.0, scale))

                left_cmd *= scale
                right_cmd *= scale

                diff_bias = 0.0
                if left_min is not None and right_min is not None:
                    diff_bias = (right_min - left_min) * self.avoid_diff_gain
                elif left_min is not None:
                    diff_bias = 0.5 * self.avoid_diff_gain
                elif right_min is not None:
                    diff_bias = -0.5 * self.avoid_diff_gain

                left_cmd -= diff_bias
                right_cmd += diff_bias

                self.get_logger().info_throttle(
                    5.0,
                    f'Obstacle ahead at {front_min:.1f} m: slowing (scale={scale:.2f}).'
                )

        # ---------- 推力饱和 ----------
        left_cmd = max(self.thrust_min, min(self.thrust_max, left_cmd))
        right_cmd = max(self.thrust_min, min(self.thrust_max, right_cmd))

        # 接近最终目标时整体降速
        if self.current_waypoint_idx == last_idx and dist < self.goal_tolerance * 1.5:
            scale = max(0.2, dist / (self.goal_tolerance * 1.5))
            left_cmd *= scale
            right_cmd *= scale

        self.publish_thrust(left_cmd, right_cmd)

    # =====================================================
    # 工具函数
    # =====================================================

    def publish_thrust(self, left: float, right: float):
        msg_left = Float64()
        msg_right = Float64()
        msg_left.data = float(left)
        msg_right.data = float(right)
        self.left_thruster_pub.publish(msg_left)
        self.right_thruster_pub.publish(msg_right)


def main(args=None):
    rclpy.init(args=args)
    node = AllInOneStack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down all_in_one_stack.')
    finally:
        node.publish_thrust(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
