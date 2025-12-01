#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

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
        # 接近目标时减速距离
        self.declare_parameter('approach_slow_dist', 10.0)
        # 数据超时时间
        self.declare_parameter('pose_timeout', 1.0)
        self.declare_parameter('scan_timeout', 1.0)
        # 位姿话题，可替换为仿真/现实的发布名
        self.declare_parameter('pose_topic', '/wamv/pose')

        # Lidar 参数（可后面调）
        # 默认 VRX 有两个 scan 话题，primary 使用 /wamv/sensors/lidars/lidar_wamv_sensor/scan，
        # secondary 使用 /wamv/sensors/lidars/lidar_wamv/scan，自动双订阅取最新
        self.declare_parameter('lidar_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/scan')
        self.declare_parameter('lidar_topic_alt', '/wamv/sensors/lidars/lidar_wamv/scan')
        self.declare_parameter('front_angle_deg', 30.0)    # 正前方扇区角度
        self.declare_parameter('side_angle_deg', 60.0)     # 左右侧扇区角度
        self.declare_parameter('obstacle_slow_dist', 15.0) # 开始减速距离
        self.declare_parameter('obstacle_stop_dist', 8.0)  # 硬避障距离
        self.declare_parameter('avoid_turn_thrust', 350.0) # 原地转向推力
        self.declare_parameter('avoid_diff_gain', 40.0)    # 左右距离偏差增益
        # 规划避障参数
        self.declare_parameter('plan_avoid_margin', 5.0)

        self.forward_thrust = float(self.get_parameter('forward_thrust').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.path_step = float(self.get_parameter('path_step').value)
        self.approach_slow_dist = float(self.get_parameter('approach_slow_dist').value)

        lidar_topic = str(self.get_parameter('lidar_topic').value)
        lidar_topic_alt = str(self.get_parameter('lidar_topic_alt').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.obstacle_slow_dist = float(self.get_parameter('obstacle_slow_dist').value)
        self.obstacle_stop_dist = float(self.get_parameter('obstacle_stop_dist').value)
        self.avoid_turn_thrust = float(self.get_parameter('avoid_turn_thrust').value)
        self.avoid_diff_gain = float(self.get_parameter('avoid_diff_gain').value)
        self.plan_avoid_margin = float(self.get_parameter('plan_avoid_margin').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

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
            pose_topic,
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
        if lidar_topic_alt and lidar_topic_alt != lidar_topic:
            self.scan_sub_alt = self.create_subscription(
                LaserScan,
                lidar_topic_alt,
                self.scan_callback,
                10
            )
        else:
            self.scan_sub_alt = None

        # ---------------- 定时控制循环 ----------------
        dt = 1.0 / self.control_rate
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info('All-in-one planner+controller stack initialized.')
        self.get_logger().info(f'Lidar topic: {lidar_topic}')
        if self.scan_sub_alt:
            self.get_logger().info(f'Lidar topic (alt): {lidar_topic_alt}')
        self.get_logger().info(f'Pose topic: {pose_topic}')

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

        # 坐标系不一致时直接拒绝，避免生成错误路径
        if self.current_pose.header.frame_id and frame_id != self.current_pose.header.frame_id:
            self.get_logger().warn(
                f'Frame mismatch: goal in "{frame_id}", current pose in '
                f'"{self.current_pose.header.frame_id}". Ignoring goal.'
            )
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        step = max(self.path_step, 0.5)

        # 初步路径点（终点）
        path_points = [(goal.x, goal.y)]

        # 如果正前方有障碍，用当前激光快速生成一个侧向绕障点
        front_min, left_min, right_min = self.analyze_lidar()
        if front_min is not None and front_min < self.obstacle_slow_dist and dist > 1e-3:
            prefer_left = True
            if left_min is not None and right_min is not None:
                prefer_left = left_min >= right_min
            elif left_min is None and right_min is not None:
                prefer_left = False

            side = 1.0 if prefer_left else -1.0
            offset = max(self.obstacle_stop_dist + self.plan_avoid_margin, self.waypoint_tolerance * 2.0)
            # 垂直于目标方向的侧向偏移
            nx = -dy / dist
            ny = dx / dist
            detour_x = start.x + side * offset * nx
            detour_y = start.y + side * offset * ny
            path_points.insert(0, (detour_x, detour_y))

            self.get_logger().info(
                f'Obstacle detected at {front_min:.1f} m, adding detour waypoint '
                f'({detour_x:.1f}, {detour_y:.1f}) to avoid.'
            )

        # 逐段插值生成路径
        waypoints = [(start.x, start.y)] + path_points
        for idx in range(len(waypoints) - 1):
            sx, sy = waypoints[idx]
            gx, gy = waypoints[idx + 1]

            seg_dx = gx - sx
            seg_dy = gy - sy
            seg_dist = math.hypot(seg_dx, seg_dy)
            seg_steps = max(int(seg_dist / step), 1)

            for i in range(seg_steps + 1):
                if idx > 0 and i == 0:
                    # 避免重复上一个分段终点
                    continue
                t = float(i) / float(seg_steps)
                px = sx + t * seg_dx
                py = sy + t * seg_dy

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
            self.get_logger().warning('Waiting for pose and path...')
            self.publish_thrust(0.0, 0.0)
            return

        now = self.get_clock().now()

        # 当前位置超时则停止，等待新数据
        pose_time = Time.from_msg(self.current_pose.header.stamp)
        pose_age = (now - pose_time).nanoseconds / 1e9
        if pose_age > self.pose_timeout:
            self.get_logger().warning(
                f'Pose data stale ({pose_age:.2f}s > {self.pose_timeout:.2f}s), holding position.'
            )
            self.publish_thrust(0.0, 0.0)
            return

        # 激光超时同样停车，避免盲航
        if self.latest_scan is None:
            self.get_logger().warning('Waiting for lidar scan...')
            self.publish_thrust(0.0, 0.0)
            return

        scan_time = Time.from_msg(self.latest_scan.header.stamp)
        scan_age = (now - scan_time).nanoseconds / 1e9
        if scan_age > self.scan_timeout:
            self.get_logger().warning(
                f'Lidar data stale ({scan_age:.2f}s > {self.scan_timeout:.2f}s), holding position.'
            )
            self.publish_thrust(0.0, 0.0)
            return

        if self.current_waypoint_idx >= len(self.path.poses):
            self.get_logger().info('All waypoints reached. Stopping.')
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
        if self.approach_slow_dist > 0.0:
            T_forward *= max(0.1, min(1.0, dist / self.approach_slow_dist))
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

                self.get_logger().info(
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
                    norm = max(max(left_min, right_min), 1e-3)
                    diff_bias = (right_min - left_min) / norm * self.avoid_diff_gain
                elif left_min is not None:
                    diff_bias = 0.5 * self.avoid_diff_gain
                elif right_min is not None:
                    diff_bias = -0.5 * self.avoid_diff_gain

                left_cmd -= diff_bias
                right_cmd += diff_bias

                self.get_logger().info(
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
