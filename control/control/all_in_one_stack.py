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

    Interfaces:

    Subscriptions:
      - /wamv/pose                        geometry_msgs/PoseStamped
      - /planning/goal                    geometry_msgs/PoseStamped
      - /wamv/sensors/lidars/lidar_wamv/scan  sensor_msgs/LaserScan  (VRX default lidar; can be changed via params)

    Publications:
      - /planning/path                    nav_msgs/Path
      - /wamv/thrusters/left/thrust      std_msgs/Float64
      - /wamv/thrusters/right/thrust     std_msgs/Float64
    """

    def __init__(self):
        super().__init__('all_in_one_stack')

        # ---------------- Parameters ----------------
        # Base forward thrust
        self.declare_parameter('forward_thrust', 400.0)
        # Heading P gain
        self.declare_parameter('kp_yaw', 600.0)
        # Waypoint switch distance
        self.declare_parameter('waypoint_tolerance', 3.0)
        # Stop distance for final goal
        self.declare_parameter('goal_tolerance', 4.0)
        # Control frequency
        self.declare_parameter('control_rate', 20.0)
        # Straight path step length
        self.declare_parameter('path_step', 5.0)
        # Slow-down distance near goal
        self.declare_parameter('approach_slow_dist', 10.0)
        # Forward thrust heading alignment threshold (deg)
        self.declare_parameter('heading_align_thresh_deg', 20.0)
        # Overshoot margin at final goal
        self.declare_parameter('overshoot_margin', 1.0)
        # Stuck detection (no progress toward goal)
        self.declare_parameter('stuck_timeout', 8.0)
        self.declare_parameter('stuck_progress_epsilon', 0.1)
        # Recovery behavior when stuck
        self.declare_parameter('recover_reverse_time', 3.0)
        self.declare_parameter('recover_turn_time', 3.0)
        self.declare_parameter('recover_reverse_thrust', -200.0)
        # Lidar debug log interval (seconds); 0 disables
        self.declare_parameter('lidar_log_interval', 2.0)
        # Data timeouts
        self.declare_parameter('pose_timeout', 1.0)
        self.declare_parameter('scan_timeout', 1.0)
        # Pose topic (can be remapped for sim/real)
        self.declare_parameter('pose_topic', '/wamv/pose')

        # Lidar params
        # VRX has two scan topics; primary uses /wamv/sensors/lidars/lidar_wamv_sensor/scan,
        # secondary uses /wamv/sensors/lidars/lidar_wamv/scan; subscribe to both and use latest
        self.declare_parameter('lidar_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/scan')
        self.declare_parameter('lidar_topic_alt', '/wamv/sensors/lidars/lidar_wamv/scan')
        self.declare_parameter('front_angle_deg', 30.0)    # Front sector angle
        self.declare_parameter('side_angle_deg', 60.0)     # Side sector angle
        self.declare_parameter('obstacle_slow_dist', 15.0) # Slow-down distance
        self.declare_parameter('obstacle_stop_dist', 8.0)  # Hard-stop distance
        self.declare_parameter('avoid_turn_thrust', 350.0) # In-place turn thrust
        self.declare_parameter('avoid_diff_gain', 40.0)    # Left/right bias gain
        self.declare_parameter('avoid_clear_margin', 3.0)  # Clearance to exit hard avoid
        self.declare_parameter('avoid_max_turn_time', 5.0) # Max time to stay in hard avoid
        # Hull radius (m) used to inflate obstacles for clearance
        self.declare_parameter('hull_radius', 1.5)
        # Planning avoidance params
        self.declare_parameter('plan_avoid_margin', 5.0)

        self.forward_thrust = float(self.get_parameter('forward_thrust').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.path_step = float(self.get_parameter('path_step').value)
        self.approach_slow_dist = float(self.get_parameter('approach_slow_dist').value)
        self.heading_align_thresh = math.radians(float(self.get_parameter('heading_align_thresh_deg').value))
        self.overshoot_margin = float(self.get_parameter('overshoot_margin').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.stuck_progress_epsilon = float(self.get_parameter('stuck_progress_epsilon').value)
        self.recover_reverse_time = float(self.get_parameter('recover_reverse_time').value)
        self.recover_turn_time = float(self.get_parameter('recover_turn_time').value)
        self.recover_reverse_thrust = float(self.get_parameter('recover_reverse_thrust').value)
        self.lidar_log_interval = float(self.get_parameter('lidar_log_interval').value)

        lidar_topic = str(self.get_parameter('lidar_topic').value)
        lidar_topic_alt = str(self.get_parameter('lidar_topic_alt').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.obstacle_slow_dist = float(self.get_parameter('obstacle_slow_dist').value)
        self.obstacle_stop_dist = float(self.get_parameter('obstacle_stop_dist').value)
        self.avoid_turn_thrust = float(self.get_parameter('avoid_turn_thrust').value)
        self.avoid_diff_gain = float(self.get_parameter('avoid_diff_gain').value)
        self.avoid_clear_margin = float(self.get_parameter('avoid_clear_margin').value)
        self.avoid_max_turn_time = float(self.get_parameter('avoid_max_turn_time').value)
        self.hull_radius = float(self.get_parameter('hull_radius').value)
        self.plan_avoid_margin = float(self.get_parameter('plan_avoid_margin').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

        # Thruster saturation
        self.thrust_min = -1000.0
        self.thrust_max = 1000.0

        # ---------------- Internal state ----------------
        self.current_pose: PoseStamped | None = None
        self.path: Path | None = None
        self.current_waypoint_idx: int = 0
        self.active: bool = False      # Whether tracking a path
        self.latest_scan: LaserScan | None = None
        self.min_goal_dist: float = float('inf')
        self.last_progress_time: float = 0.0
        self.last_goal_dist: float = float('inf')
        self.last_lidar_log_time: float = 0.0
        self.recovering: bool = False
        self.recover_phase: str = ''
        self.recover_start_time: float = 0.0
        self.recover_turn_dir: float = 1.0
        self.avoid_mode: str = ''
        self.avoid_start_time: float = 0.0
        self.avoid_turn_dir: float = 1.0

        # ---------------- Publishers ----------------
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

        # ---------------- Subscribers ----------------
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

        # ---------------- Timer control loop ----------------
        dt = 1.0 / self.control_rate
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info('All-in-one planner+controller stack initialized.')
        self.get_logger().info(f'Lidar topic: {lidar_topic}')
        if self.scan_sub_alt:
            self.get_logger().info(f'Lidar topic (alt): {lidar_topic_alt}')
        self.get_logger().info(f'Pose topic: {pose_topic}')

    # =====================================================
    # Callbacks: pose, goal, lidar
    # =====================================================

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def goal_callback(self, msg: PoseStamped):
        """
        On receiving /planning/goal, generate a straight-line path, publish /planning/path,
        and start control.
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

        # Reject goals if frame mismatches to avoid wrong paths
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

        # Initial path points (goal)
        path_points = [(goal.x, goal.y)]

        # If obstacle ahead, add a side detour waypoint using current lidar
        front_min, left_min, right_min = self.analyze_lidar()
        if front_min is not None and front_min < self.obstacle_slow_dist and dist > 1e-3:
            prefer_left = True
            if left_min is not None and right_min is not None:
                prefer_left = left_min >= right_min
            elif left_min is None and right_min is not None:
                prefer_left = False

            side = 1.0 if prefer_left else -1.0
            offset = max(self.obstacle_stop_dist + self.plan_avoid_margin, self.waypoint_tolerance * 2.0)
            # Lateral offset perpendicular to goal direction
            nx = -dy / dist
            ny = dx / dist
            detour_x = start.x + side * offset * nx
            detour_y = start.y + side * offset * ny
            path_points.insert(0, (detour_x, detour_y))

            self.get_logger().info(
                f'Obstacle detected at {front_min:.1f} m, adding detour waypoint '
                f'({detour_x:.1f}, {detour_y:.1f}) to avoid.'
            )

        # Interpolate path segment by segment
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
                    # Avoid duplicating previous segment end point
                    continue
                t = float(i) / float(seg_steps)
                px = sx + t * seg_dx
                py = sy + t * seg_dy

                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = px
                pose_stamped.pose.position.y = py
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0  # Heading is not set here

                path_msg.poses.append(pose_stamped)

        self.path = path_msg
        self.current_waypoint_idx = 0
        self.active = True
        self.min_goal_dist = float('inf')
        self.last_goal_dist = float('inf')
        self.last_progress_time = self.get_clock().now().nanoseconds / 1e9
        self.recovering = False
        self.recover_phase = ''
        self.recover_turn_dir = 1.0
        self.avoid_mode = ''

        # Publish /planning/path to keep external interface
        self.path_pub.publish(self.path)

        self.get_logger().info(
            f'New goal received ({goal.x:.1f}, {goal.y:.1f}), '
            f'generated straight-line path with {len(self.path.poses)} waypoints.'
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # =====================================================
    # Lidar analysis
    # =====================================================

    def analyze_lidar(self):
        """
        Return (front_min, left_min, right_min) in meters.
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

        # If nothing valid, fall back to sensor range_max to avoid None/-1
        fallback = scan.range_max if not math.isinf(scan.range_max) else 50.0
        if front_min == float('inf'):
            front_min = fallback
        if left_min == float('inf'):
            left_min = fallback
        if right_min == float('inf'):
            right_min = fallback

        return front_min, left_min, right_min

    # =====================================================
    # Main control loop
    # =====================================================

    def control_loop(self):
        if not self.active:
            # No path to track
            self.publish_thrust(0.0, 0.0)
            return

        if self.current_pose is None or self.path is None:
            self.get_logger().warning('Waiting for pose and path...')
            self.publish_thrust(0.0, 0.0)
            return

        now = self.get_clock().now()

        # Stop if pose is stale; wait for fresh data
        pose_time = Time.from_msg(self.current_pose.header.stamp)
        pose_age = (now - pose_time).nanoseconds / 1e9
        if pose_age > self.pose_timeout:
            self.get_logger().warning(
                f'Pose data stale ({pose_age:.2f}s > {self.pose_timeout:.2f}s), holding position.'
            )
            self.publish_thrust(0.0, 0.0)
            return

        # Stop if lidar is stale; avoid blind sailing
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

        # Overshoot detection: if distance starts increasing after getting closer, stop
        if self.current_waypoint_idx == last_idx:
            if dist < self.min_goal_dist:
                self.min_goal_dist = dist
            elif self.min_goal_dist < float('inf') and dist > self.min_goal_dist + self.overshoot_margin:
                self.get_logger().info(
                    f'Overshoot detected at goal (closest {self.min_goal_dist:.2f} m, now {dist:.2f} m). Stopping.'
                )
                self.publish_thrust(0.0, 0.0)
                self.active = False
                return

        # Waypoint / goal switching logic
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
                # Reset progress tracking for the new segment
                self.last_goal_dist = float('inf')
                self.last_progress_time = self.get_clock().now().nanoseconds / 1e9
                self.recovering = False
                self.recover_phase = ''
                return

        # ---------- Basic path tracking: heading P control ----------
        desired_yaw = math.atan2(dy, dx)
        e_yaw = normalize_angle(desired_yaw - yaw)

        # Recovery behavior (stuck) or hard-avoid behavior (cul-de-sac handling)
        if self.recovering:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if self.recover_phase == 'reverse':
                if (now_s - self.recover_start_time) < self.recover_reverse_time:
                    self.publish_thrust(self.recover_reverse_thrust, self.recover_reverse_thrust)
                    return
                # switch to turn phase
                self.recover_phase = 'turn'
                self.recover_start_time = now_s
            if self.recover_phase == 'turn':
                if (now_s - self.recover_start_time) < self.recover_turn_time:
                    turn_cmd = self.avoid_turn_thrust * self.recover_turn_dir
                    self.publish_thrust(-turn_cmd, turn_cmd)
                    return
                # end recovery
                self.recovering = False
                self.recover_phase = ''
                self.last_goal_dist = float('inf')
                self.last_progress_time = now_s
                # let normal control resume after recovery

        # Stuck detection: if no progress toward goal within timeout, stop
        now_s = self.get_clock().now().nanoseconds / 1e9
        if dist < self.last_goal_dist - self.stuck_progress_epsilon:
            self.last_goal_dist = dist
            self.last_progress_time = now_s
        elif (now_s - self.last_progress_time) > self.stuck_timeout and dist > self.goal_tolerance:
            self.get_logger().warning(
                f'Stuck detected: no progress for {now_s - self.last_progress_time:.1f}s (dist={dist:.2f} m). Starting recovery.'
            )
            self.recovering = True
            self.recover_phase = 'reverse'
            self.recover_start_time = now_s
            # Choose turn direction based on lidar
            front_min, left_min, right_min = self.analyze_lidar()
            if left_min is not None and right_min is not None:
                self.recover_turn_dir = 1.0 if left_min >= right_min else -1.0
            elif left_min is not None:
                self.recover_turn_dir = 1.0
            elif right_min is not None:
                self.recover_turn_dir = -1.0
            else:
                self.recover_turn_dir = 1.0
            self.last_goal_dist = float('inf')
            self.last_progress_time = now_s
            return

        T_forward = self.forward_thrust
        if self.approach_slow_dist > 0.0:
            T_forward *= max(0.1, min(1.0, dist / self.approach_slow_dist))
        if self.heading_align_thresh > 0.0 and abs(e_yaw) > self.heading_align_thresh:
            # Reduce forward thrust when heading error is large to prioritize turning toward goal
            T_forward *= 0.2
        T_diff = self.kp_yaw * e_yaw

        left_cmd = T_forward - T_diff
        right_cmd = T_forward + T_diff

        # ---------- Lidar-based avoidance correction ----------
        front_min, left_min, right_min = self.analyze_lidar()
        if self.lidar_log_interval > 0.0:
            now_log = self.get_clock().now().nanoseconds / 1e9
            if now_log - self.last_lidar_log_time >= self.lidar_log_interval:
                self.last_lidar_log_time = now_log
                self.get_logger().info(
                    f'Lidar mins (m): front={front_min if front_min is not None else -1:.2f}, '
                    f'left={left_min if left_min is not None else -1:.2f}, '
                    f'right={right_min if right_min is not None else -1:.2f}'
                )

        # Hard-avoid state machine to avoid oscillation
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.avoid_mode in ('reverse', 'turn'):
            # sequential reverse then turn based on side clearance
            if self.avoid_mode == 'reverse':
                if (now_s - self.avoid_start_time) < self.recover_reverse_time:
                    self.publish_thrust(self.recover_reverse_thrust, self.recover_reverse_thrust)
                    return
                self.avoid_mode = 'turn'
                self.avoid_start_time = now_s
            if self.avoid_mode == 'turn':
                clear_dist = self.obstacle_stop_dist + self.avoid_clear_margin
                time_in_turn = now_s - self.avoid_start_time
                if (front_min is None or front_min > clear_dist) or (time_in_turn > self.avoid_max_turn_time):
                    self.avoid_mode = ''
                    self.avoid_start_time = 0.0
                else:
                    turn_cmd = self.avoid_turn_thrust * self.avoid_turn_dir
                    self.publish_thrust(-turn_cmd, turn_cmd)
                    return

        if front_min is not None:
            # Inflate obstacles by hull radius for clearance
            front_min_eff = max(0.0, front_min - self.hull_radius)
            left_min_eff = max(0.0, left_min - self.hull_radius) if left_min is not None else None
            right_min_eff = max(0.0, right_min - self.hull_radius) if right_min is not None else None

            # Hard avoidance: too close, turn in place
            if front_min_eff < self.obstacle_stop_dist:
                if left_min_eff is not None and right_min_eff is not None:
                    turn_dir = 1.0 if left_min_eff > right_min_eff else -1.0
                elif left_min_eff is not None:
                    turn_dir = 1.0
                elif right_min_eff is not None:
                    turn_dir = -1.0
                else:
                    turn_dir = 1.0   # Default turn left without side info

                self.avoid_mode = 'reverse'
                self.avoid_start_time = now_s
                self.avoid_turn_dir = turn_dir

                left_cmd = self.recover_reverse_thrust
                right_cmd = self.recover_reverse_thrust

                self.get_logger().info(
                    f'EMERGENCY AVOID: obstacle at {front_min:.1f} m, '
                    f'reversing then turning {"left" if turn_dir > 0 else "right"}.'
                )

            # Soft avoidance: slow down, then bias
            elif front_min_eff < self.obstacle_slow_dist:
                denom = max(self.obstacle_slow_dist - self.obstacle_stop_dist, 0.1)
                scale = (front_min_eff - self.obstacle_stop_dist) / denom
                scale = max(0.2, min(1.0, scale))

                left_cmd *= scale
                right_cmd *= scale

                diff_bias = 0.0
                if left_min_eff is not None and right_min_eff is not None:
                    norm = max(max(left_min_eff, right_min_eff), 1e-3)
                    diff_bias = (right_min_eff - left_min_eff) / norm * self.avoid_diff_gain
                elif left_min_eff is not None:
                    diff_bias = 0.5 * self.avoid_diff_gain
                elif right_min_eff is not None:
                    diff_bias = -0.5 * self.avoid_diff_gain

                left_cmd -= diff_bias
                right_cmd += diff_bias

                self.get_logger().info(
                    f'Obstacle ahead at {front_min:.1f} m: slowing (scale={scale:.2f}).'
                )

        # ---------- Thrust saturation ----------
        left_cmd = max(self.thrust_min, min(self.thrust_max, left_cmd))
        right_cmd = max(self.thrust_min, min(self.thrust_max, right_cmd))

        # Additional slow-down near final goal
        if self.current_waypoint_idx == last_idx and dist < self.goal_tolerance * 1.5:
            scale = max(0.2, dist / (self.goal_tolerance * 1.5))
            left_cmd *= scale
            right_cmd *= scale

        self.publish_thrust(left_cmd, right_cmd)

    # =====================================================
    # Utility functions
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
