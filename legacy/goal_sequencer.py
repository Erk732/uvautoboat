#!/usr/bin/env python3
"""
Legacy: publish a sequence of goals to /planning/goal, driven by local pose feedback.
Kept here for reference; not installed or wired into the current launch flow.
"""

import math
import os
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def parse_goals(goals_str: str) -> List[Tuple[float, float]]:
    goals: List[Tuple[float, float]] = []
    for raw in goals_str.split(';'):
        part = raw.strip()
        if not part:
            continue
        xy = part.split(',')
        if len(xy) != 2:
            continue
        try:
            goals.append((float(xy[0]), float(xy[1])))
        except ValueError:
            continue
    return goals


def parse_goals_file(path: str) -> List[Tuple[float, float]]:
    goals: List[Tuple[float, float]] = []
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            # support "x,y" or "x y"
            if ',' in stripped:
                parts = stripped.split(',')
            else:
                parts = stripped.split()
            if len(parts) != 2:
                continue
            try:
                goals.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    return goals


class GoalSequencer(Node):
    def __init__(self) -> None:
        super().__init__('goal_sequencer')

        self.declare_parameter('goals', '130,0;130,50;80,60')
        self.declare_parameter('goal_file', '')
        self.declare_parameter('goal_topic', '/planning/goal')
        self.declare_parameter('pose_topic', '/wamv/pose_filtered')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('goal_tolerance', 1.0)
        self.declare_parameter('timer_period', 0.5)

        goal_file = str(self.get_parameter('goal_file').value)
        goals_param = str(self.get_parameter('goals').value)
        self.goals: List[Tuple[float, float]] = []

        if goal_file:
            if os.path.isfile(goal_file):
                try:
                    self.goals = parse_goals_file(goal_file)
                    self.get_logger().info(
                        f'Loaded {len(self.goals)} goals from file: {goal_file}'
                    )
                except Exception as e:
                    self.get_logger().warning(
                        f'Failed to read goal_file "{goal_file}": {e}. Falling back to "goals" param.'
                    )
            else:
                self.get_logger().warning(
                    f'goal_file "{goal_file}" does not exist. Falling back to "goals" param.'
                )

        if not self.goals:
            self.goals = parse_goals(goals_param)
            if self.goals:
                self.get_logger().info(f'Loaded {len(self.goals)} goals from "goals" param.')
            else:
                self.get_logger().warning('No goals parsed from "goals" parameter.')

        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 10
        )

        period = float(self.get_parameter('timer_period').value)
        self.timer = self.create_timer(period, self.tick)

        self.pose: PoseStamped | None = None
        self.goal_idx = 0
        self.goal_active = False

        if not self.goals:
            self.get_logger().warning('No goals parsed from "goals" parameter.')
        else:
            self.get_logger().info(
                f'Loaded {len(self.goals)} goals. Starting when pose data arrives...'
            )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.pose = msg

    def publish_goal(self, gx: float, gy: float) -> None:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'Published goal {self.goal_idx + 1}/{len(self.goals)}: ({gx:.1f}, {gy:.1f})'
        )

    def tick(self) -> None:
        if not self.goals:
            return
        if self.goal_idx >= len(self.goals):
            return
        if self.pose is None:
            # Wait for pose before sending the first goal
            return

        current_goal = self.goals[self.goal_idx]

        if not self.goal_active:
            self.publish_goal(current_goal[0], current_goal[1])
            self.goal_active = True
            return

        # Check arrival
        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        dx = current_goal[0] - px
        dy = current_goal[1] - py
        dist = math.hypot(dx, dy)

        if dist <= self.goal_tolerance:
            self.get_logger().info(
                f'Goal {self.goal_idx + 1} reached (dist={dist:.2f} m).'
            )
            self.goal_idx += 1
            self.goal_active = False
            if self.goal_idx >= len(self.goals):
                self.get_logger().info('All goals completed.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
