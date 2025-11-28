#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import heapq
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header


class GridAStar:
    """
    Simple A* planner on a 2D occupancy grid.
    """

    def __init__(self, width: int, height: int, data: list, occ_threshold: int = 50):
        self.width = width
        self.height = height
        self.data = data
        self.occ_threshold = occ_threshold  # cells >= threshold are considered occupied

    def is_free(self, ix: int, iy: int) -> bool:
        """
        Check if the given grid index is free (within bounds and not occupied).
        """
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return False

        val = self.data[iy * self.width + ix]
        # -1 means unknown, treat as occupied to be safe
        if val < 0:
            return False
        return val < self.occ_threshold

    def neighbors(self, ix: int, iy: int):
        """
        8-connected grid neighbors.
        """
        # (dx, dy, move_cost)
        moves = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (-1, -1, math.sqrt(2.0)),
        ]

        for dx, dy, cost in moves:
            jx = ix + dx
            jy = iy + dy
            if self.is_free(jx, jy):
                yield jx, jy, cost

    @staticmethod
    def heuristic(ix: int, iy: int, gx: int, gy: int) -> float:
        """
        Heuristic: Euclidean distance.
        """
        return math.hypot(gx - ix, gy - iy)

    def plan(self, sx: int, sy: int, gx: int, gy: int) -> Optional[List[Tuple[int, int]]]:
        """
        Run A* from (sx, sy) to (gx, gy) in grid coordinates.
        Returns a list of (ix, iy) or None if no path.
        """
        start = (sx, sy)
        goal = (gx, gy)

        if not self.is_free(sx, sy):
            return None
        if not self.is_free(gx, gy):
            return None

        open_q = []
        heapq.heappush(open_q, (0.0, start))
        g_score = {start: 0.0}
        came_from = {}
        closed = set()

        while open_q:
            f, (ix, iy) = heapq.heappop(open_q)
            if (ix, iy) in closed:
                continue

            if (ix, iy) == goal:
                # Reconstruct path
                path = [(ix, iy)]
                while (ix, iy) in came_from:
                    ix, iy = came_from[(ix, iy)]
                    path.append((ix, iy))
                path.reverse()
                return path

            closed.add((ix, iy))

            for jx, jy, cost in self.neighbors(ix, iy):
                tentative = g_score[(ix, iy)] + cost
                if (jx, jy) not in g_score or tentative < g_score[(jx, jy)]:
                    g_score[(jx, jy)] = tentative
                    came_from[(jx, jy)] = (ix, iy)
                    f_new = tentative + self.heuristic(jx, jy, gx, gy)
                    heapq.heappush(open_q, (f_new, (jx, jy)))

        return None


class MapAStarPlanner(Node):
    """
    Map-based global planner using nav_msgs/OccupancyGrid and A*.
    Subscribes to /map, /wamv/pose and /planning/goal, publishes /planning/path.
    """

    def __init__(self):
        super().__init__('map_astar_planner')

        # Parameters
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('allow_unknown', False)

        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown').value)

        # Internal state
        self.map: Optional[OccupancyGrid] = None
        self.current_pose: Optional[PoseStamped] = None

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

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

        # Publisher
        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            10
        )

        self.get_logger().info('Map A* planner node initialized.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def map_callback(self, msg: OccupancyGrid):
        """
        Receive the occupancy grid map.
        """
        self.map = msg
        self.get_logger().info_once(
            f'Map received: size=({msg.info.width} x {msg.info.height}), '
            f'resolution={msg.info.resolution:.3f}'
        )

    def pose_callback(self, msg: PoseStamped):
        """
        Receive the current boat pose in map/world frame.
        """
        self.current_pose = msg

    def goal_callback(self, msg: PoseStamped):
        """
        Receive a new goal, then run A* on the map and publish a path.
        """
        if self.map is None:
            self.get_logger().warn('No map received yet, cannot plan.')
            return

        if self.current_pose is None:
            self.get_logger().warn('No current pose received yet, cannot plan.')
            return

        # Check frames
        map_frame = self.map.header.frame_id
        pose_frame = self.current_pose.header.frame_id
        goal_frame = msg.header.frame_id

        if pose_frame != map_frame:
            self.get_logger().warn(
                f'Pose frame "{pose_frame}" does not match map frame "{map_frame}". '
                'Planner assumes they are in the same frame.'
            )
        if goal_frame != map_frame:
            self.get_logger().warn(
                f'Goal frame "{goal_frame}" does not match map frame "{map_frame}". '
                'Planner assumes they are in the same frame.'
            )

        # Convert world coordinates to grid indices
        sx, sy = self.world_to_map(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y
        )
        gx, gy = self.world_to_map(
            msg.pose.position.x,
            msg.pose.position.y
        )

        if sx is None or gx is None:
            self.get_logger().warn('Start or goal outside of map bounds, cannot plan.')
            return

        self.get_logger().info(
            f'Planning from grid ({sx}, {sy}) to ({gx}, {gy}).'
        )

        # Run A*
        astar = GridAStar(
            self.map.info.width,
            self.map.info.height,
            self.map.data,
            occ_threshold=self.occupancy_threshold
        )
        path_grid = astar.plan(sx, sy, gx, gy)

        if path_grid is None:
            self.get_logger().warn('A* failed to find a path.')
            return

        self.get_logger().info(f'A* found a path with {len(path_grid)} grid points.')

        # Convert grid path to world coordinates and publish nav_msgs/Path
        path_msg = self.grid_path_to_ros_path(path_grid)
        self.path_pub.publish(path_msg)
        self.get_logger().info('Published /planning/path.')

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def world_to_map(self, wx: float, wy: float) -> Tuple[Optional[int], Optional[int]]:
        """
        Convert world coordinates (wx, wy) to grid indices (ix, iy).
        Returns (None, None) if outside map.
        """
        if self.map is None:
            return None, None

        origin: Pose = self.map.info.origin
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height

        # Map origin is at the bottom-left corner in the world frame.
        ix = int((wx - origin.position.x) / resolution)
        iy = int((wy - origin.position.y) / resolution)

        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            return None, None

        return ix, iy

    def map_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """
        Convert grid indices (ix, iy) back to world coordinates (wx, wy).
        """
        origin: Pose = self.map.info.origin
        resolution = self.map.info.resolution

        wx = origin.position.x + (ix + 0.5) * resolution
        wy = origin.position.y + (iy + 0.5) * resolution
        return wx, wy

    def grid_path_to_ros_path(self, grid_path: List[Tuple[int, int]]) -> Path:
        """
        Convert a list of grid indices into a nav_msgs/Path in world coordinates.
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map.header.frame_id

        for ix, iy in grid_path:
            wx, wy = self.map_to_world(ix, iy)
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header

            pose_stamped.pose.position.x = wx
            pose_stamped.pose.position.y = wy
            pose_stamped.pose.position.z = 0.0

            # Orientation is not strictly needed for the planner,
            # leave it as identity quaternion.
            pose_stamped.pose.orientation.w = 1.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0

            path_msg.poses.append(pose_stamped)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = MapAStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down map_astar_planner node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
