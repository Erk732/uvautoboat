#!/usr/bin/env python3
"""
Atlantis Planner - Mission Planning with Dynamic A* Replanning

Features:
- Lawnmower pattern generation
- Dynamic A* replanning when obstacles block path
- Continuous obstacle monitoring
- Handles replan requests from controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Empty
from sensor_msgs.msg import NavSatFix, Imu
import json
import threading
import time
import math
import heapq

# A* SOLVER 
class AStarSolver:
    def __init__(self, resolution=3.0, safety_margin=5.0):
        self.resolution = resolution
        self.safety_margin = safety_margin

    def world_to_grid(self, x, y, min_x, min_y):
        return int((x - min_x)/self.resolution), int((y - min_y)/self.resolution)

    def grid_to_world(self, gx, gy, min_x, min_y):
        return (gx*self.resolution)+min_x, (gy*self.resolution)+min_y

    def plan(self, start, goal, obstacles, b_min, b_max):
        """
        A* pathfinding algorithm
        Returns list of waypoints from start to goal avoiding obstacles
        """
        padding = 40.0
        min_x = min(start[0], goal[0], b_min[0]) - padding
        max_x = max(start[0], goal[0], b_max[0]) + padding
        min_y = min(start[1], goal[1], b_min[1]) - padding
        max_y = max(start[1], goal[1], b_max[1]) + padding
        
        start_node = self.world_to_grid(start[0], start[1], min_x, min_y)
        goal_node = self.world_to_grid(goal[0], goal[1], min_x, min_y)
        
        # Build blocked set with safety margin
        blocked = set()
        for ox, oy in obstacles:
            if min_x < ox < max_x and min_y < oy < max_y:
                ogx, ogy = self.world_to_grid(ox, oy, min_x, min_y)
                steps = int(self.safety_margin/self.resolution)
                for dx in range(-steps, steps+1):
                    for dy in range(-steps, steps+1):
                        if math.hypot(dx, dy) * self.resolution <= self.safety_margin:
                            blocked.add((ogx+dx, ogy+dy))

        # A* algorithm
        open_set = [(0, start_node)]
        came_from = {}
        g_score = {start_node: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_node:
                # Reconstruct path
                path = []
                while current in came_from:
                    wx, wy = self.grid_to_world(current[0], current[1], min_x, min_y)
                    path.append((wx, wy))
                    current = came_from[current]
                path.reverse()
                return path

            # Check 8 directions
            for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0]+dx, current[1]+dy)
                
                if neighbor in blocked:
                    continue
                
                # Cost to move to neighbor
                move_cost = math.hypot(dx, dy)
                tent_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tent_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tent_g
                    # f = g + h (heuristic)
                    h = math.hypot(neighbor[0]-goal_node[0], neighbor[1]-goal_node[1])
                    f = tent_g + h
                    heapq.heappush(open_set, (f, neighbor))
        
        return []  # No path found


class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')
        
        # Parameters
        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 20.0)
        self.declare_parameter('lanes', 4)
        self.declare_parameter('geo_min_x', -50.0)
        self.declare_parameter('geo_max_x', 200.0)
        self.declare_parameter('geo_min_y', -100.0)
        self.declare_parameter('geo_max_y', 100.0)
        self.declare_parameter('planner_safe_dist', 12.0)
        self.declare_parameter('replan_check_interval', 1.0)  # Check every 1 second

        # Communication
        self.pub_target = self.create_publisher(String, '/planning/current_target', 10)
        self.pub_status = self.create_publisher(String, '/planning/mission_status', 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)
        self.create_subscription(String, '/perception/obstacle_info', self.obs_cb, 10)
        self.create_subscription(String, '/control/replan_request', self.replan_request_cb, 10)
        
        # State
        self.waypoints = []
        self.wp_index = 0
        self.start_gps = None
        self.local_pos = (0.0, 0.0)
        self.yaw = 0.0
        self.obstacles = []
        self.obstacles_changed = False
        self.state = "IDLE"
        self.last_replan_check = None
        
        # Track which segments are A* paths
        self.astar_segments = set()  # Set of waypoint indices that are part of A* paths
        
        self.astar = AStarSolver(resolution=3.0, safety_margin=8.0)
        
        # Main control loop
        self.create_timer(0.1, self.loop)
        
        # Replan check timer
        self.create_timer(self.get_parameter('replan_check_interval').value, self.check_and_replan)
        
        # Input thread
        threading.Thread(target=self.input_loop, daemon=True).start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Atlantis Planner (Dynamic A* Enabled)")
        self.get_logger().info("=" * 60)

    def input_loop(self):
        """Simple command interface"""
        time.sleep(1)
        print("\nCOMMANDS:")
        print("  'r' = Run mission (generate path and start)")
        print("  's' = Stop mission")
        print("  'p' = Print current status")
        while rclpy.ok():
            try:
                cmd = input("> ").strip().lower()
                if cmd == 'r': 
                    self.plan_initial_path()
                    self.state = "DRIVING"
                    self.wp_index = 0
                    self.get_logger().info("Mission started!")
                elif cmd == 's': 
                    self.state = "IDLE"
                    self.get_logger().info("Mission stopped")
                elif cmd == 'p':
                    self.print_status()
            except: 
                pass

    def print_status(self):
        """Print current planner status"""
        print(f"\n{'='*50}")
        print(f"State: {self.state}")
        print(f"Position: ({self.local_pos[0]:.1f}, {self.local_pos[1]:.1f})")
        print(f"Waypoint: {self.wp_index}/{len(self.waypoints)}")
        print(f"Obstacles: {len(self.obstacles)}")
        print(f"A* segments: {len(self.astar_segments)}")
        if self.wp_index < len(self.waypoints):
            tx, ty = self.waypoints[self.wp_index]
            dist = math.hypot(tx - self.local_pos[0], ty - self.local_pos[1])
            print(f"Next WP: ({tx:.1f}, {ty:.1f}) - {dist:.1f}m away")
        print(f"{'='*50}\n")

    def gps_cb(self, msg):
        """GPS callback"""
        if not self.start_gps: 
            self.start_gps = (msg.latitude, msg.longitude)
        
        R = 6371000.0  # Earth radius
        dx = math.radians(msg.latitude - self.start_gps[0]) * R
        dy = math.radians(msg.longitude - self.start_gps[1]) * R * math.cos(math.radians(self.start_gps[0]))
        self.local_pos = (dx, dy)

    def imu_cb(self, msg):
        """IMU callback"""
        q = msg.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def obs_cb(self, msg):
        """Obstacle callback - convert local obstacles to global coordinates"""
        try:
            data = json.loads(msg.data)
            new_obstacles = []
            cx, cy = math.cos(self.yaw), math.sin(self.yaw)
            
            for c in data.get('clusters', []):
                # Transform from local (sensor frame) to global coordinates
                gx = self.local_pos[0] + (c['x']*cx - c['y']*cy)
                gy = self.local_pos[1] + (c['x']*cy + c['y']*cx)
                new_obstacles.append((gx, gy))
            
            # Check if obstacles changed significantly
            if len(new_obstacles) != len(self.obstacles):
                self.obstacles_changed = True
            else:
                # Check if any obstacle moved significantly
                for new_ob in new_obstacles:
                    min_dist = min([math.hypot(new_ob[0]-old[0], new_ob[1]-old[1]) 
                                   for old in self.obstacles] + [999])
                    if min_dist > 5.0:  # 5m threshold for "changed"
                        self.obstacles_changed = True
                        break
            
            self.obstacles = new_obstacles
            
        except Exception as e:
            self.get_logger().warn(f"Obstacle parse error: {e}")

    def replan_request_cb(self, msg):
        """Handle replan requests from controller"""
        try:
            data = json.loads(msg.data)
            if data.get('type') == 'replan' and self.state == "DRIVING":
                self.get_logger().info("Replan requested by controller")
                self.replan_current_segment()
        except Exception as e:
            self.get_logger().warn(f"Replan request parse error: {e}")

    def loop(self):
        """Main control loop"""
        # Publish mission status
        msg = String()
        msg.data = json.dumps({"state": self.state})
        self.pub_status.publish(msg)
        
        if self.state != "DRIVING" or not self.waypoints: 
            return
        
        # Check if mission complete
        if self.wp_index >= len(self.waypoints):
            self.state = "FINISHED"
            self.get_logger().info("Mission Complete!")
            return

        # Publish current target
        tx, ty = self.waypoints[self.wp_index]
        dist = math.hypot(tx - self.local_pos[0], ty - self.local_pos[1])
        
        # Check if we're following an A* path
        is_astar = self.wp_index in self.astar_segments
        
        t_msg = String()
        t_msg.data = json.dumps({
            "current_position": list(self.local_pos),
            "target_waypoint": [tx, ty],
            "distance_to_target": float(dist),
            "astar_path": is_astar
        })
        self.pub_target.publish(t_msg)
        
        # Waypoint reached
        if dist < 4.0: 
            self.wp_index += 1
            if self.wp_index < len(self.waypoints):
                self.get_logger().info(f"✓ WP {self.wp_index-1} reached ({self.wp_index}/{len(self.waypoints)})")

    def check_and_replan(self):
        """Periodically check if current path is blocked and replan if needed"""
        if self.state != "DRIVING" or not self.waypoints or not self.obstacles:
            return
        
        if self.wp_index >= len(self.waypoints):
            return
        
        # Only check if not already on A* path (let A* paths be followed precisely)
        if self.wp_index in self.astar_segments:
            return
        
        # Check if direct path to next waypoint is blocked
        current = self.local_pos
        target = self.waypoints[self.wp_index]
        
        path_blocked = self._is_path_blocked(current, target)
        
        if path_blocked and self.obstacles_changed:
            self.get_logger().warn("Path blocked detected! Replanning...")
            self.replan_current_segment()
            self.obstacles_changed = False

    def _is_path_blocked(self, start, end):
        """Check if line segment from start to end intersects with any obstacle"""
        safe_dist = self.get_parameter('planner_safe_dist').value
        
        for ox, oy in self.obstacles:
            dist = self._point_to_segment_distance((ox, oy), start, end)
            if dist < safe_dist:
                return True
        return False

    def _point_to_segment_distance(self, point, seg_start, seg_end):
        """Calculate minimum distance from point to line segment"""
        px, py = point
        sx, sy = seg_start
        ex, ey = seg_end
        
        dx = ex - sx
        dy = ey - sy
        
        if dx == 0 and dy == 0:
            return math.hypot(px - sx, py - sy)
        
        # Parameter t represents position on segment (0 = start, 1 = end)
        t = max(0, min(1, ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)))
        
        # Closest point on segment
        proj_x = sx + t * dx
        proj_y = sy + t * dy
        
        return math.hypot(px - proj_x, py - proj_y)

    def replan_current_segment(self):
        """Replan from current position to next waypoint using A*"""
        if self.wp_index >= len(self.waypoints):
            return
        
        current = self.local_pos
        target = self.waypoints[self.wp_index]
        
        self.get_logger().info(f"🔍 A* planning from ({current[0]:.1f}, {current[1]:.1f}) to ({target[0]:.1f}, {target[1]:.1f})")
        
        # Get bounds
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        min_y = self.get_parameter('geo_min_y').value
        max_y = self.get_parameter('geo_max_y').value
        
        # Run A*
        new_path = self.astar.plan(
            current, 
            target, 
            self.obstacles,
            (min_x, min_y), 
            (max_x, max_y)
        )
        
        if new_path:
            # Insert A* path before current waypoint
            # Remove current waypoint and insert A* path + target
            remaining_waypoints = self.waypoints[self.wp_index+1:]
            
            # Mark these waypoints as A* segments
            self.astar_segments.clear()
            for i in range(len(new_path)):
                self.astar_segments.add(self.wp_index + i)
            
            self.waypoints = (
                self.waypoints[:self.wp_index] + 
                new_path + 
                [target] +
                remaining_waypoints
            )
            
            self.get_logger().info(f"A* path found! Added {len(new_path)} waypoints")
        else:
            self.get_logger().error("A* failed to find path! Continuing with original waypoint")

    def plan_initial_path(self):
        """Generate initial lawnmower pattern with A* for blocked segments"""
        self.waypoints = []
        self.astar_segments = set()
        
        len_ = self.get_parameter('scan_length').value
        wid = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        
        # Geofence
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        min_y = self.get_parameter('geo_min_y').value
        max_y = self.get_parameter('geo_max_y').value
        
        self.get_logger().info(f"Generating lawnmower: {lanes} lanes, {len_}m x {wid}m")
        
        for i in range(lanes):
            # Alternate direction
            sx, ex = (0.0, len_) if i % 2 == 0 else (len_, 0.0)
            y = i * wid
            
            # Clamp to geofence
            start = (max(min_x, min(max_x, sx)), max(min_y, min(max_y, y)))
            end = (max(min_x, min(max_x, ex)), max(min_y, min(max_y, y)))
            
            # Check if segment is blocked
            if self._is_path_blocked(start, end):
                self.get_logger().info(f"Lane {i} blocked - using A*")
                
                # Use A* for this segment
                path = self.astar.plan(start, end, self.obstacles, (min_x, min_y), (max_x, max_y))
                
                if path:
                    # Mark these as A* waypoints
                    start_idx = len(self.waypoints)
                    self.waypoints.extend(path)
                    for j in range(len(path)):
                        self.astar_segments.add(start_idx + j)
                    self.waypoints.append(end)
                    self.astar_segments.add(len(self.waypoints) - 1)
                else:
                    # Fallback: add endpoints anyway
                    self.get_logger().warn(f"A* failed for lane {i} - using direct path")
                    self.waypoints.append(start)
                    self.waypoints.append(end)
            else:
                # Direct path is clear
                self.waypoints.append(start)
                self.waypoints.append(end)
        
        self.get_logger().info(f"Path generated: {len(self.waypoints)} waypoints ({len(self.astar_segments)} A* segments)")


def main(args=None):
    rclpy.init(args=args)
    node = AtlantisPlanner()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()