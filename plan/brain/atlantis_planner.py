#!/usr/bin/env python3
"""
Atlantis Planner - Simple Lawnmower + A* Obstacle Avoidance

Simple logic:
1. Generate lawnmower pattern
2. Follow waypoints
3. When obstacle detected → A* replan around it
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
import json
import threading
import time
import math
import heapq


class AStarSolver:
    """Simple A* pathfinder"""
    
    def __init__(self, resolution=2.0, safety_margin=6.0):
        self.resolution = resolution
        self.safety_margin = safety_margin

    def world_to_grid(self, x, y, min_x, min_y):
        return int((x - min_x) / self.resolution), int((y - min_y) / self.resolution)

    def grid_to_world(self, gx, gy, min_x, min_y):
        return (gx * self.resolution) + min_x, (gy * self.resolution) + min_y

    def plan(self, start, goal, obstacles):
        """A* from start to goal avoiding obstacles"""
        if not obstacles:
            return [goal]  # Direct path if no obstacles
        
        padding = 30.0
        min_x = min(start[0], goal[0]) - padding
        max_x = max(start[0], goal[0]) + padding
        min_y = min(start[1], goal[1]) - padding
        max_y = max(start[1], goal[1]) + padding
        
        start_node = self.world_to_grid(start[0], start[1], min_x, min_y)
        goal_node = self.world_to_grid(goal[0], goal[1], min_x, min_y)
        
        # Build blocked cells from obstacles
        blocked = set()
        for ox, oy in obstacles:
            if min_x < ox < max_x and min_y < oy < max_y:
                ogx, ogy = self.world_to_grid(ox, oy, min_x, min_y)
                steps = int(self.safety_margin / self.resolution)
                for dx in range(-steps, steps + 1):
                    for dy in range(-steps, steps + 1):
                        if math.hypot(dx, dy) * self.resolution <= self.safety_margin:
                            blocked.add((ogx + dx, ogy + dy))
        
        # A* search
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
                path.append(goal)  # Ensure we end at exact goal
                return path
            
            # 8 directions
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if neighbor in blocked:
                    continue
                
                move_cost = math.hypot(dx, dy)
                tent_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tent_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tent_g
                    h = math.hypot(neighbor[0] - goal_node[0], neighbor[1] - goal_node[1])
                    heapq.heappush(open_set, (tent_g + h, neighbor))
        
        return [goal]  # Fallback: direct path


class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')
        
        # Parameters
        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 20.0)
        self.declare_parameter('lanes', 4)
        self.declare_parameter('waypoint_threshold', 4.0)
        self.declare_parameter('obstacle_trigger_distance', 15.0)  # Trigger A* when obstacle closer than this

        # Publishers
        self.pub_target = self.create_publisher(String, '/planning/current_target', 10)
        self.pub_status = self.create_publisher(String, '/planning/mission_status', 10)
        
        # Subscribers
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)
        self.create_subscription(String, '/perception/obstacle_info', self.obstacle_cb, 10)

        # State
        self.waypoints = []          # Original lawnmower waypoints
        self.current_path = []       # Current path (may include A* detour)
        self.path_index = 0
        self.wp_index = 0            # Which lawnmower waypoint we're heading to
        
        self.start_gps = None
        self.local_pos = (0.0, 0.0)
        self.yaw = 0.0
        
        # Obstacle state from OKO
        self.obstacles = []          # List of (x, y) in global coords
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        
        self.state = "IDLE"
        self.using_astar = False     # Currently following A* path?
        
        self.astar = AStarSolver(resolution=2.0, safety_margin=6.0)
        
        # Control loop
        self.create_timer(0.1, self.control_loop)
        
        # Input thread
        threading.Thread(target=self.input_loop, daemon=True).start()
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("ATLANTIS PLANNER")
        self.get_logger().info("Lawnmower + A* Obstacle Avoidance")
        self.get_logger().info("=" * 50)

    def input_loop(self):
        """Command interface"""
        time.sleep(1)
        print("\nCOMMANDS: 'r'=Run, 's'=Stop, 'p'=Status")
        while rclpy.ok():
            try:
                cmd = input("> ").strip().lower()
                if cmd == 'r':
                    self.generate_lawnmower()
                    self.state = "DRIVING"
                    self.get_logger().info(f"Mission started! {len(self.waypoints)} waypoints")
                elif cmd == 's':
                    self.state = "IDLE"
                    self.get_logger().info("Stopped")
                elif cmd == 'p':
                    self.print_status()
            except:
                pass

    def print_status(self):
        print(f"\n{'='*40}")
        print(f"State: {self.state}")
        print(f"Position: ({self.local_pos[0]:.1f}, {self.local_pos[1]:.1f})")
        print(f"Waypoint: {self.wp_index}/{len(self.waypoints)}")
        print(f"Using A*: {self.using_astar}")
        print(f"Obstacles: {len(self.obstacles)}")
        print(f"Obstacle distance: {self.obstacle_distance:.1f}m")
        if self.path_index < len(self.current_path):
            tx, ty = self.current_path[self.path_index]
            print(f"Next target: ({tx:.1f}, {ty:.1f})")
        print(f"{'='*40}\n")

    def gps_cb(self, msg):
        if not self.start_gps:
            self.start_gps = (msg.latitude, msg.longitude)
        R = 6371000.0
        dx = math.radians(msg.latitude - self.start_gps[0]) * R
        dy = math.radians(msg.longitude - self.start_gps[1]) * R * math.cos(math.radians(self.start_gps[0]))
        self.local_pos = (dx, dy)

    def imu_cb(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def obstacle_cb(self, msg):
        """Receive obstacle info from OKO perception"""
        try:
            data = json.loads(msg.data)
            self.obstacle_detected = data.get('obstacle_detected', False)
            self.obstacle_distance = data.get('min_distance', float('inf'))
            
            # Convert clusters to global coordinates
            new_obstacles = []
            cx, cy = math.cos(self.yaw), math.sin(self.yaw)
            
            for cluster in data.get('clusters', []):
                # Transform from sensor frame to global
                lx, ly = cluster['x'], cluster['y']
                gx = self.local_pos[0] + (lx * cx - ly * cy)
                gy = self.local_pos[1] + (lx * cy + ly * cx)
                new_obstacles.append((gx, gy))
            
            self.obstacles = new_obstacles
            
        except Exception as e:
            self.get_logger().warn(f"Obstacle parse error: {e}")

    def generate_lawnmower(self):
        """Generate simple lawnmower pattern"""
        self.waypoints = []
        
        length = self.get_parameter('scan_length').value
        width = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        
        for i in range(lanes):
            y = i * width
            if i % 2 == 0:
                self.waypoints.append((0.0, y))
                self.waypoints.append((length, y))
            else:
                self.waypoints.append((length, y))
                self.waypoints.append((0.0, y))
        
        self.wp_index = 0
        self.path_index = 0
        self.current_path = [self.waypoints[0]] if self.waypoints else []
        self.using_astar = False
        
        self.get_logger().info(f"Lawnmower: {lanes} lanes, {length}m x {width}m")

    def control_loop(self):
        """Main loop"""
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({"state": self.state})
        self.pub_status.publish(status_msg)
        
        if self.state != "DRIVING" or not self.waypoints:
            return
        
        # Check mission complete
        if self.wp_index >= len(self.waypoints):
            self.state = "FINISHED"
            self.get_logger().info("🎉 Mission Complete!")
            return
        
        # Current target waypoint
        target_wp = self.waypoints[self.wp_index]
        
        # Check if we need A* (obstacle in the way)
        trigger_dist = self.get_parameter('obstacle_trigger_distance').value
        
        if self.obstacle_detected and self.obstacle_distance < trigger_dist and not self.using_astar:
            # Obstacle detected! Plan A* path around it
            self.get_logger().warn(f"🚨 Obstacle at {self.obstacle_distance:.1f}m - Planning A* detour")
            
            astar_path = self.astar.plan(self.local_pos, target_wp, self.obstacles)
            
            if astar_path:
                self.current_path = astar_path
                self.path_index = 0
                self.using_astar = True
                self.get_logger().info(f"A* path: {len(astar_path)} waypoints")
            else:
                self.get_logger().warn("A* failed - continuing direct")
        
        # Get current target from path
        if self.path_index >= len(self.current_path):
            # Finished current path, move to next waypoint
            self.wp_index += 1
            if self.wp_index < len(self.waypoints):
                self.current_path = [self.waypoints[self.wp_index]]
                self.path_index = 0
                self.using_astar = False
                self.get_logger().info(f"✓ WP {self.wp_index-1} reached → WP {self.wp_index}")
            return
        
        current_target = self.current_path[self.path_index]
        dist = math.hypot(current_target[0] - self.local_pos[0], 
                          current_target[1] - self.local_pos[1])
        
        # Publish target for controller
        target_msg = String()
        target_msg.data = json.dumps({
            "current_position": list(self.local_pos),
            "target_waypoint": list(current_target),
            "distance_to_target": dist,
            "using_astar": self.using_astar,
            "wp_index": self.wp_index,
            "total_waypoints": len(self.waypoints)
        })
        self.pub_target.publish(target_msg)
        
        # Check if reached current path point
        threshold = self.get_parameter('waypoint_threshold').value
        if dist < threshold:
            self.path_index += 1
            if self.using_astar and self.path_index >= len(self.current_path):
                self.get_logger().info("A* detour complete")
                self.using_astar = False


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