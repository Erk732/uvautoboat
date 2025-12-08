import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import SetParametersResult
import json
import threading
import sys
import time
import math
import struct
import cv2
import numpy as np
import heapq
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Related import
from .lidar_obstacle_avoidance import (
    LidarObstacleDetector,
    ObstacleClustering,
    ObstacleAvoider,
    RealtimeObstacleMonitor
)

# --- A* SOLVER CLASS ---
class AStarSolver:
    def __init__(self, resolution=2.0, safety_margin=5.0):
        self.resolution = resolution  # Grid size in meters
        self.safety_margin = safety_margin

    def world_to_grid(self, x, y, min_x, min_y):
        gx = int((x - min_x) / self.resolution)
        gy = int((y - min_y) / self.resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy, min_x, min_y):
        wx = (gx * self.resolution) + min_x + (self.resolution / 2.0)
        wy = (gy * self.resolution) + min_y + (self.resolution / 2.0)
        return (wx, wy)

    def get_neighbors(self, node, grid_width, grid_height):
        (x, y) = node
        # 8-connected grid (diagonals allowed)
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0), 
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid_width and 0 <= ny < grid_height:
                dist = 1.414 if dx != 0 and dy != 0 else 1.0 # Diagonal cost
                neighbors.append(((nx, ny), dist))
        return neighbors

    def plan(self, start, goal, obstacles, boundary_min, boundary_max):
        # 1. Define Grid Boundaries (Local Search Area)
        # We add padding around the start/goal to give A* room to maneuver
        padding = 30.0 
        min_x = min(start[0], goal[0]) - padding
        max_x = max(start[0], goal[0]) + padding
        min_y = min(start[1], goal[1]) - padding
        max_y = max(start[1], goal[1]) + padding
        
        # Enforce Geofence limits if provided
        if boundary_min and boundary_max:
            min_x = max(min_x, boundary_min[0])
            max_x = min(max_x, boundary_max[0])
            min_y = max(min_y, boundary_min[1])
            max_y = min(max_y, boundary_max[1])

        grid_width = int((max_x - min_x) / self.resolution) + 1
        grid_height = int((max_y - min_y) / self.resolution) + 1
        
        start_node = self.world_to_grid(start[0], start[1], min_x, min_y)
        goal_node = self.world_to_grid(goal[0], goal[1], min_x, min_y)
        
        # 2. Map Obstacles to Grid
        blocked_nodes = set()
        
        # Optimization: Only consider relevant obstacles
        for ox, oy in obstacles:
            if min_x - 10 < ox < max_x + 10 and min_y - 10 < oy < max_y + 10:
                ogx, ogy = self.world_to_grid(ox, oy, min_x, min_y)
                # Mark radius around obstacle as blocked
                radius_steps = int(self.safety_margin / self.resolution)
                for dx in range(-radius_steps, radius_steps + 1):
                    for dy in range(-radius_steps, radius_steps + 1):
                        if dx*dx + dy*dy <= radius_steps*radius_steps:
                            blocked_nodes.add((ogx + dx, ogy + dy))

        # 3. A* Algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_node:
                # Reconstruct Path
                path = []
                while current in came_from:
                    wx, wy = self.grid_to_world(current[0], current[1], min_x, min_y)
                    path.append((wx, wy))
                    current = came_from[current]
                path.reverse()
                return path # Returns list of (x,y) waypoints

            for neighbor, dist_cost in self.get_neighbors(current, grid_width, grid_height):
                if neighbor in blocked_nodes: continue
                
                tentative_g = g_score[current] + dist_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    # Heuristic: Euclidean distance
                    h = ((neighbor[0] - goal_node[0])**2 + (neighbor[1] - goal_node[1])**2)**0.5
                    f_score = tentative_g + h
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return [] # No path found

# --- MAIN PLANNER NODE ---
class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')

        # --- PARAMETERS ---
        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 20.0) 
        self.declare_parameter('lanes', 4)
        self.declare_parameter('frame_id', 'map') 
        self.declare_parameter('geo_min_x', -50.0)
        self.declare_parameter('geo_max_x', 200.0)
        self.declare_parameter('geo_min_y', -100.0)
        self.declare_parameter('geo_max_y', 100.0)
        self.declare_parameter('waypoint_spacing', 10.0)
        
        # --- OBSTACLE AVOIDANCE PARAMETERS ---
        self.declare_parameter('planner_safe_dist', 10.0)
        self.declare_parameter('obstacle_lookahead', 15.0)
        self.declare_parameter('obstacle_cluster_radius', 2.0)
        self.declare_parameter('obstacle_min_distance', 5.0)
        self.declare_parameter('obstacle_max_distance', 50.0)
        self.declare_parameter('lidar_sampling_factor', 50)

        # Publishers & Subscribers
        self.path_pub = self.create_publisher(Path, '/atlantis/path', 10)
        self.pub_waypoints = self.create_publisher(String, '/atlantis/waypoints', 10)
        self.pub_config = self.create_publisher(String, '/atlantis/config', 10)
        self.pub_obstacle_map = self.create_publisher(String, '/atlantis/obstacle_map', 10)
        self.create_subscription(Empty, '/atlantis/replan', self.replan_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timers
        self.create_timer(1.0, self.publish_path)
        self.create_timer(1.0, self.publish_config)
        self.create_timer(0.5, self.publish_obstacle_map)
        self.create_timer(2.0, self.publish_lidar_statistics)

        # State
        self.path_msg = Path()
        self.waypoints = []
        self.shutdown_flag = False
        self.known_obstacles = []
        self.path_lock = threading.RLock()
        self.path_version = 0
        self.mission_enabled = False
        
        # Stats
        self.lidar_signal_count = 0
        self.lidar_obstacle_detections = 0
        self.lidar_clear_detections = 0
        self.lidar_detection_rate = 0.0
        
        # --- OBSTACLE AVOIDANCE MODULES ---
        self.lidar_detector = LidarObstacleDetector(
            min_distance=self.get_parameter('obstacle_min_distance').value,
            max_distance=self.get_parameter('obstacle_max_distance').value
        )
        self.clusterer = ObstacleClustering(
            cluster_radius=self.get_parameter('obstacle_cluster_radius').value
        )
        self.avoider = ObstacleAvoider(
            safe_distance=self.get_parameter('planner_safe_dist').value,
            look_ahead=self.get_parameter('obstacle_lookahead').value
        )
        self.monitor = RealtimeObstacleMonitor()
        
        # --- INIT A* SOLVER ---
        self.astar = AStarSolver(resolution=3.0, safety_margin=10.0)
        
        self.current_obstacles = []
        self.current_clusters = []
        #--- SMOKE DETECTION SETUP ---
        self.bridge = CvBridge()
        self.smoke_detected = False
        self.smoke_regions = []

         # Subscribe to front cameras
        self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
            self.camera_callback,
            10
        )
        self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_right_camera_sensor/image_raw',
            self.camera_callback,
            10
        )

        self.get_logger().info("Atlantis Planner Initialized")
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("Atlantis Planner Started (Waiting for 'r' command)")

    def input_loop(self):
        time.sleep(1.0)
        print("\n" + "="*40)
        print("  ATLANTIS MISSION CONTROL")
        print("  Type 'r' + ENTER -> Regenerate Lawnmower Path")
        print("  Type 'b' + ENTER -> Return to Home Base")
        print("="*40 + "\n")
        while rclpy.ok() and not self.shutdown_flag:
            try:
                user_input = input("Command (r/b) > ").strip().lower()
                if user_input == 'b': self.generate_return_home_path()
                elif user_input == 'r': self.generate_lawnmower_path()
            except: pass

    def lidar_callback(self, msg):
        try:
            self.lidar_signal_count += 1
            sampling_factor = self.get_parameter('lidar_sampling_factor').value
            
            # Extract raw pointcloud
            self.current_obstacles = self.lidar_detector.process_pointcloud(
                msg.data, msg.point_step, sampling_factor
            )

            # Filter Smoke/Noise
            filtered = []
            for obs in self.current_obstacles:
                if hasattr(obs, "intensity") and obs.intensity < 5.0: continue
                if abs(obs.z) > 0.5: continue
                dist = math.sqrt(obs.x**2 + obs.y**2)
                if dist < 2.0: continue
                filtered.append(obs)

            self.current_obstacles = filtered

            if len(self.current_obstacles) > 0:
                self.lidar_obstacle_detections += 1
            else:
                self.lidar_clear_detections += 1

            if self.lidar_signal_count > 0:
                self.lidar_detection_rate = (
                    self.lidar_obstacle_detections / self.lidar_signal_count * 100.0
                )

            self.current_clusters = self.clusterer.cluster_obstacles(self.current_obstacles)
            self.monitor.analyze_sectors(self.current_obstacles)
            self.known_obstacles = [(obs.x, obs.y) for obs in self.current_obstacles]
                    
        except Exception as e:
            self.get_logger().error(f"LIDAR callback error: {e}")

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
            white_pixel_count = cv2.countNonZero(mask)
            total_pixels = gray.shape[0] * gray.shape[1]
            smoke_ratio = white_pixel_count / total_pixels
            self.smoke_detected = smoke_ratio > 0.02
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

    def is_point_safe(self, x, y):
        safe_dist = self.get_parameter('planner_safe_dist').value
        for obs_x, obs_y in self.known_obstacles:
            dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < safe_dist:
                return False, obs_x, obs_y
        return True, None, None

    def is_line_safe(self, x1, y1, x2, y2):
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        steps = int(dist / 2.0)
        
        if steps == 0: return True

        for i in range(steps + 1):
            t = i / steps
            check_x = x1 + (x2 - x1) * t
            check_y = y1 + (y2 - y1) * t
            
            is_safe, _, _ = self.is_point_safe(check_x, check_y)
            if not is_safe:
                return False
        return True

    def adjust_point_for_obstacles(self, x, y, target_x=None, target_y=None):
        is_safe, obs_x, obs_y = self.is_point_safe(x, y)
        if not is_safe:
            safe_dist = self.get_parameter('planner_safe_dist').value
            if target_x is not None and target_y is not None:
                dx = target_x - obs_x
                dy = target_y - obs_y
                dist = math.sqrt(dx**2 + dy**2)
                if dist > 0.1:
                    shift_x = (dx / dist) * (safe_dist + 2.0)
                    shift_y = (dy / dist) * (safe_dist + 2.0)
                    return obs_x + shift_x, obs_y + shift_y
            return x, y + safe_dist + 2.0
        
        if self.smoke_detected: y += 1.0
        return x, y

    def interpolate_segment(self, start_x, start_y, end_x, end_y):
        spacing = self.get_parameter('waypoint_spacing').value
        dist = math.hypot(end_x - start_x, end_y - start_y)
        if dist <= spacing: return [(end_x, end_y)]
        points = []
        num_points = int(math.ceil(dist / spacing))
        for i in range(1, num_points + 1):
            t = i / float(num_points)
            px = start_x + (end_x - start_x) * t
            py = start_y + (end_y - start_y) * t
            points.append((px, py))
        return points

    def parameter_callback(self, params):
        if self.mission_enabled:
            return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def replan_callback(self, msg):
        self.generate_lawnmower_path()

    def apply_geofence(self, x, y):
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        min_y = self.get_parameter('geo_min_y').value
        max_y = self.get_parameter('geo_max_y').value
        return max(min_x, min(x, max_x)), max(min_y, min(y, max_y))

    def generate_return_home_path(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.get_parameter('frame_id').value
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints = []
        
        pose = PoseStamped()
        pose.header = self.path_msg.header
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        self.path_msg.poses.append(pose)
        self.waypoints.append((0.0, 0.0))
        self.publish_path()

    # --- HYBRID PATH GENERATOR (A* + Straight Line) ---
    def generate_lawnmower_path(self):
        scan_length = self.get_parameter('scan_length').value
        scan_width = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.get_parameter('frame_id').value
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints = [] 

        geo_bounds_min = (self.get_parameter('geo_min_x').value, self.get_parameter('geo_min_y').value)
        geo_bounds_max = (self.get_parameter('geo_max_x').value, self.get_parameter('geo_max_y').value)

        for i in range(lanes):
            if i % 2 == 0: x_start, x_end = 0.0, scan_length
            else: x_start, x_end = scan_length, 0.0
            y_pos = i * scan_width

            safe_start_x, safe_start_y = self.apply_geofence(x_start, y_pos)
            safe_end_x, safe_end_y = self.apply_geofence(x_end, y_pos)
            
            # Hybrid Logic
            # 1. Try Straight Line
            if self.is_line_safe(safe_start_x, safe_start_y, safe_end_x, safe_end_y):
                self.get_logger().info(f"Lane {i}: Clear path - Using Straight Line")
                points = self.interpolate_segment(safe_start_x, safe_start_y, safe_end_x, safe_end_y)
                if i == 0: self.add_waypoint(safe_start_x, safe_start_y)
                for px, py in points: self.add_waypoint(px, py)
            else:
                # 2. Path Blocked -> Use A*
                self.get_logger().warn(f"Lane {i}: BLOCKED - Calculating A* Detour...")
                obstacles = [(obs_x, obs_y) for obs_x, obs_y in self.known_obstacles]
                
                astar_path = self.astar.plan(
                    start=(safe_start_x, safe_start_y),
                    goal=(safe_end_x, safe_end_y),
                    obstacles=obstacles,
                    boundary_min=geo_bounds_min,
                    boundary_max=geo_bounds_max
                )
                
                if astar_path:
                    self.get_logger().info(f"Lane {i}: A* found path with {len(astar_path)} nodes")
                    if i == 0: self.add_waypoint(safe_start_x, safe_start_y)
                    for px, py in astar_path: self.add_waypoint(px, py)
                else:
                    self.get_logger().error(f"Lane {i}: A* FAILED. Skipping lane.")
                    continue

            if i < lanes - 1:
                next_y = (i + 1) * scan_width
                safe_next_x, safe_next_y = self.apply_geofence(x_end, next_y)
                self.add_waypoint(safe_next_x, safe_next_y)

        self.get_logger().info(f"GENERATED: Mission Path ({len(self.waypoints)} points)")
        self.publish_path()

    def add_waypoint(self, x, y):
        pose = PoseStamped()
        pose.header = self.path_msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        self.path_msg.poses.append(pose)
        self.waypoints.append((x, y))

    def publish_lidar_statistics(self):
        pass

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
        waypoint_data = {'waypoints': [{'x': wp[0], 'y': wp[1]} for wp in self.waypoints]}
        msg = String()
        msg.data = json.dumps(waypoint_data)
        self.pub_waypoints.publish(msg)

    def publish_config(self):
        config = {'scan_length': self.get_parameter('scan_length').value, 'lanes': self.get_parameter('lanes').value}
        msg = String()
        msg.data = json.dumps(config)
        self.pub_config.publish(msg)
    
    def publish_obstacle_map(self):
        obstacle_data = {'count': len(self.current_obstacles)}
        msg = String()
        msg.data = json.dumps(obstacle_data)
        self.pub_obstacle_map.publish(msg)
        
    def stop_all(self):
        self.shutdown_flag = True
        self.path_pub.publish(Path())

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisPlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()