import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix, Imu 
from rcl_interfaces.msg import SetParametersResult
import json
import threading
import sys
import time
import math
import heapq

# AStarSolver for local path planning around obstacles
class AStarSolver:
    def __init__(self, resolution=2.0, safety_margin=5.0):
        self.resolution = resolution
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
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid_width and 0 <= ny < grid_height:
                dist = 1.414 if dx != 0 and dy != 0 else 1.0
                neighbors.append(((nx, ny), dist))
        return neighbors

    def plan(self, start, goal, obstacles, boundary_min, boundary_max):
        padding = 30.0 
        min_x = min(start[0], goal[0]) - padding
        max_x = max(start[0], goal[0]) + padding
        min_y = min(start[1], goal[1]) - padding
        max_y = max(start[1], goal[1]) + padding
        
        if boundary_min and boundary_max:
            min_x = max(min_x, boundary_min[0])
            max_x = min(max_x, boundary_max[0])
            min_y = max(min_y, boundary_min[1])
            max_y = min(max_y, boundary_max[1])

        grid_width = int((max_x - min_x) / self.resolution) + 1
        grid_height = int((max_y - min_y) / self.resolution) + 1
        
        start_node = self.world_to_grid(start[0], start[1], min_x, min_y)
        goal_node = self.world_to_grid(goal[0], goal[1], min_x, min_y)
        
        blocked_nodes = set()
        for ox, oy in obstacles:
            if min_x - 10 < ox < max_x + 10 and min_y - 10 < oy < max_y + 10:
                ogx, ogy = self.world_to_grid(ox, oy, min_x, min_y)
                radius_steps = int(self.safety_margin / self.resolution)
                for dx in range(-radius_steps, radius_steps + 1):
                    for dy in range(-radius_steps, radius_steps + 1):
                        if dx*dx + dy*dy <= radius_steps*radius_steps:
                            blocked_nodes.add((ogx + dx, ogy + dy))

        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_node:
                path = []
                while current in came_from:
                    wx, wy = self.grid_to_world(current[0], current[1], min_x, min_y)
                    path.append((wx, wy))
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor, dist_cost in self.get_neighbors(current, grid_width, grid_height):
                if neighbor in blocked_nodes: continue
                tentative_g = g_score[current] + dist_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = ((neighbor[0] - goal_node[0])**2 + (neighbor[1] - goal_node[1])**2)**0.5
                    f_score = tentative_g + h
                    heapq.heappush(open_set, (f_score, neighbor))
        return []


# ATLANTIS PLANNER (MAPPING PROVBLEM FIXED)

class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')

        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 20.0) 
        self.declare_parameter('lanes', 4)
        self.declare_parameter('geo_min_x', -50.0)
        self.declare_parameter('geo_max_x', 200.0)
        self.declare_parameter('geo_min_y', -100.0)
        self.declare_parameter('geo_max_y', 100.0)
        self.declare_parameter('waypoint_spacing', 10.0)
        self.declare_parameter('waypoint_tolerance', 4.0)
        self.declare_parameter('planner_safe_dist', 10.0)

        self.pub_current_target = self.create_publisher(String, '/planning/current_target', 10)
        self.pub_mission_status = self.create_publisher(String, '/planning/mission_status', 10)
        self.path_pub = self.create_publisher(Path, '/atlantis/path', 10)
        
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        # NEW: Need IMU for rotation
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.create_subscription(String, '/perception/obstacle_info', self.obstacle_callback, 10)
        self.create_subscription(String, '/planning/detour_request', self.detour_callback, 10)
        self.create_subscription(Empty, '/atlantis/replan', self.replan_callback, 10)
        
        self.waypoints = [] 
        self.current_wp_index = 0
        self.start_gps = None
        self.current_gps = None
        self.current_local_pos = (0.0, 0.0)
        self.current_yaw = 0.0  # NEW: Track heading
        self.mission_state = "IDLE" 
        self.known_obstacles = [] 
        
        self.astar = AStarSolver(resolution=3.0, safety_margin=12.0)
        
        self.create_timer(0.1, self.mission_manager_loop)
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("Atlantis Planner (Global Mapping Fixed) Ready.")

    def input_loop(self):
        time.sleep(1.0)
        print("\n" + "="*40)
        print("  ATLANTIS MISSION MANAGER")
        print("  Type 'r' + ENTER -> START Mission")
        print("  Type 's' + ENTER -> STOP Mission")
        print("="*40 + "\n")
        while rclpy.ok():
            try:
                user_input = input("Command (r/s) > ").strip().lower()
                if user_input == 'r': 
                    self.generate_lawnmower_path()
                    self.mission_state = "DRIVING"
                    self.current_wp_index = 0
                    self.get_logger().info("Mission STARTED")
                elif user_input == 's':
                    self.mission_state = "IDLE"
                    self.get_logger().info("Mission STOPPED.")
            except: pass

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
        y, x = self.latlon_to_meters(msg.latitude, msg.longitude)
        self.current_local_pos = (x, y)

    def latlon_to_meters(self, lat, lon):
        if self.start_gps is None: return 0.0, 0.0
        R = 6371000.0
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])
        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return x, y 

    # IMU Callback for Heading ---
    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # Transform Local Obstacles to Global Map ---
    def obstacle_callback(self, msg):
        """Parse OKO Perception JSON and transform to GLOBAL MAP coordinates"""
        try:
            data = json.loads(msg.data)
            if 'clusters' in data:
                self.known_obstacles = []
                
                boat_x, boat_y = self.current_local_pos
                yaw = self.current_yaw
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)

                for c in data['clusters']:
                    # 1. Get Local Coordinates (Relative to Boat)
                    local_x = c['x']
                    local_y = c['y']
                    
                    # 2. Rotate to align with Map Frame
                    rotated_x = local_x * cos_yaw - local_y * sin_yaw
                    rotated_y = local_x * sin_yaw + local_y * cos_yaw
                    
                    # 3. Translate to Global Position
                    global_x = boat_x + rotated_x
                    global_y = boat_y + rotated_y
                    
                    self.known_obstacles.append((global_x, global_y))
        except: pass

    def detour_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('type') == 'detour':
                dx, dy = data['x'], data['y']
                self.get_logger().warn(f"↩ Inserting DETOUR at ({dx:.1f}, {dy:.1f})")
                self.waypoints.insert(self.current_wp_index, (dx, dy))
        except: pass

    def mission_manager_loop(self):
        status_msg = String()
        status_msg.data = json.dumps({
            "state": self.mission_state,
            "current_waypoint": self.current_wp_index + 1,
            "total_waypoints": len(self.waypoints)
        })
        self.pub_mission_status.publish(status_msg)

        if self.mission_state != "DRIVING" or not self.waypoints: return
        if self.current_wp_index >= len(self.waypoints):
            self.mission_state = "FINISHED"
            self.get_logger().info("MISSION COMPLETE")
            return

        target_x, target_y = self.waypoints[self.current_wp_index]
        curr_x, curr_y = self.current_local_pos
        dist = math.hypot(target_x - curr_x, target_y - curr_y)
        
        target_msg = String()
        target_msg.data = json.dumps({
            "current_position": [curr_x, curr_y],
            "target_waypoint": [target_x, target_y],
            "distance_to_target": dist,
            "waypoint_index": self.current_wp_index
        })
        self.pub_current_target.publish(target_msg)

        tolerance = self.get_parameter('waypoint_tolerance').value
        if dist < tolerance:
            self.get_logger().info(f"Reached WP {self.current_wp_index+1} -> Next")
            self.current_wp_index += 1

    def generate_lawnmower_path(self):
        scan_length = self.get_parameter('scan_length').value
        scan_width = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        self.waypoints = [] 
        
        viz_msg = Path()
        viz_msg.header.frame_id = "map"
        
        geo_min = (self.get_parameter('geo_min_x').value, self.get_parameter('geo_min_y').value)
        geo_max = (self.get_parameter('geo_max_x').value, self.get_parameter('geo_max_y').value)

        for i in range(lanes):
            if i % 2 == 0: x_start, x_end = 0.0, scan_length
            else: x_start, x_end = scan_length, 0.0
            y_pos = i * scan_width

            start = self.apply_geofence(x_start, y_pos)
            end = self.apply_geofence(x_end, y_pos)
            
            if self.is_line_safe(start[0], start[1], end[0], end[1]):
                points = self.interpolate_segment(start[0], start[1], end[0], end[1])
                if i == 0: self._add_wp(start[0], start[1], viz_msg)
                for px, py in points: self._add_wp(px, py, viz_msg)
            else:
                self.get_logger().warn(f"Lane {i} BLOCKED - Running A*...")
                path = self.astar.plan(start, end, self.known_obstacles, geo_min, geo_max)
                if path:
                    if i == 0: self._add_wp(start[0], start[1], viz_msg)
                    for px, py in path: self._add_wp(px, py, viz_msg)
                else:
                    self.get_logger().error(f"Lane {i} Unreachable - Skipping")

            if i < lanes - 1:
                next_y = (i + 1) * scan_width
                trans = self.apply_geofence(x_end, next_y)
                self._add_wp(trans[0], trans[1], viz_msg)

        self.path_pub.publish(viz_msg)
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints (Hybrid A*).")

    def _add_wp(self, x, y, viz_path):
        self.waypoints.append((x, y))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        viz_path.poses.append(pose)

    def apply_geofence(self, x, y):
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        min_y = self.get_parameter('geo_min_y').value
        max_y = self.get_parameter('geo_max_y').value
        return max(min_x, min(x, max_x)), max(min_y, min(y, max_y))

    def is_line_safe(self, x1, y1, x2, y2):
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        steps = int(dist / 2.0)
        if steps == 0: return True
        safe_dist = self.get_parameter('planner_safe_dist').value
        
        for i in range(steps + 1):
            t = i / max(1, steps)
            cx = x1 + (x2 - x1) * t
            cy = y1 + (y2 - y1) * t
            for ox, oy in self.known_obstacles:
                if math.hypot(cx - ox, cy - oy) < safe_dist:
                    return False
        return True

    def interpolate_segment(self, start_x, start_y, end_x, end_y):
        spacing = self.get_parameter('waypoint_spacing').value
        dist = math.hypot(end_x - start_x, end_y - start_y)
        if dist <= spacing: return [(end_x, end_y)]
        points = []
        num = int(math.ceil(dist / spacing))
        for i in range(1, num + 1):
            t = i / float(num)
            points.append((start_x + (end_x - start_x)*t, start_y + (end_y - start_y)*t))
        return points

    def replan_callback(self, msg):
        self.generate_lawnmower_path()

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisPlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()