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
    def __init__(self, resolution=3.0, safety_margin=10.0):
        self.resolution = resolution
        self.safety_margin = safety_margin

    def world_to_grid(self, x, y, min_x, min_y):
        return int((x - min_x)/self.resolution), int((y - min_y)/self.resolution)

    def grid_to_world(self, gx, gy, min_x, min_y):
        return (gx*self.resolution)+min_x, (gy*self.resolution)+min_y

    def plan(self, start, goal, obstacles, b_min, b_max):
        padding = 40.0
        min_x, max_x = min(start[0], goal[0])-padding, max(start[0], goal[0])+padding
        min_y, max_y = min(start[1], goal[1])-padding, max(start[1], goal[1])+padding
        
        start_node = self.world_to_grid(start[0], start[1], min_x, min_y)
        goal_node = self.world_to_grid(goal[0], goal[1], min_x, min_y)
        
        blocked = set()
        for ox, oy in obstacles:
            if min_x < ox < max_x and min_y < oy < max_y:
                ogx, ogy = self.world_to_grid(ox, oy, min_x, min_y)
                steps = int(self.safety_margin/self.resolution)
                for dx in range(-steps, steps+1):
                    for dy in range(-steps, steps+1):
                        blocked.add((ogx+dx, ogy+dy))

        open_set = [(0, start_node)]
        came_from = {}
        g_score = {start_node: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_node:
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(current[0], current[1], min_x, min_y))
                    current = came_from[current]
                return path[::-1]

            for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0]+dx, current[1]+dy)
                if neighbor in blocked: continue
                
                tent_g = g_score[current] + math.hypot(dx, dy)
                if neighbor not in g_score or tent_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tent_g
                    f = tent_g + math.hypot(neighbor[0]-goal_node[0], neighbor[1]-goal_node[1])
                    heapq.heappush(open_set, (f, neighbor))
        return []

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

        # Communication
        self.pub_target = self.create_publisher(String, '/planning/current_target', 10)
        self.pub_status = self.create_publisher(String, '/planning/mission_status', 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)
        self.create_subscription(String, '/perception/obstacle_info', self.obs_cb, 10)
        self.create_subscription(String, '/planning/detour_request', self.detour_cb, 10)
        
        self.waypoints = []
        self.wp_index = 0
        self.start_gps = None
        self.local_pos = (0.0, 0.0)
        self.yaw = 0.0
        self.obstacles = []
        self.state = "IDLE"
        
        self.astar = AStarSolver()
        self.create_timer(0.1, self.loop)
        threading.Thread(target=self.input_loop, daemon=True).start()
        self.get_logger().info("Atlantis Planner (Modular) Ready")

    def input_loop(self):
        time.sleep(1)
        print("COMMANDS: 'r' = Run, 's' = Stop")
        while rclpy.ok():
            try:
                cmd = input("> ").strip()
                if cmd == 'r': 
                    self.plan_path()
                    self.state = "DRIVING"
                    self.wp_index = 0
                elif cmd == 's': self.state = "IDLE"
            except: pass

    def gps_cb(self, msg):
        if not self.start_gps: self.start_gps = (msg.latitude, msg.longitude)
        R = 6371000.0
        dx = math.radians(msg.latitude - self.start_gps[0]) * R
        dy = math.radians(msg.longitude - self.start_gps[1]) * R * math.cos(math.radians(self.start_gps[0]))
        self.local_pos = (dx, dy)

    def imu_cb(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def obs_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacles = []
            cx, cy = math.cos(self.yaw), math.sin(self.yaw)
            for c in data.get('clusters', []):
                # Rotate local to global
                gx = self.local_pos[0] + (c['x']*cx - c['y']*cy)
                gy = self.local_pos[1] + (c['x']*cy + c['y']*cx)
                self.obstacles.append((gx, gy))
        except: pass

    def detour_cb(self, msg):
        try:
            d = json.loads(msg.data)
            if d.get('type') == 'detour':
                self.waypoints.insert(self.wp_index, (d['x'], d['y']))
                self.get_logger().info("Detour Inserted")
        except: pass

    def loop(self):
        msg = String()
        msg.data = json.dumps({"state": self.state})
        self.pub_status.publish(msg)
        
        if self.state != "DRIVING" or not self.waypoints: return
        if self.wp_index >= len(self.waypoints):
            self.state = "FINISHED"
            return

        tx, ty = self.waypoints[self.wp_index]
        dist = math.hypot(tx - self.local_pos[0], ty - self.local_pos[1])
        
        t_msg = String()
        t_msg.data = json.dumps({
            "current_position": self.local_pos,
            "target_waypoint": [tx, ty],
            "distance_to_target": dist
        })
        self.pub_target.publish(t_msg)
        
        if dist < 4.0: 
            self.wp_index += 1
            self.get_logger().info(f"WP {self.wp_index} Reached")

    def plan_path(self):
        # Lawnmower Gen + A* Hybrid
        self.waypoints = []
        len_ = self.get_parameter('scan_length').value
        wid = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        
        # Simple Geofence
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        
        for i in range(lanes):
            sx, ex = (0.0, len_) if i%2==0 else (len_, 0.0)
            y = i * wid
            
            # Check line safety
            start, end = (max(min_x, sx), y), (min(max_x, ex), y)
            
            # Simple check: Is line clear?
            blocked = False
            for ox, oy in self.obstacles:
                # Distance from point to line segment logic (simplified check)
                if min(start[0], end[0]) < ox < max(start[0], end[0]) and abs(oy - y) < 10.0:
                    blocked = True
                    break
            
            if not blocked:
                self.waypoints.append(start)
                self.waypoints.append(end)
            else:
                self.get_logger().info("Using A* for segment")
                path = self.astar.plan(start, end, self.obstacles, (-50,-100), (200,100))
                if path: self.waypoints.extend(path)
                else: self.waypoints.append(end) # Fallback

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