import rclpy
from rclpy.node import Node
import math
import heapq # Priority Queue for A*

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point

# Import the GridMap class
from path.grid_map import GridMap

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner_node')

        # --- Interfaces ---
        self.sub_odom = self.create_subscription(Odometry, '/wamv/sensors/odometry', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        self.pub_path = self.create_publisher(Path, '/planning/path', 10)

        # --- Internal State ---
        self.current_pose = None
        self.goal_pose = None
        
        # --- The Map ---
        # Create a 200x200 meter grid (large enough for our test)
        self.grid = GridMap(width_m=200, height_m=200, resolution=1.0)
        
        # --- ADD VIRTUAL OBSTACLES ---
        # "Virtual Island" at (25, 10) with radius 5m
        self.get_logger().info("Adding Virtual Obstacles...")
        self.grid.set_obstacle(x=25.0, y=10.0, radius=5.0)

        self.get_logger().info('A* Planner Ready. Waiting for Odom and Goal...')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        if self.current_pose:
            self.plan_path()
        else:
            self.get_logger().warn('Waiting for Odometry...')

    def plan_path(self):
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        # 1. Convert Start/Goal to Grid Indices
        start_idx = self.grid.world_to_grid(start_x, start_y)
        goal_idx = self.grid.world_to_grid(goal_x, goal_y)
        
        if not start_idx or not goal_idx:
            self.get_logger().warn("Start or Goal is outside map boundaries!")
            return

        # 2. Run A* Algorithm
        path_indices = self.run_astar(start_idx, goal_idx)
        
        if not path_indices:
            self.get_logger().error("No path found! Obstacle might be blocking completely.")
            return

        # 3. Convert Indices to World Points (Meters)
        path_points = []
        for r, c in path_indices:
            wx, wy = self.grid.grid_to_world(r, c)
            path_points.append((wx, wy))

        # 4. SMOOTH THE PATH (New Step)
        self.get_logger().info("Smoothing path...")
        smoothed_points = self.smooth_path(path_points)

        # 5. Publish
        self.publish_path(smoothed_points)

    def smooth_path(self, path, iterations=3):
        """
        Simple Moving Average Smoothing.
        """
        # Create a copy so we don't modify the original while reading it
        new_path = list(path)
        
        for _ in range(iterations):
            # Skip Start and End points (they must stay fixed)
            for i in range(1, len(path) - 1):
                prev_p = new_path[i-1]
                curr_p = new_path[i]
                next_p = new_path[i+1]
                
                # Average: (Prev + Curr + Next) / 3
                new_x = (prev_p[0] + curr_p[0] + next_p[0]) / 3.0
                new_y = (prev_p[1] + curr_p[1] + next_p[1]) / 3.0
                
                new_path[i] = (new_x, new_y)
                
        return new_path

    def run_astar(self, start, goal):
        self.get_logger().info(f"Planning from {start} to {goal}...")
        
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        
        while open_list:
            current_f, current = heapq.heappop(open_list)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            # 8-connected neighbors
            neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
            
            for dr, dc in neighbors:
                neighbor = (current[0] + dr, current[1] + dc)
                
                if self.grid.is_blocked(neighbor[0], neighbor[1]):
                    continue
                
                dist = 1.414 if abs(dr) + abs(dc) == 2 else 1.0
                tentative_g = g_score[current] + dist
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    
        return None 

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for x, y in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)
        self.get_logger().info(f"Published Smoothed Path with {len(path_msg.poses)} waypoints.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

