import rclpy
from rclpy.node import Node
import math
import heapq # Priority Queue for A*

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point

# Import the GridMap class we just tested
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
        # Let's put a "Virtual Island" at (25, 10) with radius 5m
        # This blocks the straight line from (0,0) to (50,20)
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

        # 3. Publish Path
        self.publish_path(path_indices)

    def run_astar(self, start, goal):
        self.get_logger().info(f"Planning from {start} to {goal}...")
        
        # Priority Queue: stores (f_cost, current_node)
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        came_from = {} # To reconstruct path: came_from[next] = current
        g_score = {start: 0} # Cost from start to current
        
        while open_list:
            # Get node with lowest F score
            current_f, current = heapq.heappop(open_list)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            # Check neighbors (8-connected: up, down, left, right, diagonals)
            neighbors = [
                (0,1), (0,-1), (1,0), (-1,0), 
                (1,1), (1,-1), (-1,1), (-1,-1)
            ]
            
            r, c = current
            
            for dr, dc in neighbors:
                neighbor = (r + dr, c + dc)
                
                # Check bounds and obstacles
                if self.grid.is_blocked(neighbor[0], neighbor[1]):
                    continue
                
                # Distance cost: 1.0 for straight, 1.414 for diagonal
                dist = 1.414 if abs(dr) + abs(dc) == 2 else 1.0
                tentative_g = g_score[current] + dist
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    
        return None # No path found

    def heuristic(self, a, b):
        # Euclidean distance heuristic
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse() # Reverse to get Start -> Goal
        return path

    def publish_path(self, path_indices):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for r, c in path_indices:
            wx, wy = self.grid.grid_to_world(r, c)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)
        self.get_logger().info(f"Published A* Path with {len(path_msg.poses)} waypoints.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
