import rclpy
from rclpy.node import Node
import math
import heapq
import tf2_geometry_msgs 
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from rclpy.time import Time # <--- FIXED IMPORT
from brain.grid_map import GridMap 

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        
        # --- 1. TF Listener (THE ONLY SOURCE OF TRUTH) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # This timer updates the boat position AND checks for replanning
        self.create_timer(0.1, self.control_loop)

        # --- 2. Subscribers ---
        # REMOVED: self.sub_odom (To prevent race condition)
        
        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        self.sub_obstacles = self.create_subscription(PoseArray, '/perception/obstacles', self.obstacle_callback, 10)
        self.pub_path = self.create_publisher(Path, '/planning/path', 10)

        self.current_pose = None
        self.goal_pose = None
        
        # Ensure your GridMap class handles the offset as discussed!
        self.grid = GridMap(width_m=300, height_m=300, resolution=1.0)
        
        self.get_logger().info('A* Planner Ready. Waiting for TF...')

    def control_loop(self):
        # Locate the boat on the map
        target_frame = 'wamv/base_link'
        
        # Check if transform is available
        if not self.tf_buffer.can_transform('map', target_frame, Time()):
            return

        try:
            # Use Time() to get the LATEST AVAILABLE transform (Fixes the future error)
            t = self.tf_buffer.lookup_transform('map', target_frame, Time())
            
            self.current_pose = Pose()
            self.current_pose.position.x = t.transform.translation.x
            self.current_pose.position.y = t.transform.translation.y
            
        except TransformException:
            pass 

    def obstacle_callback(self, msg):
        # 1. RESET THE GRID
        self.grid.reset() 

        # 2. TRANSFORM OBSTACLES
        if len(msg.poses) > 0:
            try:
                # Check for transform
                if not self.tf_buffer.can_transform('map', msg.header.frame_id, Time()):
                    return

                for pose in msg.poses:
                    p_stamped = PoseStamped()
                    p_stamped.header = msg.header 
                    p_stamped.pose = pose

                    # Transform into Map coordinates
                    p_in_map = self.tf_buffer.transform(p_stamped, 'map')

                    # Set obstacle (Radius 3.0m is safe, reduce to 2.0m if stuck)
                    self.grid.set_obstacle(p_in_map.pose.position.x, p_in_map.pose.position.y, radius=3.0)
                
                # Re-plan immediately
                if self.goal_pose and self.current_pose:
                    self.plan_path()

            except TransformException as e:
                self.get_logger().warn(f'Could not transform obstacles: {e}')

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"Goal received: {msg.pose.position.x}, {msg.pose.position.y}")
        if self.current_pose:
            self.plan_path()
        else:
            self.get_logger().warn('Waiting for Boat Position (TF)...')

    def plan_path(self):
        if not self.current_pose or not self.goal_pose: return
        
        # Convert World Coords -> Grid Indices
        start = self.grid.world_to_grid(self.current_pose.position.x, self.current_pose.position.y)
        goal = self.grid.world_to_grid(self.goal_pose.position.x, self.goal_pose.position.y)
        
        if not start or not goal: 
            self.get_logger().warn("Start or Goal outside Grid Map!")
            return

        path_indices = self.run_astar(start, goal)
        
        if path_indices:
            points = [self.grid.grid_to_world(r,c) for r,c in path_indices]
            smoothed = self.smooth_path(points)
            self.publish_path(smoothed)

    def smooth_path(self, path):
        if len(path) < 3: return path
        new_path = list(path)
        for _ in range(3):
            for i in range(1, len(path) - 1):
                prev, curr, next_p = new_path[i-1], new_path[i], new_path[i+1]
                new_path[i] = ((prev[0]+curr[0]+next_p[0])/3.0, (prev[1]+curr[1]+next_p[1])/3.0)
        return new_path

    def run_astar(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            # 8-Connectivity
            for dr, dc in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0]+dr, current[1]+dc)
                
                # Check Bounds and Obstacles
                if self.grid.is_blocked(neighbor[0], neighbor[1]): continue
                
                dist = 1.414 if abs(dr)+abs(dc)==2 else 1.0
                tentative_g = g_score[current] + dist
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + math.sqrt((neighbor[0]-goal[0])**2 + (neighbor[1]-goal[1])**2)
                    heapq.heappush(open_list, (f_score, neighbor))
        return None

    def publish_path(self, points):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for x,y in points:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            msg.poses.append(p)
        self.pub_path.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()