import rclpy
from rclpy.node import Node
import math
import heapq
import tf2_geometry_msgs 
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from rclpy.time import Time 

# Assuming grid_map is in the same folder
from .grid_map import GridMap 

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner_node')

        # --- 1. TF Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 2. Timers ---
        # Timer A: High speed loop to update boat position (10Hz)
        self.create_timer(0.1, self.update_pose)
        
        # Timer B: SLOW loop to Plan Path (0.5Hz = every 2 seconds)
        # This prevents the "Circling" behavior
        self.create_timer(2.0, self.plan_path)

        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        self.sub_obstacles = self.create_subscription(PoseArray, '/perception/obstacles', self.obstacle_callback, 10)
        self.pub_path = self.create_publisher(Path, '/planning/path', 10)

        self.current_pose = None
        self.goal_pose = None

        # Setup Grid
        self.grid = GridMap(width_m=1200, height_m=600, resolution=1.0)
        
        # --- FRAME SETTINGS ---
        self.global_frame = 'world'
        self.robot_frame = 'wamv/wamv/base_link' # there shoud 3 layers not 2 so there should be wamv/wamv/base_link

        self.get_logger().info('A* Planner Ready. Planning every 2.0 seconds.')

    def update_pose(self):
        # Locate the boat on the map using TF
        if not self.tf_buffer.can_transform(self.global_frame, self.robot_frame, Time()):
            return

        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
            self.current_pose = Pose()
            self.current_pose.position.x = t.transform.translation.x
            self.current_pose.position.y = t.transform.translation.y
        except TransformException:
            pass 

    def obstacle_callback(self, msg):
        # We UPDATE the map instantly, but we DO NOT plan here.
        self.grid.reset() 

        if len(msg.poses) > 0:
            try:
                # Ensure we can transform obstacles to map frame
                if not self.tf_buffer.can_transform(self.global_frame, msg.header.frame_id, Time()):
                    return

                for pose in msg.poses:
                    p_stamped = PoseStamped()
                    p_stamped.header = msg.header
                    p_stamped.pose = pose
                    
                    # Transform obstacle to map frame
                    p_in_map = self.tf_buffer.transform(p_stamped, self.global_frame)

                    self.grid.set_obstacle(p_in_map.pose.position.x, p_in_map.pose.position.y, radius=3.0)

            except TransformException as e:
                self.get_logger().warn(f'Obstacle TF Error: {e}')

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"Goal received. Planning starts in next cycle.")
        # We can force a plan immediately on new goal, that's safe.
        self.plan_path()

    def plan_path(self):
        # Guard clauses
        if not self.current_pose or not self.goal_pose: 
            return

        # 1. Convert World Coords -> Grid Coords
        start = self.grid.world_to_grid(self.current_pose.position.x, self.current_pose.position.y)
        goal = self.grid.world_to_grid(self.goal_pose.position.x, self.goal_pose.position.y)

        # FIX: Added better logging and fixed the 'elf' typo
        if not start: 
            self.get_logger().warn(f" Start Pos ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}) is OUTSIDE GridMap!")
            return
        if not goal: 
            self.get_logger().warn(f"Goal Pos ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f}) is OUTSIDE GridMap!")
            return  

        # 2. Run A*
        path_indices = self.run_astar(start, goal)

        # 3. Publish
        if path_indices:
            points = [self.grid.grid_to_world(r,c) for r,c in path_indices]
            smoothed = self.smooth_path(points)
            self.publish_path(smoothed)
        else:
            self.get_logger().warn("A* failed to find a path!")

    def smooth_path(self, path):
        if len(path) < 3: return path
        new_path = list(path)
        # Increased smoothing iterations slightly for straighter lines
        for _ in range(5):
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

            for dr, dc in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0]+dr, current[1]+dc)
                
                # Check Bounds and Obstacles
                if self.grid.is_blocked(neighbor[0], neighbor[1]): continue

                # Cost Calculation
                dist = 1.414 if abs(dr)+abs(dc)==2 else 1.0
                tentative_g = g_score[current] + dist

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    
                    # Heuristic (Euclidean Distance)
                    h_score = math.sqrt((neighbor[0]-goal[0])**2 + (neighbor[1]-goal[1])**2)
                    
                    # F = G + H
                    heapq.heappush(open_list, (tentative_g + h_score, neighbor))
        return None

    def publish_path(self, points):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
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