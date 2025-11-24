import rclpy
from rclpy.node import Node
import math
import heapq
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from path.grid_map import GridMap
from rclpy.qos import qos_profile_sensor_data

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        
        # --- STRATEGY 1: TF Listener (Automatic) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.update_position)

        # --- STRATEGY 2: Manual Backup (If TF fails, we use this) ---
        self.sub_odom = self.create_subscription(
            Odometry, 
            '/wamv/sensors/odometry', 
            self.odom_callback, 
            qos_profile_sensor_data)

        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        self.sub_obstacles = self.create_subscription(PoseArray, '/perception/obstacles', self.obstacle_callback, 10)
        self.pub_path = self.create_publisher(Path, '/planning/path', 10)

        self.current_pose = None
        self.goal_pose = None
        self.grid = GridMap(width_m=200, height_m=200, resolution=1.0)
        self.get_logger().info('A* Planner Ready. Waiting for Position (TF or Manual)...')

    def update_position(self):
        # Try to get position automatically
        target_frame = 'wamv/base_link'
        if not self.tf_buffer.can_transform('map', target_frame, rclpy.time.Time()):
            target_frame = 'base_link'

        try:
            t = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
            self.current_pose = Pose()
            self.current_pose.position.x = t.transform.translation.x
            self.current_pose.position.y = t.transform.translation.y
        except TransformException:
            pass # If TF fails, we just wait for Manual Odom

    def odom_callback(self, msg):
        # This is the Backup: Update position from manual command
        self.current_pose = msg.pose.pose

    def obstacle_callback(self, msg):
        if len(msg.poses) > 0:
            for pose in msg.poses:
                self.grid.set_obstacle(pose.position.x, pose.position.y, radius=3.0)
            if self.goal_pose and self.current_pose:
                self.plan_path()

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        if self.current_pose:
            self.plan_path()
        else:
            self.get_logger().warn('Waiting for Position... (Try sending Manual Odom!)')

    def plan_path(self):
        if not self.current_pose or not self.goal_pose: return
        start = self.grid.world_to_grid(self.current_pose.position.x, self.current_pose.position.y)
        goal = self.grid.world_to_grid(self.goal_pose.position.x, self.goal_pose.position.y)
        if not start or not goal: return

        path_indices = self.run_astar(start, goal)
        if path_indices:
            points = [self.grid.grid_to_world(r,c) for r,c in path_indices]
            smoothed = self.smooth_path(points)
            self.publish_path(smoothed)

    def smooth_path(self, path):
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
            
            for dr, dc in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0]+dr, current[1]+dc)
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