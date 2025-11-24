import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data # <--- THE FIX
import math
import heapq

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray
from path.grid_map import GridMap

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner_node')

        # 1. Subscribe to Odom with QoS FIX (Best Effort)
        self.sub_odom = self.create_subscription(
            Odometry, 
            '/wamv/sensors/odometry', 
            self.odom_callback, 
            qos_profile_sensor_data) # <--- Using Sensor Data QoS

        # 2. Subscribe to Goal
        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)

        # 3. Subscribe to Obstacles
        self.sub_obstacles = self.create_subscription(PoseArray, '/perception/obstacles', self.obstacle_callback, 10)

        # 4. Publisher
        self.pub_path = self.create_publisher(Path, '/planning/path', 10)

        self.current_pose = None
        self.goal_pose = None
        self.grid = GridMap(width_m=200, height_m=200, resolution=1.0)

        self.get_logger().info('A* Planner Ready. QoS set to Best Effort.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def obstacle_callback(self, msg):
        if len(msg.poses) > 0:
            # Update map
            for pose in msg.poses:
                self.grid.set_obstacle(pose.position.x, pose.position.y, radius=3.0)
            # Re-plan if moving
            if self.goal_pose and self.current_pose:
                self.plan_path()

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        if self.current_pose:
            self.plan_path()
        else:
            self.get_logger().warn('Waiting for Odometry... (Check Simulation is Unpaused)')

    def plan_path(self):
        if not self.current_pose or not self.goal_pose: return

        sx = self.current_pose.position.x
        sy = self.current_pose.position.y
        gx = self.goal_pose.position.x
        gy = self.goal_pose.position.y

        start = self.grid.world_to_grid(sx, sy)
        goal = self.grid.world_to_grid(gx, gy)

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
                prev = new_path[i-1]
                curr = new_path[i]
                next_p = new_path[i+1]
                nx = (prev[0] + curr[0] + next_p[0]) / 3.0
                ny = (prev[1] + curr[1] + next_p[1]) / 3.0
                new_path[i] = (nx, ny)
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
