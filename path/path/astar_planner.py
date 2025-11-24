import rclpy
from rclpy.node import Node
import math
import heapq

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray

# Import the GridMap class
from path.grid_map import GridMap

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner_node')

        # --- Interfaces ---
        # 1. Listen to Boat Position
        self.sub_odom = self.create_subscription(Odometry, '/wamv/sensors/odometry', self.odom_callback, 10)
        
        # 2. Listen to User Goal
        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        
        # 3. Listen to DETECTED OBSTACLES (New!)
        self.sub_obstacles = self.create_subscription(PoseArray, '/perception/obstacles', self.obstacle_callback, 10)
        
        # 4. Publish Path
        self.pub_path = self.create_publisher(Path, '/planning/path', 10)

        # --- Internal State ---
        self.current_pose = None
        self.goal_pose = None
        
        # --- The Map ---
        self.grid = GridMap(width_m=200, height_m=200, resolution=1.0)
        
        self.get_logger().info('Dynamic A* Planner Ready. Waiting for Obstacles or Goal...')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def obstacle_callback(self, msg):
        """
        Receives a list of detected objects (buoys) and updates the map.
        """
        obstacles_detected = len(msg.poses)
        if obstacles_detected > 0:
            self.get_logger().info(f'⚠️ Detected {obstacles_detected} obstacles! Updating Map...')
            
            for pose in msg.poses:
                # Get x, y of the obstacle
                ox = pose.position.x
                oy = pose.position.y
                # Mark it on the grid (radius 3.0m safety buffer)
                self.grid.set_obstacle(ox, oy, radius=3.0)
            
            # If we already have a goal, RE-PLAN immediately!
            if self.goal_pose and self.current_pose:
                self.get_logger().info('Obstacle appeared on map. Re-planning path...')
                self.plan_path()

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
        
        start_idx = self.grid.world_to_grid(start_x, start_y)
        goal_idx = self.grid.world_to_grid(goal_x, goal_y)
        
        if not start_idx or not goal_idx:
            return

        path_indices = self.run_astar(start_idx, goal_idx)
        
        if not path_indices:
            self.get_logger().error("No path found! Obstacles are blocking the way.")
            return

        path_points = []
        for r, c in path_indices:
            wx, wy = self.grid.grid_to_world(r, c)
            path_points.append((wx, wy))

        smoothed_points = self.smooth_path(path_points)
        self.publish_path(smoothed_points)

    def smooth_path(self, path, iterations=3):
        new_path = list(path)
        for _ in range(iterations):
            for i in range(1, len(path) - 1):
                prev_p = new_path[i-1]
                curr_p = new_path[i]
                next_p = new_path[i+1]
                new_x = (prev_p[0] + curr_p[0] + next_p[0]) / 3.0
                new_y = (prev_p[1] + curr_p[1] + next_p[1]) / 3.0
                new_path[i] = (new_x, new_y)
        return new_path

    def run_astar(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_list:
            current_f, current = heapq.heappop(open_list)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
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

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Build and Run

1.  **Build:** (Always rebuild after changing python files to be safe)
    ```bash
    cd ~/seal_ws
    colcon build --packages-select path
    source install/setup.bash
    ```

2.  **Run the Planner:**
    ```bash
    ros2 run path astar_planner
    ```

### Step 3: Test the "Dynamic Detection"

We are going to play a game. We will tell the boat to go straight, and then **while it is planning**, we will throw a fake obstacle in its way using the terminal.

**1. Setup (Terminator Panes):**
* **Top Left:** `astar_planner` running.
* **Top Right:** Rviz (Topic `/planning/path`).
* **Bottom:** Your command center.

**2. Send Start and Goal (The Straight Line):**
```bash
# Start at 0,0
ros2 topic pub --once /wamv/sensors/odometry nav_msgs/msg/Odometry "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}"

# Go to 50,0
ros2 topic pub --once /planning/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 50.0, y: 0.0, z: 0.0}}}"
```
*Look at Rviz:* You should see a **straight line** because we removed the hardcoded island.

**3. "Detect" a Buoy! (The Magic)**
Now, simulate that your Perception team found a buoy right in the middle of the path (at x=25, y=0). Run this command:

```bash
ros2 topic pub --once /perception/obstacles geometry_msgs/msg/PoseArray "{header: {frame_id: 'map'}, poses: [{position: {x: 25.0, y: 0.0, z: 0.0}}]}"