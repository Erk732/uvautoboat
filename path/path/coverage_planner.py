import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner_node')

        # Subscribe to a goal (this will be the CENTER of our search area)
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/planning/goal',
            self.goal_callback,
            10)

        self.pub_path = self.create_publisher(Path, '/planning/path', 10)
        
        # Settings for the search
        self.search_width = 40.0  # meters (Total width)
        self.search_height = 40.0 # meters (Total height)
        self.lane_spacing = 5.0   # meters (Distance between lines)

        self.get_logger().info('Coverage Planner Ready. Send a Goal to set the Search Center.')

    def goal_callback(self, msg):
        center_x = msg.pose.position.x
        center_y = msg.pose.position.y
        self.get_logger().info(f'Generating search pattern around ({center_x}, {center_y})...')
        
        self.generate_lawnmower(center_x, center_y)

    def generate_lawnmower(self, cx, cy):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # Calculate boundaries
        min_x = cx - (self.search_width / 2.0)
        max_x = cx + (self.search_width / 2.0)
        min_y = cy - (self.search_height / 2.0)
        max_y = cy + (self.search_height / 2.0)

        # Generate Zig-Zag points
        current_y = min_y
        going_right = True

        while current_y <= max_y:
            # We add two points per "lane": Start and End
            
            if going_right:
                # Left -> Right
                self.add_waypoint(path_msg, min_x, current_y)
                self.add_waypoint(path_msg, max_x, current_y)
            else:
                # Right -> Left
                self.add_waypoint(path_msg, max_x, current_y)
                self.add_waypoint(path_msg, min_x, current_y)

            # Move up to next lane
            current_y += self.lane_spacing
            going_right = not going_right

        self.pub_path.publish(path_msg)
        self.get_logger().info(f'Published Coverage Path with {len(path_msg.poses)} waypoints.')

    def add_waypoint(self, path_msg, x, y):
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        path_msg.poses.append(pose)

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
