import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
import math

class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')

        # --- PARAMETERS ---
        # These can now be changed dynamically via command line or GUI
        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 50.0)
        self.declare_parameter('lanes', 8)
        self.declare_parameter('frame_id', 'map') # Frame for RViz

        # Publisher for the controller and RViz
        self.path_pub = self.create_publisher(Path, '/atlantis/path', 10)
        
        # Subscriber to trigger replanning (optional, e.g. from dashboard)
        self.create_subscription(Empty, '/atlantis/replan', self.replan_callback, 10)

        # Timer to publish path periodically (so RViz and Controller get it even if they start late)
        self.create_timer(1.0, self.publish_path)
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # State
        self.path_msg = Path()
        self.generate_lawnmower_path()
        
        self.get_logger().info("Atlantis Planner Started")

    def parameter_callback(self, params):
        for param in params:
            if param.name in ['scan_length', 'scan_width', 'lanes']:
                self.get_logger().info(f"Parameter {param.name} changed to {param.value}")
                # Update local variable (not strictly needed as we read from param in generate, 
                # but good for safety if cached)
        
        # Regenerate path immediately on parameter change
        self.generate_lawnmower_path()
        return rclpy.rcl_interfaces.msg.SetParametersResult(successful=True)

    def replan_callback(self, msg):
        self.generate_lawnmower_path()

    def generate_lawnmower_path(self):
        scan_length = self.get_parameter('scan_length').value
        scan_width = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        frame_id = self.get_parameter('frame_id').value

        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        # Assuming Local Tangent Plane (Start is 0,0)
        # Note: In the original code, you converted GPS to meters. 
        # The planner should work in meters relative to start.
        
        for i in range(lanes):
            # Alternate direction
            if i % 2 == 0:
                x_start = 0.0
                x_end = scan_length
            else:
                x_start = scan_length
                x_end = 0.0

            y_pos = i * scan_width

            # Create Pose for Start of Lane (optional, depending on granularity needed)
            pose_start = PoseStamped()
            pose_start.header = self.path_msg.header
            pose_start.pose.position.x = x_start
            pose_start.pose.position.y = y_pos
            self.path_msg.poses.append(pose_start)

            # Create Pose for End of Lane
            pose_end = PoseStamped()
            pose_end.header = self.path_msg.header
            pose_end.pose.position.x = x_end
            pose_end.pose.position.y = y_pos
            self.path_msg.poses.append(pose_end)

            # Add transition to next lane if not last lane
            if i < lanes - 1:
                next_y = (i + 1) * scan_width
                pose_trans = PoseStamped()
                pose_trans.header = self.path_msg.header
                pose_trans.pose.position.x = x_end
                pose_trans.pose.position.y = next_y
                self.path_msg.poses.append(pose_trans)

        self.get_logger().info(f"Generated Path with {len(self.path_msg.poses)} waypoints")
        self.publish_path()

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()