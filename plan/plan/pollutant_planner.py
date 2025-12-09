import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import random
import math
import json

class PollutantPlanner(Node):
    def __init__(self):
        super().__init__('pollutant_planner')

        # --- PARAMETERS ---
        self.declare_parameter('num_waypoints', 5)
        self.declare_parameter('pollutant_threshold', 0.5)  # Example threshold

        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.pollutant_threshold = self.get_parameter('pollutant_threshold').value

        # --- STATE ---
        self.waypoints = self.generate_waypoints()
        
        # --- PUBLISHERS ---
        self.path_pub = self.create_publisher(Path, '/atlantis/path', 10)
        self.pollutant_pub = self.create_publisher(String, '/atlantis/pollutant_status', 10)

        # --- TIMER ---
        self.create_timer(2.0, self.publish_path_and_pollutants)

        self.get_logger().info("Pollutant Planner Ready")

    def generate_waypoints(self):
        """Generate dummy waypoints for the mission (x, y) in meters."""
        waypoints = []
        for i in range(self.num_waypoints):
            x = i * 10.0
            y = math.sin(i) * 5.0
            waypoints.append((x, y))
        return waypoints

    def detect_pollutants(self, x, y):
        """
        Dummy pollutant detection.
        Replace this with actual sensor data subscription in future.
        Returns a dict: {'x': x, 'y': y, 'pollutant_level': float, 'alert': bool}
        """
        pollutant_level = random.random()  # Simulate sensor reading 0.0 - 1.0
        alert = pollutant_level > self.pollutant_threshold
        return {
            'x': x,
            'y': y,
            'pollutant_level': pollutant_level,
            'alert': alert
        }

    def publish_path_and_pollutants(self):
        """Publish path and pollutant info to AtlantisController."""
        # 1. Publish Path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(self.waypoints)} waypoints")

        # 2. Publish pollutant status
        for wp in self.waypoints:
            pollutant_info = self.detect_pollutants(wp[0], wp[1])
            msg = String()
            msg.data = json.dumps(pollutant_info)
            self.pollutant_pub.publish(msg)
            if pollutant_info['alert']:
                self.get_logger().warn(f"High pollutant detected at ({wp[0]:.1f}, {wp[1]:.1f}): {pollutant_info['pollutant_level']:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PollutantPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
