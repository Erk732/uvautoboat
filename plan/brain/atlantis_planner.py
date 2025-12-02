import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from rcl_interfaces.msg import SetParametersResult
import json
import threading
import sys

class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')

        # --- PARAMETERS ---
        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 50.0) 
        self.declare_parameter('lanes', 8)
        self.declare_parameter('frame_id', 'map') 

        # --- GEOFENCE PARAMETERS (Added these) ---
        # Safe box: -50m behind to 200m forward, and 100m wide left/right
        self.declare_parameter('geo_min_x', -50.0)
        self.declare_parameter('geo_max_x', 200.0)
        self.declare_parameter('geo_min_y', -100.0)
        self.declare_parameter('geo_max_y', 100.0)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/atlantis/path', 10)
        self.pub_waypoints = self.create_publisher(String, '/atlantis/waypoints', 10)
        self.pub_config = self.create_publisher(String, '/atlantis/config', 10)

        # Subscribers
        self.create_subscription(Empty, '/atlantis/replan', self.replan_callback, 10)
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timers
        self.create_timer(1.0, self.publish_path)
        self.create_timer(1.0, self.publish_config)

        # State
        self.path_msg = Path()
        self.waypoints = []
        self.shutdown_flag = False
        
        # Start Input Thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("Atlantis Planner Started with Geofence Protection")

    def input_loop(self):
        """Interactive command loop"""
        print("\n" + "="*40)
        print("  ATLANTIS MISSION CONTROL")
        print("  [ENTER]   -> Generate Lawnmower Path")
        print("  [b]+ENTER -> Return to Home Base (0,0)")
        print("  [Ctrl+C]  -> STOP everything")
        print("="*40 + "\n")

        while rclpy.ok() and not self.shutdown_flag:
            try:
                user_input = input("Command > ")
                
                if user_input.strip().lower() == 'b':
                    self.generate_return_home_path()
                else:
                    self.generate_lawnmower_path()
            except EOFError:
                break
            except Exception:
                pass

    def parameter_callback(self, params):
        for param in params:
            if param.name in ['scan_length', 'scan_width', 'lanes', 'geo_max_x']:
                self.get_logger().info(f"Parameter {param.name} changed to {param.value}")
        
        # Only regenerate if we already have a path
        if len(self.waypoints) > 0 and len(self.waypoints) > 2:
             self.generate_lawnmower_path()
        return SetParametersResult(successful=True)

    def replan_callback(self, msg):
        self.generate_lawnmower_path()

    # --- ADDED THIS MISSING FUNCTION ---
    def apply_geofence(self, x, y):
        """Clamps the coordinate to stay inside the Safe Box"""
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        min_y = self.get_parameter('geo_min_y').value
        max_y = self.get_parameter('geo_max_y').value

        # Clamp X
        safe_x = max(min_x, min(x, max_x))
        # Clamp Y
        safe_y = max(min_y, min(y, max_y))
            
        return safe_x, safe_y

    def generate_return_home_path(self):
        """Generates a simple path back to (0,0)"""
        frame_id = self.get_parameter('frame_id').value
        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints = []

        # Waypoint 1: (0,0) - The Home Base
        pose = PoseStamped()
        pose.header = self.path_msg.header
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        self.path_msg.poses.append(pose)
        self.waypoints.append((0.0, 0.0))

        self.get_logger().info("GENERATED: Return to Home Path")
        self.publish_path()

    def generate_lawnmower_path(self):
        """Generates the mission path"""
        scan_length = self.get_parameter('scan_length').value
        scan_width = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        frame_id = self.get_parameter('frame_id').value

        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints = [] 

        for i in range(lanes):
            if i % 2 == 0:
                x_start = 0.0
                x_end = scan_length
            else:
                x_start = scan_length
                x_end = 0.0
            y_pos = i * scan_width

            # --- APPLY GEOFENCE ---
            # We calculate the ideal points first, then clamp them
            safe_start_x, safe_start_y = self.apply_geofence(x_start, y_pos)
            safe_end_x, safe_end_y = self.apply_geofence(x_end, y_pos)
            # ----------------------

            # Start of lane (FIXED: Uses safe_start variables now)
            pose_start = PoseStamped()
            pose_start.header = self.path_msg.header
            pose_start.pose.position.x = safe_start_x
            pose_start.pose.position.y = safe_start_y
            self.path_msg.poses.append(pose_start)
            
            # End of lane (FIXED: Uses safe_end variables now)
            pose_end = PoseStamped()
            pose_end.header = self.path_msg.header
            pose_end.pose.position.x = safe_end_x
            pose_end.pose.position.y = safe_end_y
            self.path_msg.poses.append(pose_end)
            
            self.waypoints.append((safe_start_x, safe_start_y))
            self.waypoints.append((safe_end_x, safe_end_y))

            # Transition to next lane
            if i < lanes - 1:
                next_y = (i + 1) * scan_width
                
                # Geofence the transition point too
                safe_next_x, safe_next_y = self.apply_geofence(x_end, next_y)
                
                pose_trans = PoseStamped()
                pose_trans.header = self.path_msg.header
                pose_trans.pose.position.x = safe_next_x
                pose_trans.pose.position.y = safe_next_y
                self.path_msg.poses.append(pose_trans)
                self.waypoints.append((safe_next_x, safe_next_y))

        self.get_logger().info(f"GENERATED: Mission Path ({len(self.waypoints)} points)")
        self.publish_path()

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
        
        waypoint_data = {'waypoints': [{'x': wp[0], 'y': wp[1]} for wp in self.waypoints]}
        msg = String()
        msg.data = json.dumps(waypoint_data)
        self.pub_waypoints.publish(msg)

    def publish_config(self):
        config = {
            'scan_length': self.get_parameter('scan_length').value,
            'scan_width': self.get_parameter('scan_width').value,
            'lanes': self.get_parameter('lanes').value
        }
        msg = String()
        msg.data = json.dumps(config)
        self.pub_config.publish(msg)
        
    def stop_all(self):
        """Send empty path to stop controller on shutdown"""
        self.shutdown_flag = True
        empty = Path()
        empty.header.frame_id = self.get_parameter('frame_id').value
        self.path_pub.publish(empty)
        self.get_logger().warn("Planner stopping - Sent STOP command to controller")

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()