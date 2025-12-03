import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import SetParametersResult
import json
import threading
import sys
import time
import math
import struct

# Import new obstacle avoidance module
from .lidar_obstacle_avoidance import (
    LidarObstacleDetector,
    ObstacleClustering,
    ObstacleAvoider,
    RealtimeObstacleMonitor
)

class AtlantisPlanner(Node):
    def __init__(self):
        super().__init__('atlantis_planner')

        # --- PARAMETERS ---
        self.declare_parameter('scan_length', 150.0)
        self.declare_parameter('scan_width', 20.0) 
        self.declare_parameter('lanes', 4)
        self.declare_parameter('frame_id', 'map') 

        # --- GEOFENCE PARAMETERS ---
        self.declare_parameter('geo_min_x', -50.0)
        self.declare_parameter('geo_max_x', 200.0)
        self.declare_parameter('geo_min_y', -100.0)
        self.declare_parameter('geo_max_y', 100.0)
        
        # --- OBSTACLE AVOIDANCE PARAMETERS ---
        self.declare_parameter('planner_safe_dist', 10.0)  # Keep waypoints 10m away from obstacles
        self.declare_parameter('obstacle_lookahead', 15.0)  # Look-ahead distance for avoidance
        self.declare_parameter('obstacle_cluster_radius', 2.0)  # Radius for clustering
        self.declare_parameter('obstacle_min_distance', 5.0)  # Min LIDAR range
        self.declare_parameter('obstacle_max_distance', 50.0)  # Max LIDAR range
        self.declare_parameter('lidar_sampling_factor', 50)  # Process every Nth point

        # Publishers
        self.path_pub = self.create_publisher(Path, '/atlantis/path', 10)
        self.pub_waypoints = self.create_publisher(String, '/atlantis/waypoints', 10)
        self.pub_config = self.create_publisher(String, '/atlantis/config', 10)
        self.pub_obstacle_map = self.create_publisher(String, '/atlantis/obstacle_map', 10)

        # Subscribers
        self.create_subscription(Empty, '/atlantis/replan', self.replan_callback, 10)
        
        # NEW: Listen to LIDAR to map obstacles
        self.create_subscription(
            PointCloud2, 
            '/wamv/sensors/lidars/lidar_wamv_sensor/points', 
            self.lidar_callback, 
            10
        )
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timers
        self.create_timer(1.0, self.publish_path)
        self.create_timer(1.0, self.publish_config)
        self.create_timer(0.5, self.publish_obstacle_map)

        # State
        self.path_msg = Path()
        self.waypoints = []
        self.shutdown_flag = False
        self.known_obstacles = []  # List of (x, y) tuples
        
        # --- OBSTACLE AVOIDANCE MODULES ---
        self.lidar_detector = LidarObstacleDetector(
            min_distance=self.get_parameter('obstacle_min_distance').value,
            max_distance=self.get_parameter('obstacle_max_distance').value
        )
        self.clusterer = ObstacleClustering(
            cluster_radius=self.get_parameter('obstacle_cluster_radius').value
        )
        self.avoider = ObstacleAvoider(
            safe_distance=self.get_parameter('planner_safe_dist').value,
            look_ahead=self.get_parameter('obstacle_lookahead').value
        )
        self.monitor = RealtimeObstacleMonitor()
        
        # Current obstacles from latest LIDAR scan
        self.current_obstacles = []
        self.current_clusters = []
        
        # --- SAFETY FEATURES ---
        # Path versioning for thread safety
        self.path_lock = threading.RLock()
        self.path_version = 0
        self.mission_enabled = False  # Check before allowing param changes
        
        # --- LIDAR SIGNAL MONITORING ---
        self.lidar_signal_count = 0  # Total LIDAR signals received
        self.lidar_obstacle_detections = 0  # Signals with obstacles detected
        self.lidar_clear_detections = 0  # Signals with no obstacles
        self.last_signal_time = None
        self.lidar_detection_rate = 0.0  # Percentage with obstacles
        
        # Create timer for monitoring stats
        self.create_timer(2.0, self.publish_lidar_statistics)
        
        # --- AUTO START ---
        # Wait a moment for LIDAR to warm up before generating first path
        threading.Timer(2.0, self.generate_lawnmower_path).start()
        
        # Start Input Thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("Atlantis Planner Started - Path Generated Automatically")

    def input_loop(self):
        time.sleep(1.0)
        print("\n" + "="*40)
        print("  ATLANTIS MISSION CONTROL")
        print("  Type 'r' + ENTER -> Regenerate Lawnmower Path (Checks Obstacles)")
        print("  Type 'b' + ENTER -> Return to Home Base (0,0)")
        print("  [Ctrl+C]         -> STOP everything")
        print("="*40 + "\n")

        while rclpy.ok() and not self.shutdown_flag:
            try:
                user_input = input("Command (r/b) > ").strip().lower()
                if user_input == 'b':
                    self.generate_return_home_path()
                elif user_input == 'r':
                    self.generate_lawnmower_path()
            except EOFError:
                break
            except Exception:
                pass

    def lidar_callback(self, msg):
        """Enhanced: Process LIDAR with monitoring and safety"""
        try:
            # --- LIDAR SIGNAL MONITORING ---
            self.lidar_signal_count += 1
            self.last_signal_time = self.get_clock().now()
            
            # Extract obstacles using new detector
            point_step = msg.point_step
            sampling_factor = self.get_parameter('lidar_sampling_factor').value
            
            self.current_obstacles = self.lidar_detector.process_pointcloud(
                msg.data, point_step, sampling_factor
            )
            
            # Track obstacle detections
            if len(self.current_obstacles) > 0:
                self.lidar_obstacle_detections += 1
            else:
                self.lidar_clear_detections += 1
            
            # Calculate detection rate
            if self.lidar_signal_count > 0:
                self.lidar_detection_rate = (
                    self.lidar_obstacle_detections / self.lidar_signal_count * 100.0
                )
            
            # Cluster obstacles
            self.current_clusters = self.clusterer.cluster_obstacles(self.current_obstacles)
            
            # Analyze sectors for real-time monitoring
            self.monitor.analyze_sectors(self.current_obstacles)
            
            # Update known obstacles for path planning
            self.known_obstacles = [(obs.x, obs.y) for obs in self.current_obstacles]
            
            # Log sector analysis occasionally
            if len(self.current_obstacles) > 0:
                if self.monitor.is_critical(threshold=8.0):
                    self.get_logger().warn(
                        f"LIDAR: Front={self.monitor.front_distance:.1f}m, "
                        f"Left={self.monitor.left_distance:.1f}m, "
                        f"Right={self.monitor.right_distance:.1f}m "
                        f"({len(self.current_obstacles)} obstacles detected)"
                    )
                    
        except Exception as e:
            self.get_logger().error(f"LIDAR callback error: {e}")

    def is_point_safe(self, x, y):
        """Enhanced: Check if point is too close to known obstacles"""
        safe_dist = self.get_parameter('planner_safe_dist').value
        
        for obs_x, obs_y in self.known_obstacles:
            # Distance check
            dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < safe_dist:
                return False, obs_x, obs_y
        return True, None, None

    def adjust_point_for_obstacles(self, x, y, target_x=None, target_y=None):
        """
        Enhanced: If point is unsafe, shifts it away from obstacles toward target
        
        Args:
            x, y: Point to check
            target_x, target_y: Original target (for smart direction)
            
        Returns:
            Adjusted (x, y) coordinates
        """
        is_safe, obs_x, obs_y = self.is_point_safe(x, y)
        
        if not is_safe:
            self.get_logger().warn(f"Waypoint ({x:.1f}, {y:.1f}) is unsafe! Adjusting...")
            safe_dist = self.get_parameter('planner_safe_dist').value
            
            # If we have target, shift away from obstacle toward target
            if target_x is not None and target_y is not None:
                # Direction from obstacle to target
                dx = target_x - obs_x
                dy = target_y - obs_y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > 0.1:
                    # Shift in direction away from obstacle
                    shift_x = (dx / dist) * (safe_dist + 2.0)
                    shift_y = (dy / dist) * (safe_dist + 2.0)
                    return obs_x + shift_x, obs_y + shift_y
            
            # Fallback: Simple perpendicular shift
            return x, y + safe_dist + 2.0
            
        return x, y

    def parameter_callback(self, params):
        """Handle parameter changes with safety checks"""
        
        # SAFETY: Block parameter changes during mission
        if self.mission_enabled:
            self.get_logger().error(
                "❌ Cannot change parameters during active mission! "
                "Stop mission first."
            )
            return SetParametersResult(successful=False)
        
        # Validate new parameters before applying
        for param in params:
            # Validate scan_length
            if param.name == 'scan_length':
                if param.value < 50.0:
                    self.get_logger().error(
                        f"❌ Parameter validation failed: scan_length must be >= 50.0, "
                        f"got {param.value}"
                    )
                    return SetParametersResult(successful=False)
            
            # Validate lanes
            if param.name == 'lanes':
                if param.value < 1 or param.value > 20:
                    self.get_logger().error(
                        f"❌ Parameter validation failed: lanes must be between 1-20, "
                        f"got {param.value}"
                    )
                    return SetParametersResult(successful=False)
            
            # Validate scan_width
            if param.name == 'scan_width':
                if param.value < 5.0:
                    self.get_logger().error(
                        f"❌ Parameter validation failed: scan_width must be >= 5.0, "
                        f"got {param.value}"
                    )
                    return SetParametersResult(successful=False)
            
            self.get_logger().info(f"✓ Parameter {param.name} = {param.value} (validated)")
        
        # Safe to regenerate path
        if len(self.waypoints) > 0 and len(self.waypoints) > 2:
            self.get_logger().info("Regenerating path with new parameters...")
            self.generate_lawnmower_path()
        
        return SetParametersResult(successful=True)

    def replan_callback(self, msg):
        self.generate_lawnmower_path()

    def apply_geofence(self, x, y):
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        min_y = self.get_parameter('geo_min_y').value
        max_y = self.get_parameter('geo_max_y').value
        safe_x = max(min_x, min(x, max_x))
        safe_y = max(min_y, min(y, max_y))
        return safe_x, safe_y

    def generate_return_home_path(self):
        frame_id = self.get_parameter('frame_id').value
        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints = []

        pose = PoseStamped()
        pose.header = self.path_msg.header
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        self.path_msg.poses.append(pose)
        self.waypoints.append((0.0, 0.0))

        self.get_logger().info("GENERATED: Return to Home Path")
        self.publish_path()

    def generate_lawnmower_path(self):
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

            # 1. Apply Geofence
            safe_start_x, safe_start_y = self.apply_geofence(x_start, y_pos)
            safe_end_x, safe_end_y = self.apply_geofence(x_end, y_pos)
            
            # 2. Check Obstacles (ENHANCED: pass target for smart adjustment)
            safe_start_x, safe_start_y = self.adjust_point_for_obstacles(
                safe_start_x, safe_start_y, safe_end_x, safe_end_y
            )
            safe_end_x, safe_end_y = self.adjust_point_for_obstacles(
                safe_end_x, safe_end_y, safe_start_x, safe_start_y
            )

            # Start of lane
            pose_start = PoseStamped()
            pose_start.header = self.path_msg.header
            pose_start.pose.position.x = safe_start_x
            pose_start.pose.position.y = safe_start_y
            self.path_msg.poses.append(pose_start)
            
            # End of lane
            pose_end = PoseStamped()
            pose_end.header = self.path_msg.header
            pose_end.pose.position.x = safe_end_x
            pose_end.pose.position.y = safe_end_y
            self.path_msg.poses.append(pose_end)
            
            self.waypoints.append((safe_start_x, safe_start_y))
            self.waypoints.append((safe_end_x, safe_end_y))

            if i < lanes - 1:
                next_y = (i + 1) * scan_width
                safe_next_x, safe_next_y = self.apply_geofence(x_end, next_y)
                # Check Obstacles for transition (ENHANCED)
                safe_next_x, safe_next_y = self.adjust_point_for_obstacles(
                    safe_next_x, safe_next_y, safe_start_x, safe_start_y
                )
                
                pose_trans = PoseStamped()
                pose_trans.header = self.path_msg.header
                pose_trans.pose.position.x = safe_next_x
                pose_trans.pose.position.y = safe_next_y
                self.path_msg.poses.append(pose_trans)
                self.waypoints.append((safe_next_x, safe_next_y))

        self.get_logger().info(f"GENERATED: Mission Path ({len(self.waypoints)} points, {len(self.current_clusters)} obstacles detected)")
        self.publish_path()

    def publish_lidar_statistics(self):
        """Publish LIDAR monitoring statistics periodically"""
        try:
            # Publish every ~10 LIDAR frames (roughly 1Hz if LIDAR is 10Hz)
            publish_frequency = 10
            
            if self.lidar_signal_count % publish_frequency == 0 and self.lidar_signal_count > 0:
                stats_data = {
                    'total_signals': self.lidar_signal_count,
                    'signals_with_obstacles': self.lidar_obstacle_detections,
                    'signals_clear': self.lidar_clear_detections,
                    'obstacle_detection_rate': round(self.lidar_detection_rate, 2),
                    'path_version': self.path_version,
                    'mission_enabled': self.mission_enabled,
                    'timestamp': str(self.get_clock().now())
                }
                
                # Log statistics
                self.get_logger().info(
                    f"LIDAR Stats: Total={stats_data['total_signals']} | "
                    f"With_Obstacles={stats_data['signals_with_obstacles']} | "
                    f"Clear={stats_data['signals_clear']} | "
                    f"Detection_Rate={stats_data['obstacle_detection_rate']}% | "
                    f"Path_Version={stats_data['path_version']}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Statistics publishing error: {e}")

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
    
    def publish_obstacle_map(self):
        """Publish current obstacle detections for visualization"""
        obstacle_data = {
            'count': len(self.current_obstacles),
            'clusters': len(self.current_clusters),
            'front_distance': self.monitor.front_distance if self.monitor.front_distance != float('inf') else 999.9,
            'left_distance': self.monitor.left_distance if self.monitor.left_distance != float('inf') else 999.9,
            'right_distance': self.monitor.right_distance if self.monitor.right_distance != float('inf') else 999.9,
            'best_direction': self.monitor.get_best_direction(),
            'is_critical': self.monitor.is_critical(threshold=8.0)
        }
        msg = String()
        msg.data = json.dumps(obstacle_data)
        self.pub_obstacle_map.publish(msg)
        
    def stop_all(self):
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