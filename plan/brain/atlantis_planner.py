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
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Related import
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
        self.declare_parameter('geo_min_x', -50.0)
        self.declare_parameter('geo_max_x', 200.0)
        self.declare_parameter('geo_min_y', -100.0)
        self.declare_parameter('geo_max_y', 100.0)
        
        # --- OBSTACLE AVOIDANCE PARAMETERS ---
        self.declare_parameter('planner_safe_dist', 10.0)
        self.declare_parameter('obstacle_lookahead', 15.0)
        self.declare_parameter('obstacle_cluster_radius', 2.0)
        self.declare_parameter('obstacle_min_distance', 5.0)
        self.declare_parameter('obstacle_max_distance', 50.0)
        self.declare_parameter('lidar_sampling_factor', 50)

        # Publishers & Subscribers
        self.path_pub = self.create_publisher(Path, '/atlantis/path', 10)
        self.pub_waypoints = self.create_publisher(String, '/atlantis/waypoints', 10)
        self.pub_config = self.create_publisher(String, '/atlantis/config', 10)
        self.pub_obstacle_map = self.create_publisher(String, '/atlantis/obstacle_map', 10)
        self.create_subscription(Empty, '/atlantis/replan', self.replan_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timers
        self.create_timer(1.0, self.publish_path)
        self.create_timer(1.0, self.publish_config)
        self.create_timer(0.5, self.publish_obstacle_map)
        self.create_timer(2.0, self.publish_lidar_statistics)

        # State
        self.path_msg = Path()
        self.waypoints = []
        self.shutdown_flag = False
        self.known_obstacles = []
        self.path_lock = threading.RLock()
        self.path_version = 0
        self.mission_enabled = False
        
        # Stats
        self.lidar_signal_count = 0
        self.lidar_obstacle_detections = 0
        self.lidar_clear_detections = 0
        self.lidar_detection_rate = 0.0
        
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
        
        self.current_obstacles = []
        self.current_clusters = []
        #--- SMOKE DETECTION SETUP ---
        self.bridge = CvBridge()
        self.smoke_detected = False  # Binary flag
        self.smoke_regions = []      # Optional: list of region coordinates

         # Subscribe to front cameras
        self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
            self.camera_callback,
            10
        )
        self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_right_camera_sensor/image_raw',
            self.camera_callback,
            10
        )

        self.get_logger().info("Atlantis Planner with camera smoke detection initialized")


        # --- FIX: DISABLED AUTO-START ---
        # threading.Timer(2.0, self.generate_lawnmower_path).start()
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("Atlantis Planner Started (Waiting for 'r' command)")

    def input_loop(self):
        time.sleep(1.0)
        print("\n" + "="*40)
        print("  ATLANTIS MISSION CONTROL")
        print("  Type 'r' + ENTER -> Regenerate Lawnmower Path")
        print("  Type 'b' + ENTER -> Return to Home Base")
        print("="*40 + "\n")
        while rclpy.ok() and not self.shutdown_flag:
            try:
                user_input = input("Command (r/b) > ").strip().lower()
                if user_input == 'b': self.generate_return_home_path()
                elif user_input == 'r': self.generate_lawnmower_path()
            except: pass

    def lidar_callback(self, msg):
        try:
            self.lidar_signal_count += 1
            sampling_factor = self.get_parameter('lidar_sampling_factor').value
            
            # Extract raw pointcloud
            self.current_obstacles = self.lidar_detector.process_pointcloud(
                msg.data, msg.point_step, sampling_factor
            )

            # Filter Smoke/Noise
            filtered = []
            for obs in self.current_obstacles:
                if hasattr(obs, "intensity") and obs.intensity < 5.0: continue
                if abs(obs.z) > 0.5: continue
                dist = math.sqrt(obs.x**2 + obs.y**2)
                if dist < 2.0: continue
                filtered.append(obs)

            self.current_obstacles = filtered

            if len(self.current_obstacles) > 0:
                self.lidar_obstacle_detections += 1
            else:
                self.lidar_clear_detections += 1

            if self.lidar_signal_count > 0:
                self.lidar_detection_rate = (
                    self.lidar_obstacle_detections / self.lidar_signal_count * 100.0
                )

            self.current_clusters = self.clusterer.cluster_obstacles(self.current_obstacles)
            self.monitor.analyze_sectors(self.current_obstacles)
            self.known_obstacles = [(obs.x, obs.y) for obs in self.current_obstacles]
                    
        except Exception as e:
            self.get_logger().error(f"LIDAR callback error: {e}")

    # ---------------- CAMERA CALLBACK ----------------
    def camera_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # ---- Simple smoke detection logic (can replace with ML model) ----
            # Convert to gray
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Threshold light gray / whitish regions as smoke
            _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

            # Count white pixels
            white_pixel_count = cv2.countNonZero(mask)
            total_pixels = gray.shape[0] * gray.shape[1]

            # If proportion of white pixels > threshold, mark smoke detected
            smoke_ratio = white_pixel_count / total_pixels
            self.smoke_detected = smoke_ratio > 0.02  # 2% threshold, adjust as needed

            # Optional: store coordinates of detected smoke (for later soft avoidance)
            self.smoke_regions = np.column_stack(np.where(mask > 0))

        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

    def is_point_safe(self, x, y):
        safe_dist = self.get_parameter('planner_safe_dist').value
        for obs_x, obs_y in self.known_obstacles:
            dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < safe_dist:
                return False, obs_x, obs_y
        return True, None, None

    def is_line_safe(self, x1, y1, x2, y2):
        """Walks along the line checking for obstacles every 2m"""
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        steps = int(dist / 2.0)
        
        if steps == 0: return True

        for i in range(steps + 1):
            t = i / steps
            check_x = x1 + (x2 - x1) * t
            check_y = y1 + (y2 - y1) * t
            
            is_safe, _, _ = self.is_point_safe(check_x, check_y)
            if not is_safe:
                return False
        return True

    def adjust_point_for_obstacles(self, x, y, target_x=None, target_y=None):
        is_safe, obs_x, obs_y = self.is_point_safe(x, y)
        if not is_safe:
            safe_dist = self.get_parameter('planner_safe_dist').value
            if target_x is not None and target_y is not None:
                dx = target_x - obs_x
                dy = target_y - obs_y
                dist = math.sqrt(dx**2 + dy**2)
                if dist > 0.1:
                    shift_x = (dx / dist) * (safe_dist + 2.0)
                    shift_y = (dy / dist) * (safe_dist + 2.0)
                    return obs_x + shift_x, obs_y + shift_y
            return x, y + safe_dist + 2.0
        
        # Soft smoke avoidance (binary)
        if self.smoke_detected:
            # Slight nudge in y-direction (example)
            y += 1.0  # adjust magnitude as needed
        return x, y
        return x, y

    def parameter_callback(self, params):
        if self.mission_enabled:
            return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def replan_callback(self, msg):
        self.generate_lawnmower_path()

    def apply_geofence(self, x, y):
        min_x = self.get_parameter('geo_min_x').value
        max_x = self.get_parameter('geo_max_x').value
        min_y = self.get_parameter('geo_min_y').value
        max_y = self.get_parameter('geo_max_y').value
        return max(min_x, min(x, max_x)), max(min_y, min(y, max_y))

    def generate_return_home_path(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.get_parameter('frame_id').value
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints = []
        
        pose = PoseStamped()
        pose.header = self.path_msg.header
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        self.path_msg.poses.append(pose)
        self.waypoints.append((0.0, 0.0))
        self.publish_path()

    def generate_lawnmower_path(self):
        scan_length = self.get_parameter('scan_length').value
        scan_width = self.get_parameter('scan_width').value
        lanes = self.get_parameter('lanes').value
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.get_parameter('frame_id').value
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints = [] 

        for i in range(lanes):
            if i % 2 == 0:
                x_start, x_end = 0.0, scan_length
            else:
                x_start, x_end = scan_length, 0.0
            y_pos = i * scan_width

            # 1. Geofence
            safe_start_x, safe_start_y = self.apply_geofence(x_start, y_pos)
            safe_end_x, safe_end_y = self.apply_geofence(x_end, y_pos)
            
            # 2. Adjust Endpoints for Obstacles
            safe_start_x, safe_start_y = self.adjust_point_for_obstacles(safe_start_x, safe_start_y, safe_end_x, safe_end_y)
            safe_end_x, safe_end_y = self.adjust_point_for_obstacles(safe_end_x, safe_end_y, safe_start_x, safe_start_y)

            # 3. CRITICAL: Check the line between them!
            if not self.is_line_safe(safe_start_x, safe_start_y, safe_end_x, safe_end_y):
                self.get_logger().warn(f"Skipping lane {i}: Path intersects obstacles!")
                continue

            # Add Start
            pose_start = PoseStamped()
            pose_start.header = self.path_msg.header
            pose_start.pose.position.x = safe_start_x
            pose_start.pose.position.y = safe_start_y
            self.path_msg.poses.append(pose_start)
            
            # Add End
            pose_end = PoseStamped()
            pose_end.header = self.path_msg.header
            pose_end.pose.position.x = safe_end_x
            pose_end.pose.position.y = safe_end_y
            self.path_msg.poses.append(pose_end)
            
            self.waypoints.append((safe_start_x, safe_start_y))
            self.waypoints.append((safe_end_x, safe_end_y))

            # Add Transition
            if i < lanes - 1:
                next_y = (i + 1) * scan_width
                safe_next_x, safe_next_y = self.apply_geofence(x_end, next_y)
                safe_next_x, safe_next_y = self.adjust_point_for_obstacles(safe_next_x, safe_next_y, safe_start_x, safe_start_y)
                
                pose_trans = PoseStamped()
                pose_trans.header = self.path_msg.header
                pose_trans.pose.position.x = safe_next_x
                pose_trans.pose.position.y = safe_next_y
                self.path_msg.poses.append(pose_trans)
                self.waypoints.append((safe_next_x, safe_next_y))

        self.get_logger().info(f"GENERATED: Mission Path ({len(self.waypoints)} points)")
        self.publish_path()

    def publish_lidar_statistics(self):
        pass

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
        waypoint_data = {'waypoints': [{'x': wp[0], 'y': wp[1]} for wp in self.waypoints]}
        msg = String()
        msg.data = json.dumps(waypoint_data)
        self.pub_waypoints.publish(msg)

    def publish_config(self):
        config = {'scan_length': self.get_parameter('scan_length').value, 'lanes': self.get_parameter('lanes').value}
        msg = String()
        msg.data = json.dumps(config)
        self.pub_config.publish(msg)
    
    def publish_obstacle_map(self):
        obstacle_data = {'count': len(self.current_obstacles)}
        msg = String()
        msg.data = json.dumps(obstacle_data)
        self.pub_obstacle_map.publish(msg)
        
    def stop_all(self):
        self.shutdown_flag = True
        self.path_pub.publish(Path())

def main(args=None):
    rclpy.init(args=args)
    node = AtlantisPlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()