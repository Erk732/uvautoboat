import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class VRXObstacleAvoidance(Node):
    """
    ROS2 node for obstacle avoidance in VRX environment
    Subscribes to laser/lidar or camera-based obstacle detection
    Publishes velocity commands to avoid obstacles
    """
    
    def __init__(self):
        super().__init__('vrx_obstacle_avoidance')
        
        # Declare and get parameters
        self.declare_parameter('min_safe_distance', 5.0)  # meters
        self.declare_parameter('max_linear_speed', 1.0)   # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Subscribe to sensor topics (adjust topic names for your VRX setup)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv/scan',  # VRX lidar topic
            self.lidar_callback,
            10
        )
        
        # Alternative: Subscribe to 3D lidar point cloud
        # self.pointcloud_sub = self.create_subscription(
        #     PointCloud2,
        #     '/wamv/sensors/lidars/lidar_wamv/points',
        #     self.pointcloud_callback,
        #     10
        # )
        
        # Subscribe to odometry for current pose/velocity
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wamv/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/wamv/cmd_vel',
            10
        )
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # State variables
        self.current_scan = None
        self.current_odom = None
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.obstacle_angle = 0.0
        
        self.get_logger().info('VRX Obstacle Avoidance Node Started')
        self.get_logger().info(f'Min Safe Distance: {self.min_safe_distance}m')
        self.get_logger().info(f'Max Linear Speed: {self.max_linear_speed}m/s')
    
    def lidar_callback(self, msg):
        """Process LIDAR scan data"""
        self.current_scan = msg
        
        # Find minimum distance and its angle
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings (inf, nan)
        valid_indices = np.isfinite(ranges)
        if not np.any(valid_indices):
            return
        
        valid_ranges = ranges[valid_indices]
        
        # Get minimum distance
        self.min_distance = np.min(valid_ranges)
        min_idx = np.argmin(ranges)
        
        # Calculate angle of closest obstacle
        self.obstacle_angle = msg.angle_min + min_idx * msg.angle_increment
        
        # Check if obstacle is within safety threshold
        self.obstacle_detected = self.min_distance < self.min_safe_distance
        
        # Analyze sectors for navigation
        self.analyze_scan_sectors(ranges, msg)
    
    def analyze_scan_sectors(self, ranges, scan_msg):
        """Divide scan into sectors to determine best direction"""
        num_ranges = len(ranges)
        
        # Divide into front, left, right sectors
        front_sector = ranges[int(num_ranges*0.4):int(num_ranges*0.6)]
        left_sector = ranges[int(num_ranges*0.6):]
        right_sector = ranges[:int(num_ranges*0.4)]
        
        # Calculate average distance in each sector (ignore inf)
        self.front_clear = np.mean(front_sector[np.isfinite(front_sector)]) if np.any(np.isfinite(front_sector)) else float('inf')
        self.left_clear = np.mean(left_sector[np.isfinite(left_sector)]) if np.any(np.isfinite(left_sector)) else float('inf')
        self.right_clear = np.mean(right_sector[np.isfinite(right_sector)]) if np.any(np.isfinite(right_sector)) else float('inf')
    
    def odom_callback(self, msg):
        """Store current odometry data"""
        self.current_odom = msg
    
    def control_loop(self):
        """Main control loop for obstacle avoidance"""
        if self.current_scan is None:
            self.get_logger().warn('No sensor data received yet', throttle_duration_sec=2.0)
            return
        
        cmd = Twist()
        
        if self.obstacle_detected:
            self.get_logger().info(
                f'⚠️  OBSTACLE DETECTED at {self.min_distance:.2f}m, angle: {math.degrees(self.obstacle_angle):.1f}°',
                throttle_duration_sec=0.5
            )
            
            # STOP or move backward if very close
            if self.min_distance < self.min_safe_distance * 0.5:
                self.get_logger().warn('🛑 CRITICAL: Stopping/Reversing!')
                cmd.linear.x = -0.3  # Reverse slowly
                cmd.angular.z = 0.0
            else:
                # Determine turn direction based on where obstacle is
                # If obstacle is on the left (positive angle), turn right (negative angular)
                # If obstacle is on the right (negative angle), turn left (positive angular)
                
                # Check which side has more clearance
                if hasattr(self, 'left_clear') and hasattr(self, 'right_clear'):
                    if self.left_clear > self.right_clear:
                        # More space on left, turn left
                        cmd.angular.z = self.max_angular_speed
                        self.get_logger().info('↺ Turning LEFT (more clearance)')
                    else:
                        # More space on right, turn right
                        cmd.angular.z = -self.max_angular_speed
                        self.get_logger().info('↻ Turning RIGHT (more clearance)')
                else:
                    # Simple avoidance: turn away from obstacle
                    if self.obstacle_angle > 0:
                        cmd.angular.z = -self.max_angular_speed  # Turn right
                    else:
                        cmd.angular.z = self.max_angular_speed   # Turn left
                
                # Slow forward movement while turning
                cmd.linear.x = self.max_linear_speed * 0.3
        
        else:
            # Path is clear - move forward
            self.get_logger().info(
                f'✓ Path clear (min dist: {self.min_distance:.2f}m)',
                throttle_duration_sec=1.0
            )
            cmd.linear.x = self.max_linear_speed
            cmd.angular.z = 0.0
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
    
    def shutdown(self):
        """Stop the robot on shutdown"""
        self.get_logger().info('Shutting down - Stopping robot')
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    
    node = VRXObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()