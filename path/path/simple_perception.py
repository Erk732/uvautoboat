import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import math

class SimplePerception(Node):
    def __init__(self):
        super().__init__('simple_perception_node')
        # Listen to the simpler LaserScan topic
        self.sub_lidar = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv/scan',
            self.lidar_callback,
            10)
        self.pub_obstacles = self.create_publisher(PoseArray, '/perception/obstacles', 10)

    def lidar_callback(self, msg):
        pose_array = PoseArray()
        pose_array.header = msg.header
        
        # Check every 10th ray to save speed
        for i in range(0, len(msg.ranges), 10):
            dist = msg.ranges[i]
            
            # Filter bad data
            if dist == float('inf') or math.isnan(dist): continue
            
            # If an object is close (between 1m and 15m)
            if 1.0 < dist < 15.0:
                # Calculate X, Y from Angle and Distance
                angle = msg.angle_min + (i * msg.angle_increment)
                
                # Polar to Cartesian coordinates
                obs_x = dist * math.cos(angle)
                obs_y = dist * math.sin(angle)
                
                p = Pose()
                p.position.x = obs_x
                p.position.y = obs_y
                pose_array.poses.append(p)

        if len(pose_array.poses) > 0:
            self.pub_obstacles.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()