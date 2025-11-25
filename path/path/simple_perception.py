import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import qos_profile_sensor_data
import math

class SimplePerception(Node):
    def __init__(self):
        super().__init__('simple_perception_node')
        
        # CORRECT TOPIC for your Sydney Regatta Custom world
        self.sub_lidar = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            self.lidar_callback,
            qos_profile_sensor_data)
        
        self.pub_obstacles = self.create_publisher(PoseArray, '/perception/obstacles', 10)
        self.get_logger().info('Perception Node Ready.')

    def lidar_callback(self, msg):
        pose_array = PoseArray()
        pose_array.header = msg.header
        
        for i in range(0, len(msg.ranges), 1):# Process every point
            dist = msg.ranges[i]
            if dist == float('inf') or math.isnan(dist): continue
            
            # Detect objects 1m - 20m away
            if 1.0 < dist < 20.0:
                angle = msg.angle_min + (i * msg.angle_increment)
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