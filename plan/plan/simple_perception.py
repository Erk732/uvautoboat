import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import struct
import math

class SimplePerception(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Subscribe to the 3D Lidar topic from your list
        self.subscription = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data) # Best Effort is CRITICAL
        
        self.obstacle_pub = self.create_publisher(PoseArray, '/perception/obstacles', 10)
        self.get_logger().info('Perception Node Ready (3D Mode)')

    def listener_callback(self, msg):
        obstacles = PoseArray()
        obstacles.header = msg.header
        
        # Read the binary data from the point cloud
        # We process every 10th point to keep it fast
        point_step = msg.point_step
        data = msg.data
        
        for i in range(0, len(data), point_step * 10):
            try:
                # Unpack x, y, z (assumes float32)
                x, y, z = struct.unpack_from('fff', data, i)
            except:
                continue

            dist = math.sqrt(x*x + y*y)
            
            # 1. Distance Filter: Ignore boat (<1m) and far mountains (>30m)
            if dist < 1.0 or dist > 30.0:
                continue
                
            # 2. Height Filter: Ignore water (z < -0.5) and sky (z > 1.5)
            if z < -0.5 or z > 1.5:
                continue
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            obstacles.poses.append(pose)

        if len(obstacles.poses) > 0:
            self.obstacle_pub.publish(obstacles)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePerception()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()