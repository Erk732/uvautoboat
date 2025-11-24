import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import struct
import math

class SimplePerception(Node):
    def __init__(self):
        super().__init__('simple_perception_node')

        # Subscribe to VRX Lidar
        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv/points',
            self.lidar_callback,
            10)

        # Publish to your Planner
        self.pub_obstacles = self.create_publisher(PoseArray, '/perception/obstacles', 10)

        self.get_logger().info('Simple Perception Node Started. Watching for buoys...')

    def lidar_callback(self, msg):
        # We need to parse the raw byte data from PointCloud2
        # This is a manual parser to avoid dependency issues
        
        # Calculate offset for X, Y, Z fields
        x_offset = -1
        y_offset = -1
        z_offset = -1
        
        for i, field in enumerate(msg.fields):
            if field.name == 'x': x_offset = field.offset
            if field.name == 'y': y_offset = field.offset
            if field.name == 'z': z_offset = field.offset

        if x_offset == -1 or y_offset == -1: return

        point_step = msg.point_step
        data = msg.data
        
        obstacles = []
        
        # We will act like a "Cluster Detector"
        # If we see a point close to the boat that is NOT water, we mark it.
        
        detected_x = 0.0
        detected_y = 0.0
        count = 0

        # Scan through points (skip every 10th point to save CPU)
        for i in range(0, len(data), point_step * 10):
            # Unpack float32 from bytes
            x_bytes = data[i+x_offset : i+x_offset+4]
            y_bytes = data[i+y_offset : i+y_offset+4]
            z_bytes = data[i+z_offset : i+z_offset+4]
            
            x = struct.unpack('f', x_bytes)[0]
            y = struct.unpack('f', y_bytes)[0]
            z = struct.unpack('f', z_bytes)[0]

            # FILTERS:
            # 1. Range: Only look 2m to 20m in front
            if x < 2.0 or x > 20.0: continue
            
            # 2. Width: Only look 5m to the left/right
            if abs(y) > 5.0: continue
            
            # 3. Height: Ignore water (usually z < 0.5 or z > -0.5 depending on mounting)
            # VRX Lidar is usually high up. Objects are roughly same height as Lidar or slightly lower.
            # Water is usually much lower. Let's assume anything "solid" is an obstacle.
            # (Refining this depends on your specific Lidar mounting z-height)
            
            detected_x += x
            detected_y += y
            count += 1

        # If we found enough points, average them to find the "Center" of the buoy
        if count > 5:
            avg_x = detected_x / count
            avg_y = detected_y / count
            
            # Create Obstacle Message
            pose_array = PoseArray()
            pose_array.header = msg.header # Use same frame as Lidar
            
            p = Pose()
            p.position.x = avg_x
            p.position.y = avg_y
            p.position.z = 0.0
            pose_array.poses.append(p)
            
            self.pub_obstacles.publish(pose_array)
            # self.get_logger().info(f'Obstacle Detected at X: {avg_x:.2f}, Y: {avg_y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
