#!/usr/bin/env python3
"""
TF Broadcaster using Gazebo pose directly

This version polls the Gazebo /world/sydney_regatta/dynamic_pose/info topic
directly via gz command, extracting the wamv pose.

This is the most accurate method when ground_truth_odometry isn't bridged.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import subprocess
import re
import threading
import time


class TFBroadcasterGazebo(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_gazebo')
        
        self.br = TransformBroadcaster(self)
        
        # Current pose (updated by background thread)
        self.current_pose = None
        self.lock = threading.Lock()
        
        # Start background thread to poll Gazebo
        self.running = True
        self.gz_thread = threading.Thread(target=self.poll_gazebo, daemon=True)
        self.gz_thread.start()
        
        # Timer to broadcast TF at 50Hz
        self.timer = self.create_timer(0.02, self.broadcast_tf)
        
        self.get_logger().info("📢 TF Broadcaster (Gazebo Direct) Started!")
        self.get_logger().info("   Polling Gazebo for wamv pose directly")
        
    def poll_gazebo(self):
        """Background thread that polls Gazebo for pose data"""
        while self.running:
            try:
                # Run gz topic echo to get one message
                result = subprocess.run(
                    ['gz', 'topic', '-e', '-t', '/world/sydney_regatta/dynamic_pose/info', '-n', '1'],
                    capture_output=True,
                    text=True,
                    timeout=2.0
                )
                
                if result.returncode == 0:
                    pose = self.parse_wamv_pose(result.stdout)
                    if pose:
                        with self.lock:
                            self.current_pose = pose
                            
            except subprocess.TimeoutExpired:
                pass
            except Exception as e:
                self.get_logger().warn(f"Gazebo poll error: {e}")
                
            time.sleep(0.05)  # Poll at ~20Hz
            
    def parse_wamv_pose(self, output):
        """Parse wamv pose from gz topic output"""
        # Split into pose blocks
        pose_blocks = output.split('pose {')
        
        for block in pose_blocks:
            if 'name: "wamv"' in block and 'name: "wamv/' not in block:
                # This is the main wamv pose (not a link)
                pose = {}
                
                # Extract position
                pos_match = re.search(
                    r'position\s*\{\s*x:\s*([-\d.e]+)\s*y:\s*([-\d.e]+)\s*z:\s*([-\d.e]+)',
                    block, re.DOTALL
                )
                if pos_match:
                    pose['x'] = float(pos_match.group(1))
                    pose['y'] = float(pos_match.group(2))
                    pose['z'] = float(pos_match.group(3))
                else:
                    # Handle missing values (default to 0)
                    pose['x'] = float(re.search(r'x:\s*([-\d.e]+)', block).group(1)) if re.search(r'x:\s*([-\d.e]+)', block) else 0.0
                    pose['y'] = float(re.search(r'y:\s*([-\d.e]+)', block).group(1)) if re.search(r'y:\s*([-\d.e]+)', block) else 0.0
                    pose['z'] = float(re.search(r'z:\s*([-\d.e]+)', block).group(1)) if re.search(r'z:\s*([-\d.e]+)', block) else 0.0
                    
                # Extract orientation
                ori_section = block[block.find('orientation'):] if 'orientation' in block else ''
                pose['qx'] = float(re.search(r'x:\s*([-\d.e]+)', ori_section).group(1)) if re.search(r'x:\s*([-\d.e]+)', ori_section) else 0.0
                pose['qy'] = float(re.search(r'y:\s*([-\d.e]+)', ori_section).group(1)) if re.search(r'y:\s*([-\d.e]+)', ori_section) else 0.0
                pose['qz'] = float(re.search(r'z:\s*([-\d.e]+)', ori_section).group(1)) if re.search(r'z:\s*([-\d.e]+)', ori_section) else 0.0
                pose['qw'] = float(re.search(r'w:\s*([-\d.e]+)', ori_section).group(1)) if re.search(r'w:\s*([-\d.e]+)', ori_section) else 1.0
                
                return pose
                
        return None
        
    def broadcast_tf(self):
        """Broadcast current pose as TF"""
        with self.lock:
            pose = self.current_pose
            
        if pose is None:
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'wamv/wamv/base_link'
        
        t.transform.translation.x = pose['x']
        t.transform.translation.y = pose['y']
        t.transform.translation.z = pose['z']
        
        t.transform.rotation.x = pose['qx']
        t.transform.rotation.y = pose['qy']
        t.transform.rotation.z = pose['qz']
        t.transform.rotation.w = pose['qw']
        
        self.br.sendTransform(t)
        
    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterGazebo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
