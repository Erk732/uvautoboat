#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DynamicMap(Node):
    def __init__(self):
        super().__init__('dynamic_map')  # Correct node name
        self.subscription = self.create_subscription(
            Pose,
            '/wamv/pose',   # Topic publishing WAM-V position
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.x_data = []
        self.y_data = []

        # Set up matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.sc, = self.ax.plot([], [], 'bo-', label='WAM-V Path')
        self.ax.set_title('Dynamic WAM-V Map')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.axis('equal')

        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=500)
        plt.show(block=False)

    def pose_callback(self, msg):
        # Store X, Y positions
        self.x_data.append(msg.position.x)
        self.y_data.append(msg.position.y)

    def update_plot(self, frame):
        self.sc.set_data(self.x_data, self.y_data)
        if self.x_data and self.y_data:
            self.ax.set_xlim(min(self.x_data) - 5, max(self.x_data) + 5)
            self.ax.set_ylim(min(self.y_data) - 5, max(self.y_data) + 5)
        self.fig.canvas.draw_idle()

def main():
    rclpy.init()
    node = DynamicMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
