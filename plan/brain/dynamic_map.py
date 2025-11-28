#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
import pandas as pd
import matplotlib.pyplot as plt

class DynamicMap(Node):
    def __init__(self, csv_file):
        super().__init__('dynamic_map_node')

        # Load static objects (buoys/posts)
        self.static_data = pd.read_csv(csv_file)
        self.static_objects = self.static_data.to_dict('records')

        # Subscribe to boat pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/wamv/pose',
            self.pose_callback,
            10
        )

        # Initialize matplotlib interactive plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12,8))
        self.boat_x = []
        self.boat_y = []

        # Plot static objects once
        self.plot_static_objects()

    def plot_static_objects(self):
        colors = {
            'mb_marker_buoy_red': 'red',
            'mb_marker_buoy_black': 'black',
            'mb_marker_buoy_green': 'green',
            'mb_marker_buoy_white': 'white',
            'mb_round_buoy_orange': 'orange',
            'mb_round_buoy_black': 'black',
            'post_0': 'brown',
            'post_1': 'brown',
            'post_2': 'brown',
            'ground_station_0': 'blue',
            'ground_station_1': 'blue',
            'ground_station_2': 'blue'
        }

        for obj in self.static_objects:
            x = obj['x']
            y = obj['y']
            name = obj['name']
            c = colors.get(name, 'gray')
            self.ax.scatter(x, y, color=c, s=100, label=name)
            self.ax.text(x+0.5, y+0.5, name, fontsize=8)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Sydney Regatta Center Map')
        self.ax.grid(True)
        self.ax.legend(loc='upper right', fontsize=8)

    def pose_callback(self, msg: PoseStamped):
        # Extract boat position
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Append to trajectory
        self.boat_x.append(x)
        self.boat_y.append(y)

        # Clear dynamic plot and redraw
        self.ax.clear()
        self.plot_static_objects()

        # Plot boat trajectory and current position
        self.ax.plot(self.boat_x, self.boat_y, 'b-', linewidth=2, label='Boat Path')
        self.ax.scatter(x, y, color='magenta', s=150, label='WAM-V')

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Sydney Regatta Center Map (Dynamic)')
        self.ax.grid(True)
        self.ax.legend(loc='upper right', fontsize=8)

        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicMap('/home/bot/ht_ws/src/uvautoboat/plan/brain/map_coordinates.csv')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
