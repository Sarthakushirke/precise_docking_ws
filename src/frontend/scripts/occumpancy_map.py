#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def map_callback(self, msg):
        # Get map dimensions
        width = msg.info.width
        height = msg.info.height

        # Convert occupancy grid data to numpy array
        grid_data = np.array(msg.data).reshape((height, width))

        # Plot the map
        plt.imshow(grid_data, cmap='gray', origin='lower')
        plt.title('Occupancy Grid Map')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
