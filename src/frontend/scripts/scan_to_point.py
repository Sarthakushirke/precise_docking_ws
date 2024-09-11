#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import struct
import math  # For checking inf values
import matplotlib.pyplot as plt  # For plotting the graph

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Topic from which to read LaserScan messages
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.laser_projector = LaserProjection()

        # Lists to store x and y coordinates
        self.x_points = []
        self.y_points = []

    def listener_callback(self, msg):
        # Print the total number of valid ranges (excluding inf values)
        self.print_valid_ranges(msg)

        # Convert the LaserScan to PointCloud2
        pointcloud = self.laser_projector.projectLaser(msg)
        # Publish the PointCloud2 data
        self.publisher.publish(pointcloud)

        # Extract and plot (x, y) coordinates from PointCloud2 data
        self.extract_xy_from_pointcloud(pointcloud)
        self.plot_xy_points()

    def print_valid_ranges(self, scan_msg):
        # Filter out 'inf' values and count only valid ranges
        valid_ranges = [r for r in scan_msg.ranges if not math.isinf(r)]
        total_valid_ranges = len(valid_ranges)
        # self.get_logger().info(f"Total number of valid ranges: {total_valid_ranges}")

    def extract_xy_from_pointcloud(self, pointcloud_msg):
        # Clear the x and y points lists before adding new data
        self.x_points.clear()
        self.y_points.clear()

        # PointCloud2 fields and data processing
        point_step = pointcloud_msg.point_step  # Length of a point in bytes
        data = pointcloud_msg.data  # Byte array containing the point cloud data
        num_points = pointcloud_msg.width * pointcloud_msg.height  # Number of points
        # self.get_logger().info(f"Total number of points in PointCloud2: {num_points}")

        for i in range(num_points):
            offset = i * point_step
            # Extract x and y (each are 4 bytes, FLOAT32)
            x = struct.unpack_from('f', data, offset)[0]
            y = struct.unpack_from('f', data, offset + 4)[0]
            # Append x, y to lists for plotting
            self.x_points.append(x)
            self.y_points.append(y)

    def plot_xy_points(self):
        # Plot the x, y points on a graph
        plt.figure(figsize=(8, 6))
        plt.scatter(self.x_points, self.y_points, c='blue', marker='o',s=10)
        plt.title('2D LiDAR Points')
        plt.xlabel('X Coordinate (meters)')
        plt.ylabel('Y Coordinate (meters)')
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    laser_to_pointcloud = LaserToPointCloud()
    rclpy.spin(laser_to_pointcloud)
    # Destroy the node explicitly
    laser_to_pointcloud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
