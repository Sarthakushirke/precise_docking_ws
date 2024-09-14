#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import struct
import numpy as np  # Import numpy for array operations
import matplotlib.pyplot as plt  # For plotting the graphs
from frontend.icp import icp

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

        # Store previous and current scans as numpy arrays
        self.prev_scan = None
        self.current_scan = None

    def listener_callback(self, msg):
        # Convert the LaserScan to PointCloud2
        pointcloud = self.laser_projector.projectLaser(msg)
        # Publish the PointCloud2 data
        self.publisher.publish(pointcloud)


def main(args=None):
    rclpy.init(args=args)
    laser_to_pointcloud = LaserToPointCloud()
    rclpy.spin(laser_to_pointcloud)
    # Destroy the node explicitly
    laser_to_pointcloud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
