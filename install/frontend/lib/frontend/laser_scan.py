#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from collections import deque
import matplotlib.pyplot as plt
import numpy as np

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        # Subscribe to the LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Buffer to hold scan data
        self.scans = deque(maxlen=100)
        
        # Timer to periodically plot the latest scan data
        self.timer = self.create_timer(2.0, self.plot_latest_scan)  # Adjust the period as necessary

    def listener_callback(self, msg):
        # Append each new scan with its timestamp to the deque
        self.scans.append((msg.header.stamp.sec, msg.ranges))
        self.get_logger().info(f"Received scan at {msg.header.stamp.sec}")

    def plot_latest_scan(self):
        # Check if there are scans available
        if self.scans:
            _, scan = self.scans[-1]  # Get the latest scan
            angles = np.linspace(-np.pi / 2, np.pi / 2, len(scan))
            plt.figure()
            plt.polar(angles, scan, marker='.')
            plt.title('Latest Scan')
            plt.show()
        else:
            self.get_logger().info("No scan data available.")

def main(args=None):
    rclpy.init(args=args)  # Initialize the RCLPY library
    laser_scan_subscriber = LaserScanSubscriber()  # Create our node
    try:
        rclpy.spin(laser_scan_subscriber)  # Spin the node so the callback function is called.
    except KeyboardInterrupt:
        # Allow a clean exit on Ctrl+C
        print("Caught keyboard interrupt. Exiting.")
    finally:
        laser_scan_subscriber.destroy_node()
        rclpy.shutdown()  # Shutdown RCLPY

if __name__ == '__main__':
    main()
