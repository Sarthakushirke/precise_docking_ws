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

        # Extract (x, y) coordinates from PointCloud2 data as numpy arrays
        self.extract_xy_from_pointcloud(pointcloud)

        # If there's a previous scan, compare with the current scan
        if self.prev_scan is not None and self.is_data_changed():
            # Call the icp_svd algorithm with the previous and current scans
            P = self.prev_scan
            Q = self.current_scan

            # Check the shapes before calling icp_svd
            print(f"Running ICP with shapes P: {P.shape}, Q: {Q.shape}")


            # Run ICP using SVD (assuming icp_svd takes numpy arrays)
            T = icp(P, Q)
            print("ICP Results:", T)

            # Apply the transformation matrix T to the current scan (Q)
            Q_h = np.ones((Q.shape[0], 3))  # Convert Q to homogeneous coordinates (N x 3)
            Q_h[:, :2] = Q  # Set x and y coordinates
            
            # Apply the transformation T
            Q_transformed_h = (T @ Q_h.T).T  # Apply transformation and convert back
            
            # Convert the transformed points back to 2D (x, y)
            Q_transformed = Q_transformed_h[:, :2]

            # Print the transformed points
            # print("Transformed Q (aligned to P):", Q_transformed)

            # Plot both previous and current scans
            self.plot_previous_and_current(Q_transformed)

        # Store the current scan as the previous scan for the next iteration
        self.prev_scan = np.copy(self.current_scan)  # Make a copy to avoid overwriting

    def extract_xy_from_pointcloud(self, pointcloud_msg):
        # PointCloud2 fields and data processing
        point_step = pointcloud_msg.point_step  # Length of a point in bytes
        data = pointcloud_msg.data  # Byte array containing the point cloud data
        num_points = pointcloud_msg.width * pointcloud_msg.height  # Number of points

        # Initialize a numpy array to store the (x, y) coordinates of the current scan
        self.current_scan = np.zeros((num_points, 2))  # 2 columns: x and y

        for i in range(num_points):
            offset = i * point_step
            # Extract x and y (each are 4 bytes, FLOAT32)
            x = struct.unpack_from('f', data, offset)[0]
            y = struct.unpack_from('f', data, offset + 4)[0]
            # Store the x and y in the current_scan array
            self.current_scan[i] = [x, y]

    def is_data_changed(self):
        """Compares the current scan with the previous one to see if the data has changed."""
        if self.prev_scan is None or self.current_scan is None:
            return False

        # Check if the number of points is different
        if self.prev_scan.shape != self.current_scan.shape:
            print("TRUE length changed")
            return True

        # Check if any point data is different
        if not np.array_equal(self.prev_scan, self.current_scan):
            print("TRUE data changed")
            return True

        print("FALSE - No data change")
        return False

    # def plot_previous_and_current(self, Q_transformed):
    #     """Plots the previous scan, current scan, and transformed scan on the same figure."""

    #     # Clear the current plot (allows for continuous updates)
    #     plt.clf()

    #     # Plot the previous scan in red
    #     if self.prev_scan is not None:
    #         plt.scatter(self.prev_scan[:, 0], self.prev_scan[:, 1], c='red', marker='o', label='Previous Scan')

    #     # Plot the current scan in blue
    #     if self.current_scan is not None:
    #         plt.scatter(self.current_scan[:, 0], self.current_scan[:, 1], c='blue', marker='o', label='Current Scan')

    #     # Plot the transformed scan in green (only circle outline, no fill)
    #     if Q_transformed is not None:
    #         plt.scatter(Q_transformed[:, 0], Q_transformed[:, 1], edgecolors='green', facecolors='none', marker='o', label='Transformed Scan')

    #     # Configure the plot
    #     plt.title('Previous, Current, and Transformed Scans')
    #     plt.xlabel('X Coordinate (meters)')
    #     plt.ylabel('Y Coordinate (meters)')
    #     plt.legend()
    #     plt.grid(True)

    #     # Use plt.pause to update the figure without blocking
    #     plt.pause(0.01)  # Pause for a small time to allow plot to update




def main(args=None):
    rclpy.init(args=args)
    laser_to_pointcloud = LaserToPointCloud()
    rclpy.spin(laser_to_pointcloud)
    # Destroy the node explicitly
    laser_to_pointcloud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
