#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import matplotlib.pyplot as plt
import math


class PointCloudTransformer(Node):

    def __init__(self):
        super().__init__('odometry_path_plotter')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to odometry for robot path
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Store odometry positions (robot path)
        self.robot_path = []

        # Set up a plot for live updating
        plt.ion()  # Turn on interactive mode for live updating
        self.fig, self.ax = plt.subplots()

        # Set a threshold for filtering outliers based on distance
        self.outlier_threshold = 0.1  # Set to a reasonable threshold (meters)

    def odom_callback(self, msg):
        # Extract position and orientation from odometry data
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom

        # Transform odometry pose to the map frame
        transformed_pose = self.transform_odometry_to_map(odom_pose.pose)

        if transformed_pose:
            # Check if the new point is an outlier based on distance
            if self.is_outlier(transformed_pose.position.x, transformed_pose.position.y):
                self.get_logger().warn('Outlier detected and discarded')
            else:
                # Add the current transformed position to the robot path
                self.robot_path.append((transformed_pose.position.x, transformed_pose.position.y))
                
                # Call the plotting function to visualize the odometry path
                self.plot_odometry_path()

    def transform_odometry_to_map(self, odom_pose):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return None  # Skip this callback if transform is not available

            # Look up the transformation from odom to map
            transform_stamped = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'odom',  # Source frame
                rclpy.time.Time()  # Get the latest available transform
            )

            # Transform the odometry pose from odom frame to map frame
            transformed_pose = do_transform_pose(odom_pose, transform_stamped)

            self.get_logger().info(f"Transforming odometry.")

            return transformed_pose

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform odometry to map frame: {ex}")
            return None

    def is_outlier(self, x, y):
        """Check if the new odometry point is an outlier based on distance."""
        if len(self.robot_path) == 0:
            return False  # No previous point to compare to
        
        # Get the last recorded point
        last_x, last_y = self.robot_path[-1]
        
        # Calculate the Euclidean distance between the last point and the new point
        distance = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)

        # If the distance is greater than the threshold, it's considered an outlier
        return distance > self.outlier_threshold

    def plot_odometry_path(self):
        """Plot only the robot's path (odometry data) without creating a new figure every time."""

        # Clear the current axis to avoid overplotting
        self.ax.clear()

        # Plot the robot's path (odometry data)
        if self.robot_path:
            x_path, y_path = zip(*self.robot_path)
            self.ax.plot(x_path, y_path, c='blue', label='Robot Path', marker='o')

        # Set titles, labels, and legend
        self.ax.set_title('Robot Path (Odometry)')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)
        self.ax.legend()

        # Redraw and pause for real-time update
        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
