#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException
import struct
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import os
import datetime

class PointCloudTransformer(Node):

    def __init__(self):
        super().__init__('pointcloud_transformer')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to current, previous, and transformed point clouds
        self.current_cloud_subscriber = self.create_subscription(
            PointCloud2,
            '/icp_data_cloud',  # Current point cloud topic
            self.current_pointcloud_callback,
            10
        )

        self.previous_cloud_subscriber = self.create_subscription(
            PointCloud2,
            '/icp_previous_cloud',  # Previous point cloud topic
            self.previous_pointcloud_callback,
            10
        )

        self.transformed_cloud_subscriber = self.create_subscription(
            PointCloud2,
            '/icp_transformed_cloud',  # Transformed point cloud topic
            self.transformed_pointcloud_callback,
            10
        )

        # Subscribe to odometry for robot path
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Variables to store current, previous, and transformed point clouds
        self.current_cloud = None
        self.previous_cloud = None
        self.transformed_cloud = None

        # Initialize an empty list for the lidar map 
        self.lidar_map = []

        # Store odometry positions (robot path)
        self.robot_path = []

        # Initialize an empty list for the global map (accumulated points)
        self.global_map = []

        # Flags to check if all point clouds have been received
        self.received_current = False
        self.received_previous = False
        self.received_transformed = False

        # Set up a plot for live updating
        plt.ion()  # Turn on interactive mode for live updating
        self.fig, self.ax = plt.subplots()

    def current_pointcloud_callback(self, msg):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('diff_drive/lidar_link', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return  # Skip this callback if transform is not available
            
            # Look up the transformation from diff_lidar to map
            transform_stamped = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'diff_drive/lidar_link',  # Source frame
                rclpy.time.Time()  # Get the latest available transform
            )
            self.current_cloud = self.transform_pointcloud_2d(msg,transform_stamped)
            self.received_current = True
            self.check_and_plot()

            # Accumulate the points into the global map
            self.global_map.extend(self.current_cloud.tolist())

            # Plot the updated global map
            # self.plot_global_map()

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform point cloud to map frame: {ex}")


    def previous_pointcloud_callback(self, msg):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('diff_drive/lidar_link', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return  # Skip this callback if transform is not available
            
            # Look up the transformation from diff_lidar to map
            transform_stamped = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'diff_drive/lidar_link',  # Source frame
                rclpy.time.Time()  # Get the latest available transform
            )
            self.previous_cloud = self.transform_pointcloud_2d(msg,transform_stamped)
            self.received_previous = True
            self.check_and_plot()

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform point cloud to map frame: {ex}")

    def transformed_pointcloud_callback(self, msg):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('diff_drive/lidar_link', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return  # Skip this callback if transform is not available
            
            # Look up the transformation from diff_lidar to map
            transform_stamped = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'diff_drive/lidar_link',  # Source frame
                rclpy.time.Time()  # Get the latest available transform
            )
            self.transformed_cloud = self.transform_pointcloud_2d(msg,transform_stamped)
            self.received_transformed = True
            self.check_and_plot()

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform point cloud to map frame: {ex}")
        

    def check_and_plot(self):
        # Check if all point clouds have been received
        if self.received_current and self.received_previous and self.received_transformed:
            # Once all clouds are received, plot the map
            self.plot_lidar_map()
            # Reset the flags
            self.received_current = False
            self.received_previous = False
            self.received_transformed = False

    def odom_callback(self, msg):
        # Extract position and orientation from odometry data
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom

        # Print the pose (position and orientation) from odometry
        self.print_pose(odom_pose.pose)

        # Transform odometry pose to the map frame
        transformed_pose = self.transform_odometry_to_map(odom_pose.pose)

        if transformed_pose:
            # Add the current transformed position to the robot path
            self.robot_path.append((transformed_pose.position.x, transformed_pose.position.y))

    def print_pose(self, pose):
        # Print position
        self.get_logger().info(f"Odometry Position in odom frame: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")

    def transform_odometry_to_map(self, odom_pose):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return  # Skip this callback if transform is not available

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

    # def transform_pointcloud_2d(self, cloud_msg):

    #     # Extract the point cloud data (x, y only for 2D)
    #     points = self.pointcloud2_to_xy(cloud_msg)


    #     return points


    def transform_pointcloud_2d(self, cloud_msg, transform_stamped):
        # Extract the transformation matrix from TransformStamped
        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation

        # Convert the quaternion to a 2D rotation matrix (yaw only)
        yaw = self.quaternion_to_yaw(rotation)

        # Create the transformation matrix (3x3 for 2D)
        T = np.eye(3)
        T[0:2, 0:2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
        T[0:2, 2] = [translation.x, translation.y]  # Set translation (x, y)

        # Extract the point cloud data (x, y only for 2D)
        points = self.pointcloud2_to_xy(cloud_msg)

        # Apply the transformation to each point
        points_hom = np.hstack([points, np.ones((points.shape[0], 1))])  # Convert to homogeneous coordinates
        transformed_points_hom = (T @ points_hom.T).T  # Apply the transformation

        # Extract transformed xy
        transformed_points = transformed_points_hom[:, 0:2]

        return transformed_points
    
    def quaternion_to_yaw(self, q):
        """Convert a quaternion into a yaw (2D rotation around the z-axis)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)


    def plot_lidar_map(self, output_dir="plots", base_filename="lidar_map"):
        """Plot the current point clouds and the robot's path, and save each plot with a unique name."""
        
        # Create the output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Create a new figure for each plot to avoid overwriting
        fig, ax = plt.subplots()

        # Plot current, previous, and transformed clouds
        if self.current_cloud is not None and len(self.current_cloud) > 0:
            ax.scatter(self.current_cloud[:, 0], self.current_cloud[:, 1], s=2, c='blue', marker='o', label='Current Cloud')
        else:
            self.get_logger().info('Current cloud is empty.')

        
        if self.previous_cloud is not None:
            ax.scatter(self.previous_cloud[:, 0], self.previous_cloud[:, 1], s=2, c='green', label='Previous Cloud')


        if self.transformed_cloud is not None and len(self.transformed_cloud) > 0:
            ax.scatter(self.transformed_cloud[:, 0], self.transformed_cloud[:, 1], s=2, c='red', marker='*', label='Transformed Cloud')

        # Plot the robot's path (odometry data)
        if self.robot_path:
            x_path, y_path = zip(*self.robot_path)
            ax.plot(x_path, y_path, c='red', label='Robot Path', marker='x')

        # Set titles, labels, and legend
        ax.set_title('2D Global Map with Robot Path and Clouds')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.grid(True)
        ax.legend()

        # Create a unique filename based on timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{base_filename}_{timestamp}.png"
        save_path = os.path.join(output_dir, filename)

        # Save the plot
        fig.savefig(save_path)
        self.get_logger().info(f"Plot saved to {save_path}")

        # Close the figure to free up memory and avoid overlap
        plt.close(fig)



        # Redraw the plot for live updates
        # plt.draw()
        # plt.pause(0.05)
        


    def pointcloud2_to_xy(self, cloud_msg):
        """Extract the (x, y) points from a PointCloud2 message."""
        points = []
        for p in self.read_points(cloud_msg, field_names=("x", "y"), skip_nans=True):
            points.append([p[0], p[1]])  # Only (x, y) for 2D
        return np.array(points)

    def read_points(self, cloud, field_names=None, skip_nans=False):
        """Yield namedtuples for each point in a PointCloud2."""
        fmt = "<ff"  # Only 2 floats: x, y
        width, height = cloud.width, cloud.height
        point_step, row_step = cloud.point_step, cloud.row_step

        # Iterate through the point cloud data
        for v in range(height):
            for u in range(width):
                point_offset = row_step * v + u * point_step
                if skip_nans:
                    x, y = struct.unpack_from(fmt, cloud.data, offset=point_offset)
                    if not (np.isnan(x) or np.isnan(y)):
                        yield (x, y)
                else:
                    yield struct.unpack_from(fmt, cloud.data, offset=point_offset)

    def plot_global_map(self):
        """Plot the accumulated global map and robot's path."""
        self.ax.clear()

        # Plot the global map (points from the point cloud)
        if self.global_map:
            x, y = zip(*self.global_map)
            self.ax.scatter(x, y, s=1, c='black', label='Global Map')

        # Plot the robot's path (odometry data)
        if self.robot_path:
            x_path, y_path = zip(*self.robot_path)
            self.ax.plot(x_path, y_path, c='red', label='Robot Path', marker='x')

        self.ax.set_title('2D Global Map with Robot Path')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.grid(True)
        self.ax.legend()

        # Redraw the plot
        plt.draw()
        plt.pause(0.01)


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
