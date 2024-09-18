#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_py as tf2
import struct
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

class PointCloudTransformer(Node):

    def __init__(self):
        super().__init__('pointcloud_transformer')

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to the transformed point cloud
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            'transformed_cloud',  # Topic name of the transformed cloud
            self.pointcloud_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Initialize an empty list for the global map (accumulated points)
        self.global_map = []

        # Store odometry positions (robot path)
        self.robot_path = []

        # Set up a plot for live updating
        plt.ion()  # Turn on interactive mode for live updating
        self.fig, self.ax = plt.subplots()

    def pointcloud_callback(self, msg):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('diff_drive/lidar_link', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return  # Skip this callback if transform is not available

            # Look up the transformation from odom to map
            transform_stamped = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'diff_drive/lidar_link',  # Source frame
                rclpy.time.Time()  # Get the latest available transform
            )


            points_before = self.pointcloud2_to_xy(msg)

            self.get_logger().info(f"Original points in lidar frame: {points_before}")

            self.get_logger().info(f"Original points counts: {len(points_before)}")

            # Transform the point cloud data (2D version)
            transformed_cloud = self.transform_pointcloud_2d(msg, transform_stamped)

            self.get_logger().info(f"Transformed Points in map frame: {transformed_cloud}")

            self.get_logger().info(f"Transformed Points Count: {len(transformed_cloud)}")

            # Accumulate the points into the global map
            self.global_map.extend(transformed_cloud.tolist())

            # Plot the updated global map
            self.plot_global_map()

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not transform point cloud to map frame: {ex}")


    def odom_callback(self, msg):
        # Extract position and orientation from odometry data
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom


        # Print the pose (position and orientation) from odometry
        self.print_pose(odom_pose.pose)

        # Transform odometry pose to the map frame
        transformed_pose = self.transform_odometry_to_map(odom_pose.pose)

        # if transformed_pose:
        #     # Print the current transformed pose
        #     self.print_pose(transformed_pose)
   
            # Add the current transformed position to the lists
            # self.x_positions.append(transformed_pose.position.x)
            # self.y_positions.append(transformed_pose.position.y)

    
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

            # Add the current transformed position to the robot path
            self.robot_path.append((transformed_pose.position.x, transformed_pose.position.y))
            
            self.get_logger().info(f"transforming")

            return transformed_pose

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform odometry to map frame: {ex}")

            return None


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

    # def plot_global_map(self):
    #     """Plot the accumulated global map."""
    #     self.ax.clear()

    #     # Unzip the accumulated global map into x and y lists
    #     if self.global_map:
    #         x, y = zip(*self.global_map)

    #         # Plot the global map
    #         self.ax.scatter(x, y, s=1, c='black', label='Global Map')
    #         self.ax.set_title('2D Global Map')
    #         self.ax.set_xlabel('X')
    #         self.ax.set_ylabel('Y')
    #         self.ax.grid(True)

    #         # Redraw the plot
    #         plt.draw()
    #         plt.pause(0.01)


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

    def quaternion_to_yaw(self, q):
        """Convert a quaternion into a yaw (2D rotation around the z-axis)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

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