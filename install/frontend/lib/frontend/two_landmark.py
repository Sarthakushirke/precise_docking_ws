#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Twist
import numpy as np
import gtsam
import matplotlib.patches as patches
import gtsam.utils.plot as gtsam_plot


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

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Noise models for the factors (adjust based on your data)
        self.odometry_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.1, np.radians(1.0)]))  # [x, y, theta]
        
        self.landmark_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.00001]))  # Noise model for landmark measurements

        # Create a factor graph
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()

        # Initialize odometry and command velocity data containers
        self.odom_data = None
        self.cmd_velocity = None

        # Initialize previous odometry data
        self.previous_odom_data = None

        # Track the pose number (start from pose 0)
        self.pose_num = 0

        # Set the number of poses after which to trigger optimization
        self.pose_threshold = 5

        # Lists to store odometry and optimized poses for plotting
        self.odometry_poses_x = []
        self.odometry_poses_y = []
        self.optimized_poses_x = []
        self.optimized_poses_y = []

        # Add this in the class __init__ method
        self.landmark_inserted = set()  # Track inserted landmark keys

        # Store odometry positions (robot path)
        self.robot_path = []

        # Set up a plot for live updating
        plt.ion()  # Turn on interactive mode for live updating
        self.fig, self.ax = plt.subplots()

        # Set a threshold for filtering outliers based on distance
        self.outlier_threshold = 1  # Set to a reasonable threshold (meters)

    def odom_callback(self, msg):
        # Extract position and orientation from odometry data
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom
        
        print("Odom pose with out transformation", odom_pose.pose)
        # Transform odometry pose to the map frame
        transformed_pose = self.transform_odometry_to_map(odom_pose.pose)
        print("")
        if transformed_pose:
            print("The x and y", transformed_pose.position.x, transformed_pose.position.y)
            # Check if the new point is an outlier based on distance
            if self.is_outlier(transformed_pose.position.x, transformed_pose.position.y):
                self.get_logger().warn('Outlier detected and discarded')
                self.robot_path.append((transformed_pose.position.x, transformed_pose.position.y))
            else:
                self.odom_data = transformed_pose
                self.process_data()
                # Add the current transformed position to the robot path
                self.robot_path.append((transformed_pose.position.x, transformed_pose.position.y))
                
                # Call the plotting function to visualize the odometry path
                self.plot_odometry_path()


    def cmd_vel_callback(self, msg):
        """Handles incoming cmd_vel data"""
        if msg is None:
            print("cmd_vel is None")
        else:
            self.cmd_velocity = msg
            self.process_data()

    def process_data(self):
        """Process odometry data and velocity"""
        if self.odom_data and self.cmd_velocity:
            if self.cmd_velocity.linear.x == 0 and self.cmd_velocity.angular.z == 0:
                return  # Skip processing if both linear and angular velocities are zero

            odom_pose_extract = self.extract_pose2_from_msg(self.odom_data)
            pose_key = gtsam.symbol('x', self.pose_num)

            # Add odometry pose to the list for plotting
            self.odometry_poses_x.append(odom_pose_extract.x())
            self.odometry_poses_y.append(odom_pose_extract.y())
            
            # Add factors to the graph
            if self.pose_num > 0:
                prev_pose_key = gtsam.symbol('x', self.pose_num - 1)
                prev_pose = self.initial_estimates.atPose2(prev_pose_key)
                odometry_factor = prev_pose.between(odom_pose_extract)
                self.graph.add(gtsam.BetweenFactorPose2(
                    prev_pose_key, pose_key, odometry_factor, self.odometry_noise_model))
            else:
                # Add prior factor for the first pose
                prior_noise = gtsam.noiseModel.Diagonal.Variances([0.5, 0.5, np.radians(1.0)])
                self.graph.add(gtsam.PriorFactorPose2(pose_key, odom_pose_extract, prior_noise))

            # Insert initial guess for this pose
            self.initial_estimates.insert(pose_key, odom_pose_extract)

            # Handle the two landmarks
            self.add_landmark_factors(odom_pose_extract, pose_key)

            # Increment pose count
            self.pose_num += 1
            print(f"Pose count: {self.pose_num}")

            # Trigger optimization after threshold
            if self.pose_num % self.pose_threshold == 0:
                print("We entered optimization")
                self.optimize_pose_graph()

    def add_landmark_factors(self, odom_pose_extract, pose_key):
        """Add landmark factors for the landmarks at (-5, 4) and (-3, 2)"""

        # Handle first landmark (-5, 4)
        landmark_pose_1 = gtsam.Point2(-5, 4)
        landmark_id_1 = 0
        landmark_key_1 = gtsam.symbol('L', landmark_id_1)

        if landmark_id_1 not in self.landmark_inserted:
            print(f"Adding Landmark Node: {landmark_key_1} -> (-5,4)")
            self.initial_estimates.insert(landmark_key_1, landmark_pose_1)
            self.landmark_inserted.add(landmark_id_1)
            landmark_prior_noise_1 = gtsam.noiseModel.Diagonal.Variances([0.1, 0.0001])
            self.graph.add(gtsam.PriorFactorPoint2(landmark_key_1, landmark_pose_1, landmark_prior_noise_1))

        bearing_1 = odom_pose_extract.bearing(landmark_pose_1)
        range_measurement_1 = odom_pose_extract.range(landmark_pose_1)
        self.graph.add(gtsam.BearingRangeFactor2D(
            pose_key, landmark_key_1, bearing_1, range_measurement_1, self.landmark_noise_model))

        # Handle second landmark (-3, 2)
        landmark_pose_2 = gtsam.Point2(-3, 2)
        landmark_id_2 = 1
        landmark_key_2 = gtsam.symbol('L', landmark_id_2)

        if landmark_id_2 not in self.landmark_inserted:
            print(f"Adding Landmark Node: {landmark_key_2} -> (-3,2)")
            self.initial_estimates.insert(landmark_key_2, landmark_pose_2)
            self.landmark_inserted.add(landmark_id_2)
            landmark_prior_noise_2 = gtsam.noiseModel.Diagonal.Variances([0.1, 0.0001])
            self.graph.add(gtsam.PriorFactorPoint2(landmark_key_2, landmark_pose_2, landmark_prior_noise_2))

        bearing_2 = odom_pose_extract.bearing(landmark_pose_2)
        range_measurement_2 = odom_pose_extract.range(landmark_pose_2)
        self.graph.add(gtsam.BearingRangeFactor2D(
            pose_key, landmark_key_2, bearing_2, range_measurement_2, self.landmark_noise_model))

    def optimize_pose_graph(self):
        """Optimize the factor graph using Levenberg-Marquardt"""
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimates)
        result = optimizer.optimize()

        # Compute marginals
        marginals = gtsam.Marginals(self.graph, result)

        # Clear previous optimized poses
        self.optimized_poses_x.clear()
        self.optimized_poses_y.clear()

        # Plot the entire trajectory
        for i in range(self.pose_num):
            pose_key = gtsam.symbol('x', i)
            optimized_pose = result.atPose2(pose_key)
            self.optimized_poses_x.append(optimized_pose.x())
            self.optimized_poses_y.append(optimized_pose.y())

            # Instead of manually plotting ellipses, use gtsam_plot to plot the pose and covariance
            # gtsam_plot.plot_pose2(0, optimized_pose, 1, marginals.marginalCovariance(pose_key))

            # Get the 2D covariance matrix for the current pose
            covariance = marginals.marginalCovariance(pose_key)
            cov_2d = covariance[:2, :2]  # Extract the (x, y) covariance matrix
            print("Here is the covariance", cov_2d)

            # Store the covariance for plotting
            mean = [optimized_pose.x(), optimized_pose.y()]

            # Eigenvalues and eigenvectors of the covariance matrix
            eigvals, eigvecs = np.linalg.eigh(cov_2d)

            # Get the angle of the ellipse
            angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))

            # Width and height of the ellipse (scaled by 1 standard deviation)
            n_std = 1.0
            width, height = 2 * n_std * np.sqrt(eigvals)

            # Plot the covariance ellipse
            ellipse = patches.Ellipse(mean, width, height, angle, edgecolor='black', facecolor='none', lw=2)
            self.ax.add_patch(ellipse)

            # Plot the mean point
            self.ax.scatter(mean[0], mean[1], c='red', label='Mean')

            # Set plot limits
            self.ax.set_xlim([mean[0] - 10, mean[0] + 10])
            self.ax.set_ylim([mean[1] - 10, mean[1] + 10])

            # Set equal aspect ratio
            self.ax.set_aspect('equal', 'box')

            self.ax.set_xlabel("X Position")
            self.ax.set_ylabel("Y Position")

            # Show the plot
            plt.grid(True)
            plt.legend()
            plt.draw()
            plt.pause(0.001)

        # Now plot the landmarks' covariances
        for landmark_id in self.landmark_inserted:
            landmark_key = gtsam.symbol('L', landmark_id)
            landmark_pose = result.atPoint2(landmark_key)

            # Plot the landmark using a simple scatter plot
            landmark_positions = {0: (-5, 4), 1: (-3, 2)}
            landmark_position = landmark_positions.get(landmark_id, (0, 0))

            plt.scatter(landmark_position[0], landmark_position[1], marker='s', color='green', label=f'Landmark {landmark_id}')

            # Get the covariance matrix for the landmark
            landmark_covariance = marginals.marginalCovariance(landmark_key)
            landmark_cov_2d = landmark_covariance[:2, :2]  # Only interested in x, y covariance
            print(f"The landmark covariance for Landmark {landmark_id}:", landmark_cov_2d)

            # Plot the covariance ellipse for the landmark
            mean = landmark_position

            # Eigenvalues and eigenvectors of the covariance matrix
            eigvals, eigvecs = np.linalg.eigh(landmark_cov_2d)

            # Get the angle of the ellipse
            angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))

            # Width and height of the ellipse (scaled by 1 standard deviation)
            n_std = 1.0
            width, height = 2 * n_std * np.sqrt(eigvals)

            # Plot the covariance ellipse
            ellipse = patches.Ellipse(mean, width, height, angle, edgecolor='black', facecolor='none', lw=2)
            self.ax.add_patch(ellipse)

            # Plot the mean point
            self.ax.scatter(mean[0], mean[1], c='red', label=f'Landmark {landmark_id} Mean')

        plt.draw()
        plt.pause(0.001)

    def plot_odometry_path(self):
        """Plot only the robot's path (odometry data) without creating a new figure every time."""

        # Clear the current axis to avoid overplotting
        self.ax.clear()

        # Plot the robot's path (odometry data)
        if self.robot_path:
            x_path, y_path = zip(*self.robot_path)
            self.ax.plot(x_path, y_path, c='blue', label='Robot Path', marker='o')

        # Plot the optimized path
        if self.optimized_poses_x and self.optimized_poses_y:
            self.ax.plot(self.optimized_poses_x, self.optimized_poses_y, c='red', label='Optimized Path', marker='x')

        # Plot the landmarks
        self.ax.scatter(-5, 4, c='green', label='Landmark (-5, 4)', marker='s')
        self.ax.scatter(-3, 2, c='green', label='Landmark (-3, 2)', marker='s')

        # Set titles, labels, and legend
        self.ax.set_title('Robot Path, Optimized Path, and Landmarks')
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

