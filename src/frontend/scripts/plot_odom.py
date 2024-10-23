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
        
        # self.marker_sub = self.create_subscription(
        #     PointStamped, '/marker_in_robot_frame', self.marker_callback, 10)  # Subscribe to ArUco marker data
        
        
        # Noise models for the factors (adjust based on your data)
        self.odometry_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.1, np.radians(1.0)]))  # [x, y, theta]
        
        self.landmark_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.00001]))  # Noise model for landmark measurements

        # self.landmark_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
        #     np.array([0.01, 0.01, np.radians(5.0)]))  # Now includes orientation noise [x, y, theta]

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
        self.landmark_x = []
        self.landmark_y = []
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
        
        print("Odom pose with out transformation",odom_pose.pose )
        # Transform odometry pose to the map frame
        transformed_pose = self.transform_odometry_to_map(odom_pose.pose)
        print("")
        if transformed_pose:
            print("The x and y",transformed_pose.position.x, transformed_pose.position.y)
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

    # def marker_callback(self, msg):
    #     """Handles incoming ArUco marker pose data"""
    #     self.marker_data = msg.point  # Store the marker's pose
    #     # Add the landmark position to the list for plotting
    #     self.landmark_x.append(self.marker_data.x)
    #     self.landmark_y.append(self.marker_data.y)

    #     self.process_data()


    def process_data(self):
        """Process odometry data and velocity"""
        if self.odom_data and self.cmd_velocity :
            # and self.marker_data
            # Check if cmd_velocity is non-zero
            if self.cmd_velocity.linear.x == 0 and self.cmd_velocity.angular.z == 0:
                return  # Skip processing if both linear and angular velocities are zero

            odom_pose_extract = self.extract_pose2_from_msg(self.odom_data)
            pose_key = gtsam.symbol('x', self.pose_num)

            # Add odometry pose to the list for plotting
            self.odometry_poses_x.append(odom_pose_extract.x())
            self.odometry_poses_y.append(odom_pose_extract.y())
            
            # Add factors to the graph
            if self.pose_num > 0:
                # Add odometry factor
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

            # Handle landmark observations
            # if self.marker_data is not None:
                # landmark_pose = gtsam.Point2(self.marker_data.x, self.marker_data.y)
            landmark_pose = gtsam.Point2(-5, 4)
            # landmark_pose = gtsam.Pose2(-5, 4, np.radians(90))  # Example: Landmark at (-5, 4) with 90° orientation
            landmark_id = 0  # Use the actual ID from the ArUco marker
            landmark_key = gtsam.symbol('L', landmark_id)  # Use symbol for landmark key

            # If landmark has not been inserted yet
            if landmark_id not in self.landmark_inserted:
                # print(f"Adding Landmark Node: {landmark_key} -> ({self.marker_data.x}, {self.marker_data.y})")
                # landmark_pose = gtsam.Point2(self.marker_data.x, self.marker_data.y)
                print(f"Adding Landmark Node: {landmark_key} -> (-5,4)")
                landmark_pose = gtsam.Point2(-5, 4)
                # landmark_pose = gtsam.Pose2(-5, 4, np.radians(90)) 
                self.initial_estimates.insert(landmark_key, landmark_pose)
                self.landmark_inserted.add(landmark_id)

                # **Add a prior factor on the landmark with very low covariance**
                landmark_prior_noise = gtsam.noiseModel.Diagonal.Variances([0.1, 0.0001])  # Very small variances
                print(f"Adding Prior Factor for Landmark: {landmark_key}")
                self.graph.add(gtsam.PriorFactorPoint2(landmark_key, landmark_pose, landmark_prior_noise))

                # landmark_prior_noise = gtsam.noiseModel.Diagonal.Variances([1e-6, 1e-6, 1e-8])
                # self.graph.add(gtsam.PriorFactorPose2(landmark_key, landmark_pose, landmark_prior_noise))
                # self.get_logger().info(f"Added prior factor for landmark {landmark_id}")

            
            # Add measurement factor between current pose and landmark
            bearing = odom_pose_extract.bearing(landmark_pose)
            print("Bearing ", bearing)
            range_measurement = odom_pose_extract.range(landmark_pose)
            print("Range measurement", range_measurement)
            print(f"Adding Landmark Observation Factor: Between {pose_key} and {landmark_key}")
            self.graph.add(gtsam.BearingRangeFactor2D(
                pose_key, landmark_key, bearing, range_measurement, self.landmark_noise_model))

            # relative_pose = odom_pose_extract.between(landmark_pose)

            # print("Relative pose", relative_pose)

            # print(f"Adding Landmark Observation Factor (with Orientation): Between {pose_key} and {landmark_key}")
            # self.graph.add(gtsam.BetweenFactorPose2(
            #     pose_key, landmark_key, relative_pose, self.landmark_noise_model))


            # Increment pose count
            self.pose_num += 1
            print(f"Pose count: {self.pose_num}")

            # Trigger optimization after threshold
            if self.pose_num % self.pose_threshold == 0:
                print("We entered optimization")
                self.optimize_pose_graph()

    

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
        
    def extract_pose2_from_msg(self, pose_msg):
        """Convert geometry_msgs/Pose to gtsam.Pose2 (x, y, yaw)"""
        x = pose_msg.position.x
        y = pose_msg.position.y
        yaw = self.quaternion_to_yaw(pose_msg.orientation)
        return gtsam.Pose2(x, y, yaw)
    
    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle for 2D"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def is_outlier(self, x, y):
        """Check if the new odometry point is an outlier based on distance."""
        if len(self.robot_path) == 0:
            return False  # No previous point to compare to
        
        # Get the last recorded point
        last_x, last_y = self.robot_path[-1]
        print("Last x and y", last_x, last_y)
        print("Current x and y",x,y)
        
        # Calculate the Euclidean distance between the last point and the new point
        distance = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)

        # If the distance is greater than the threshold, it's considered an outlier
        return distance > self.outlier_threshold
    
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
            # self.plot_covariance_ellipse(self.ax, mean, cov_2d, n_std=2, edgecolor='purple', alpha=0.5)

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

            # Add title and labels
            # self.ax.set_title(f"Covariance Ellipse at {mean}\nWidth: {width:.3f}, Height: {height:.3f}, Angle: {angle:.2f}°")
            self.ax.set_xlabel("X Position")
            self.ax.set_ylabel("Y Position")

            # Show the plot
            plt.grid(True)
            plt.legend()
            # plt.show()
            # Redraw and pause for real-time update
            plt.draw()
            plt.pause(0.001)

        #Now plot the landmarks' covariances
        for landmark_id in self.landmark_inserted:
            landmark_key = gtsam.symbol('L', landmark_id)
            landmark_pose = result.atPoint2(landmark_key)

            # Plot the landmark using a simple scatter plot
            plt.scatter(-3, 2, marker='s', color='green', label=f'Landmark {landmark_id}')
            
        #     # Optionally, plot the landmark's covariance using gtsam_plot
        #     gtsam_plot.plot_point2(landmark_key, landmark_pose, marginals.marginalCovariance(landmark_key))

            # landmark_pose = result.atPose2(landmark_key)

            # Get the covariance matrix for the landmark
            landmark_covariance = marginals.marginalCovariance(landmark_key)
            landmark_cov_2d = landmark_covariance[:2, :2]  # Only interested in x, y covariance
            print("The landmark covariance", landmark_cov_2d)

            #Plot the covariance ellipse for the landmark
            landmark_mean = [-3, 2]

                        # Eigenvalues and eigenvectors of the covariance matrix
            eigvals, eigvecs = np.linalg.eigh(landmark_cov_2d)

            # Get the angle of the ellipse
            angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))

            # Width and height of the ellipse (scaled by 1 standard deviation)
            n_std = 1.0
            width, height = 2 * n_std * np.sqrt(eigvals)

            # Plot the covariance ellipse
            ellipse = patches.Ellipse(landmark_mean, width, height, angle, edgecolor='black', facecolor='none', lw=2)
            self.ax.add_patch(ellipse)

            # Plot the mean point
            self.ax.scatter(landmark_mean[0], landmark_mean[1], c='red', label='Mean')

            # # Set plot limits
            # self.ax.set_xlim([mean[0] - 10, mean[0] + 10])
            # self.ax.set_ylim([mean[1] - 10, mean[1] + 10])

            # Set equal aspect ratio
            self.ax.set_aspect('equal', 'box')

            # Add title and labels
            # self.ax.set_title(f"Covariance Ellipse at {mean}\nWidth: {width:.3f}, Height: {height:.3f}, Angle: {angle:.2f}°")
            self.ax.set_xlabel("X Position")
            self.ax.set_ylabel("Y Position")

            # Show the plot
            plt.grid(True)
            plt.legend()
            # plt.show()
            # Redraw and pause for real-time update
            plt.draw()
            plt.pause(0.001)

        # plt.scatter(-5, 4, marker='s', color='green', label=f'Landmark')

        # plt.grid(True)
        # plt.draw()  # Update the figure
        # plt.pause(0.001)  # Pause for a moment to update the plot


        
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
        # if self.landmark_x and self.landmark_y:
        self.ax.scatter(-3, 2, c='green', label='Landmarks', marker='s')


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
