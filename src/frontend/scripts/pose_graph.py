#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import gtsam
import numpy as np
import matplotlib.pyplot as plt  # Import Matplotlib
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose


class PoseGraphOptimization(Node):
    def __init__(self):
        super().__init__('pose_graph_optimization')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions for odometry and ICP data
        self.odometry_sub = self.create_subscription(
            Odometry, '/odom', self.odometry_callback, 10)

        self.icp_sub = self.create_subscription(
            TransformStamped, '/icp_transform', self.icp_callback, 10)
        
        # Noise models for the factors (adjust based on your data)
        self.odometry_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.1, np.radians(5.0)]))  # [x, y, theta]
        self.icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.05, 0.05, np.radians(2.0)]))  # Adjust based on ICP accuracy
        
        # Create a factor graph
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()

        # Initialize the odometry and ICP data containers
        self.odom_data = None
        self.icp_data = None

        # Initialize previous odometry data
        self.previous_odom_data = None

        
        
        # Track the pose number (start from pose 0)
        self.pose_num = 0
        
        # Set the number of poses after which to trigger optimization
        self.pose_threshold = 10

        # Lists to store odometry and optimized poses for plotting
        self.odometry_poses_x = []
        self.odometry_poses_y = []
        self.optimized_poses_x = []
        self.optimized_poses_y = []

        # Initialize the plot
        plt.ion()  # Turn on interactive mode
        self.figure, self.ax = plt.subplots()
        # Line for odometry trajectory
        self.line_odom, = self.ax.plot([], [], 'r--', label='Odometry Trajectory')
        # Line for optimized trajectory
        self.line_opt, = self.ax.plot([], [], 'b-', label='Optimized Trajectory')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Robot Trajectory')
        self.ax.grid(True)
        self.ax.legend()
    
    # def odometry_callback(self, msg):
    #     """Handles incoming odometry data"""
    #     # Extract position and orientation from odometry data
    #     odom_pose = PoseStamped()
    #     odom_pose.header = msg.header
    #     odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom


    #     # Print the pose (position and orientation) from odometry
    #     # self.print_pose(odom_pose.pose)

    #     # Transform odometry pose to the map frame
    #     transformed_pose = self.transform_odometry_to_lidar(odom_pose.pose)
    #     # self.odom_data = msg.pose.pose
    #     self.odom_data = transformed_pose
    #     self.process_data()

    def odometry_callback(self, msg):
        """Handles incoming odometry data"""
        # Extract position and orientation from odometry data
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom

        # Transform odometry pose to the lidar frame
        transformed_pose = self.transform_odometry_to_lidar(odom_pose.pose)
        if transformed_pose is None:
            self.get_logger().warn('Failed to transform odometry pose to lidar frame.')
            return  # Skip processing if transformation failed

        # Compare transformed_pose with previous odometry data
        if self.previous_odom_data is not None:
            check = self.is_pose_changed(self.previous_odom_data, transformed_pose)
            print(check)
            if self.is_pose_changed(self.previous_odom_data, transformed_pose):
                # Odometry data has changed
                self.odom_data = transformed_pose
                self.previous_odom_data = transformed_pose
                self.process_data()
            else:
                # Odometry data has not changed significantly
                pass  # Do nothing
        else:
            # No previous odometry data, so store current and process
            self.odom_data = transformed_pose
            self.previous_odom_data = transformed_pose
            # self.process_data()

    def is_pose_changed(self, previous_pose, current_pose):

        if previous_pose.position == current_pose.position or abs(previous_pose.position.x - current_pose.position.x) > 1:
            return False
        else:
            return True

    def icp_callback(self, msg):
        """Handles incoming ICP transformation data"""
        self.icp_data = msg
        self.process_data()

    def process_data(self):
        """Process odometry and ICP data when both are available"""
        if self.odom_data and self.icp_data:
            # Extract odometry pose (convert to 2D pose)

            odom_pose = self.extract_pose2_from_msg(self.odom_data)
            
            # Extract ICP relative pose
            icp_relative_pose = self.extract_pose2_from_transform(self.icp_data.transform)
            
            # Add odometry pose to the list for plotting
            self.odometry_poses_x.append(odom_pose.x())
            self.odometry_poses_y.append(odom_pose.y())
            
            # Add factors to the graph
            if self.pose_num > 0:
                # Add odometry factor
                self.graph.add(gtsam.BetweenFactorPose2(
                    self.pose_num - 1, self.pose_num, odom_pose.between(odom_pose), self.odometry_noise_model))
                # Add ICP factor
                self.graph.add(gtsam.BetweenFactorPose2(
                    self.pose_num - 1, self.pose_num, icp_relative_pose, self.icp_noise_model))
            else:
                # For the first pose, add a prior factor to fix the initial pose
                prior_noise = gtsam.noiseModel.Diagonal.Variances([1e-6, 1e-6, 1e-8])
                self.graph.add(gtsam.PriorFactorPose2(self.pose_num, odom_pose, prior_noise))

            # Insert initial guess for this pose
            self.initial_estimates.insert(self.pose_num, odom_pose)

            # Increment pose count
            self.pose_num += 1
            print(self.pose_num)

            # Reset the odometry and ICP data after processing
            self.odom_data = None
            self.icp_data = None

            # Trigger optimization after threshold
            if self.pose_num >= self.pose_threshold:
                self.optimize_pose_graph()



    def transform_odometry_to_lidar(self, odom_pose):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('odom', 'diff_drive/lidar_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return  # Skip this callback if transform is not available

            # Look up the transformation from odom to map
            transform_stamped = self.tf_buffer.lookup_transform(
                'diff_drive/lidar_link',  # Target frame
                'odom',  # Source frame
                rclpy.time.Time()  # Get the latest available transform
            )

            # Transform the odometry pose from odom frame to map frame
            transformed_pose = do_transform_pose(odom_pose, transform_stamped)


            return transformed_pose

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform odometry to map frame: {ex}")

            return None
    
    def extract_pose2_from_msg(self, pose_msg):
        """Convert geometry_msgs/Pose to gtsam.Pose2 (x, y, yaw)"""
        x = pose_msg.position.x
        y = pose_msg.position.y
        # Convert quaternion to yaw (only yaw for 2D case)
        yaw = self.quaternion_to_yaw(pose_msg.orientation)
        return gtsam.Pose2(x, y, yaw)

    def extract_pose2_from_transform(self, transform):
        """Convert geometry_msgs/Transform to gtsam.Pose2 (x, y, yaw)"""
        x = transform.translation.x
        y = transform.translation.y
        # Convert quaternion to yaw
        yaw = self.quaternion_to_yaw(transform.rotation)
        return gtsam.Pose2(x, y, yaw)

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle for 2D"""
        import math
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def optimize_pose_graph(self):
        """Optimize the factor graph using Levenberg-Marquardt"""
        self.get_logger().info("Optimizing pose graph...")
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimates)
        result = optimizer.optimize()

        # Clear previous optimized poses
        self.optimized_poses_x.clear()
        self.optimized_poses_y.clear()

        # Extract optimized poses
        for i in range(self.pose_num):
            optimized_pose = result.atPose2(i)
            self.get_logger().info(
                f'Optimized Pose {i}: x={optimized_pose.x()}, y={optimized_pose.y()}, theta={optimized_pose.theta()}')
            self.optimized_poses_x.append(optimized_pose.x())
            self.optimized_poses_y.append(optimized_pose.y())

        # Plot the odometry and optimized trajectories
        # self.update_plot()

        # Reset the graph and initial estimates for the next batch of poses
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.pose_num = 0  # Reset pose count for next optimization

        # Clear odometry poses for next batch (optional)
        self.odometry_poses_x.clear()
        self.odometry_poses_y.clear()

    def update_plot(self):
        """Update the plot with odometry and optimized trajectories"""
        # Update odometry trajectory
        self.line_odom.set_xdata(self.odometry_poses_x)
        self.line_odom.set_ydata(self.odometry_poses_y)
        # Update optimized trajectory
        self.line_opt.set_xdata(self.optimized_poses_x)
        self.line_opt.set_ydata(self.optimized_poses_y)
        # Adjust plot limits
        self.ax.relim()
        self.ax.autoscale_view()
        # Redraw the plot
        plt.draw()
        plt.pause(0.001)  # Small pause to update the plot

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphOptimization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Pose graph optimization completed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
