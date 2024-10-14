#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import gtsam
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Twist
from matplotlib.patches import Ellipse


class PoseGraphOptimization(Node):
    def __init__(self):
        super().__init__('pose_graph_optimization')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions for odometry and velocity data
        self.odometry_sub = self.create_subscription(
            Odometry, '/odom', self.odometry_callback, 10)
                
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Noise models for the factors (adjust based on your data)
        self.odometry_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.1, np.radians(5.0)]))  # [x, y, theta]
        
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
        self.pose_threshold = 10

        # Lists to store odometry and optimized poses for plotting
        self.odometry_poses_x = []
        self.odometry_poses_y = []
        self.optimized_poses_x = []
        self.optimized_poses_y = []

        # Initialize the plot
        plt.ion()
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

    def odometry_callback(self, msg):
        """Handles incoming odometry data"""
        # Extract position and orientation from odometry data
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom

        # Transform odometry pose to the lidar frame
        transformed_pose = self.transform_odometry_to_lidar(odom_pose.pose)
        if transformed_pose is None:
            return  # Skip processing if transformation failed
        
        self.odom_data = transformed_pose
        self.process_data()

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
            # Check if cmd_velocity is non-zero
            if self.cmd_velocity.linear.x == 0 and self.cmd_velocity.angular.z == 0:
                return  # Skip processing if both linear and angular velocities are zero

            odom_pose = self.extract_pose2_from_msg(self.odom_data)
            pose_key = gtsam.symbol('x', self.pose_num)

            # Add odometry pose to the list for plotting
            self.odometry_poses_x.append(odom_pose.x())
            self.odometry_poses_y.append(odom_pose.y())
            
            # Add factors to the graph
            if self.pose_num > 0:
                # Add odometry factor
                prev_pose_key = gtsam.symbol('x', self.pose_num - 1)
                prev_pose = self.initial_estimates.atPose2(prev_pose_key)
                odometry_factor = prev_pose.between(odom_pose)

                self.graph.add(gtsam.BetweenFactorPose2(
                    prev_pose_key, pose_key, odometry_factor, self.odometry_noise_model))
            else:
                # Add prior factor for the first pose
                prior_noise = gtsam.noiseModel.Diagonal.Variances([1e-6, 1e-6, 1e-8])
                self.graph.add(gtsam.PriorFactorPose2(pose_key, odom_pose, prior_noise))

            # Insert initial guess for this pose
            self.initial_estimates.insert(pose_key, odom_pose)

            # Increment pose count
            self.pose_num += 1
            print(f"Pose count: {self.pose_num}")

            # Trigger optimization after threshold
            if self.pose_num % self.pose_threshold == 0:
                print("We entered optimization")
                self.optimize_pose_graph()

    def transform_odometry_to_lidar(self, odom_pose):
        try:
            if not self.tf_buffer.can_transform('odom', 'diff_drive/lidar_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                return  # Skip if transformation not available

            transform_stamped = self.tf_buffer.lookup_transform(
                'diff_drive/lidar_link', 'odom', rclpy.time.Time())
            return do_transform_pose(odom_pose, transform_stamped)
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform odometry to lidar frame: {ex}")
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

        
        # Clear the current axis to ensure no overlap
        self.ax.clear()

        # Plot the odometry and optimized trajectories
        self.ax.cla()
        # print("Odometry values", self.odometry_poses_x, self.odometry_poses_y, )
        self.ax.plot(self.odometry_poses_x, self.odometry_poses_y, 'r--', label='Odometry Trajectory')
        print("Optimized values",self.optimized_poses_x,self.optimized_poses_y)
        self.ax.plot(self.optimized_poses_x, self.optimized_poses_y, 'b-', label='Optimized Trajectory')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Robot Trajectory')
        self.ax.grid(True)
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)




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
