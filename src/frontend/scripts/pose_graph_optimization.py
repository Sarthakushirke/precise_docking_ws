#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import gtsam
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, Pose
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist

class PoseGraphOptimization(Node):
    def __init__(self):
        super().__init__('pose_graph_optimization')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions for odometry, velocity, and ArUco marker data
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.marker_sub = self.create_subscription(PoseArray, '/aruco_poses_in_map', self.aruco_callback, 10)

        # Noise models for the factors
        self.odometry_noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, np.radians(5.0)]))  # [x, y, theta]
        self.landmark_noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))  # Noise model for landmark measurements

        # Create a factor graph
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()

        # Initialize data containers
        self.odom_data = None
        self.cmd_velocity = None
        self.previous_odom_data = None
        self.pose_num = 0
        self.landmark_inserted = set()  # Track inserted landmark keys
        self.pose_threshold = 10  # Optimization trigger

        # Lists to store odometry and optimized poses for plotting
        self.odometry_poses_x = []
        self.odometry_poses_y = []
        self.optimized_poses_x = []
        self.optimized_poses_y = []

        # Initialize the plot
        plt.ion()
        self.figure, self.ax = plt.subplots()

    def odometry_callback(self, msg):
        """Handles incoming odometry data"""
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose

        self.odom_data = self.extract_pose2_from_msg(odom_pose.pose)
        self.process_data()

    def cmd_vel_callback(self, msg):
        """Handles incoming cmd_vel data"""
        self.cmd_velocity = msg

        print("In the command velocity", self.cmd_velocity)
        self.process_data()

    def aruco_callback(self, msg):
        """Handles incoming ArUco markers in map frame"""
        for pose in msg.poses:
            yaw = self.quaternion_to_yaw(pose.orientation)  # Extract yaw (θ)
            z_distance = pose.position.z*10  # Distance to marker

            print("z_distance",z_distance)

            # Compute global landmark position using robot's current pose
            if self.odom_data:
                landmark_x = self.odom_data.x() + z_distance * np.cos(yaw)
                landmark_y = self.odom_data.y() + z_distance * np.sin(yaw)

                print("Odom data", self.odom_data)

                print("Landmark data",landmark_x, landmark_y)

                self.add_landmark(landmark_x, landmark_y)

    def process_data(self):
        """Process odometry data and velocity"""
        if not (self.odom_data and self.cmd_velocity):

            print("1")
            return
        
        if self.cmd_velocity.linear.x == 0 and self.cmd_velocity.angular.z == 0:
            print("2")
            return  # Skip if the robot is stationary

        # Ensure new pose is different from previous pose
        if self.previous_odom_data and np.isclose(self.previous_odom_data.x(), self.odom_data.x(), atol=1e-3) and np.isclose(self.previous_odom_data.y(), self.odom_data.y(), atol=1e-3):
            print("3")
            return  # Skip duplicate poses


        
        pose_key = gtsam.symbol('x', self.pose_num)

        self.odometry_poses_x.append(self.odom_data.x())
        self.odometry_poses_y.append(self.odom_data.y())

        if self.pose_num > 0:
            prev_pose_key = gtsam.symbol('x', self.pose_num - 1)
            prev_pose = self.initial_estimates.atPose2(prev_pose_key)
            odometry_factor = prev_pose.between(self.odom_data)
            self.graph.add(gtsam.BetweenFactorPose2(prev_pose_key, pose_key, odometry_factor, self.odometry_noise_model))
        else:
            prior_noise = gtsam.noiseModel.Diagonal.Variances([1e-6, 1e-6, 1e-8])
            self.graph.add(gtsam.PriorFactorPose2(pose_key, self.odom_data, prior_noise))

        self.initial_estimates.insert(pose_key, self.odom_data)
        self.pose_num += 1
        self.previous_odom_data = self.odom_data

        print("self.pose_num in process data",self.pose_num)

        if self.pose_num % self.pose_threshold == 0:
            self.optimize_pose_graph()

    def add_landmark(self, x, y):
        """Adds an ArUco marker landmark to the pose graph"""
        if self.pose_num == 0:
            self.get_logger().warn("Skipping landmark addition: No pose available yet.")
            return  # Skip adding a landmark if no pose is available

        landmark_pose = gtsam.Point2(x, y)
        landmark_id = len(self.landmark_inserted)
        landmark_key = gtsam.symbol('L', landmark_id)

        if landmark_id not in self.landmark_inserted:
            self.initial_estimates.insert(landmark_key, landmark_pose)
            self.landmark_inserted.add(landmark_id)

            landmark_prior_noise = gtsam.noiseModel.Diagonal.Variances([1e-6, 1e-6])
            self.graph.add(gtsam.PriorFactorPoint2(landmark_key, landmark_pose, landmark_prior_noise))

        # Add measurement factor between current pose and landmark
        pose_key = gtsam.symbol('x', max(0, self.pose_num - 1))  # Ensure non-negative pose index
        bearing = self.odom_data.bearing(landmark_pose)
        range_measurement = self.odom_data.range(landmark_pose)

        self.graph.add(gtsam.BearingRangeFactor2D(pose_key, landmark_key, bearing, range_measurement, self.landmark_noise_model))


    def optimize_pose_graph(self):
        """Optimize the factor graph using Levenberg-Marquardt"""
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimates)
        result = optimizer.optimize()

        # Clear previous optimized poses
        self.optimized_poses_x.clear()
        self.optimized_poses_y.clear()

        for i in range(self.pose_num):
            pose_key = gtsam.symbol('x', i)
            optimized_pose = result.atPose2(pose_key)
            self.optimized_poses_x.append(optimized_pose.x())
            self.optimized_poses_y.append(optimized_pose.y())

                    # Store last optimized pose
            if i == self.pose_num - 1:
                last_pose = optimized_pose

        print("Last optimized pose:", last_pose)

        self.plot_trajectories()

    def plot_trajectories(self):
        """Plot odometry, optimized trajectories, and landmark positions."""
        self.ax.cla()  # Clear previous plot

        # Plot odometry trajectory
        self.ax.plot(self.odometry_poses_x, self.odometry_poses_y, 'r--', label='Odometry Trajectory')

        # Plot optimized trajectory
        self.ax.plot(self.optimized_poses_x, self.optimized_poses_y, 'b-', label='Optimized Trajectory')

        # Plot landmark positions
        for landmark_id in self.landmark_inserted:
            landmark_key = gtsam.symbol('L', landmark_id)
            optimized_landmark = self.initial_estimates.atPoint2(landmark_key)

            # Plot landmark position as a green dot
            self.ax.scatter(optimized_landmark[0], optimized_landmark[1], color='g', marker='o', label='Landmark' if landmark_id == 0 else "")

            # Annotate landmark ID on the plot
            self.ax.text(optimized_landmark[0], optimized_landmark[1], f"L{landmark_id}", fontsize=10, ha='right')

        # Labels, title, and legend
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Robot Trajectory & Landmarks')
        self.ax.grid(True)
        self.ax.legend()

        # Draw and pause for real-time update
        plt.draw()
        plt.pause(0.001)


    def extract_pose2_from_msg(self, pose_msg):
        """Convert geometry_msgs/Pose to gtsam.Pose2 (x, y, yaw)"""
        x = pose_msg.position.x
        y = pose_msg.position.y
        yaw = self.quaternion_to_yaw(pose_msg.orientation)
        return gtsam.Pose2(x, y, yaw)

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle for 2D"""
        rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        _, _, yaw = rotation.as_euler('xyz', degrees=False)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphOptimization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
