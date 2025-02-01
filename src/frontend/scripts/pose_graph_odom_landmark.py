#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import gtsam
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped,PointStamped
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
        
        self.marker_sub = self.create_subscription(
            PointStamped, '/marker_in_robot_frame', self.marker_callback, 10)  # Subscribe to ArUco marker data

        # Noise models for the factors (adjust based on your data)
        self.odometry_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.1, np.radians(5.0)]))  # [x, y, theta]
        
        self.landmark_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.1]))  # Noise model for landmark measurements
        
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

        self.marker_data = None

        # Track which landmarks have already been inserted
        self.landmark_inserted = set()  # Track inserted landmark keys

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

    def marker_callback(self, msg):
        """Handles incoming ArUco marker pose data"""
        self.marker_data = msg.point  # Store the marker's pose
        print("This is the data",msg.point)

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

            
            # Check if previous odometry data exists and compare it with the current odometry
            if self.previous_odom_data is not None:
                # Compare x, y, and theta values to avoid adding duplicate poses
                if (np.isclose(self.previous_odom_data.x(), odom_pose.x(), atol=1e-3) and
                    np.isclose(self.previous_odom_data.y(), odom_pose.y(), atol=1e-3) and
                    np.isclose(self.previous_odom_data.theta(), odom_pose.theta(), atol=1e-3)):
                    # Skip processing if odometry data is effectively the same
                    return

            print("Odom previous and current",self.previous_odom_data, odom_pose)
            
            pose_key = gtsam.symbol('x', self.pose_num)
            print(f"Adding Pose Node: {pose_key} -> ({odom_pose.x()}, {odom_pose.y()}, {odom_pose.theta()})")

            # Add odometry pose to the list for plotting
            self.odometry_poses_x.append(odom_pose.x())
            self.odometry_poses_y.append(odom_pose.y())
            
            # Add factors to the graph
            if self.pose_num > 0:
                # Add odometry factor
                prev_pose_key = gtsam.symbol('x', self.pose_num - 1)
                prev_pose = self.initial_estimates.atPose2(prev_pose_key)
                odometry_factor = prev_pose.between(odom_pose)
                print(f"Adding Odometry Factor: Between {prev_pose_key} and {pose_key}")
                self.graph.add(gtsam.BetweenFactorPose2(
                    prev_pose_key, pose_key, odometry_factor, self.odometry_noise_model))
            else:
                # Add prior factor for the first pose
                prior_noise = gtsam.noiseModel.Diagonal.Variances([1e-6, 1e-6, 1e-8])
                print(f"Adding Prior Factor for the first pose: {pose_key}")
                self.graph.add(gtsam.PriorFactorPose2(pose_key, odom_pose, prior_noise))

            # Insert initial guess for this pose
            self.initial_estimates.insert(pose_key, odom_pose)

            # Handle landmark observations
            if self.marker_data is not None:
                landmark_pose = gtsam.Point2(self.marker_data.x, self.marker_data.y)
                print(landmark_pose)
                landmark_id = 0  # Use the actual ID from the ArUco marker
                landmark_key = gtsam.symbol('L', landmark_id)  # Use symbol for landmark key

                # If landmark has not been inserted yet
                if landmark_id not in self.landmark_inserted:
                    print(f"Adding Landmark Node: {landmark_key} -> ({self.marker_data.x}, {self.marker_data.y})")
                    self.initial_estimates.insert(landmark_key, landmark_pose)
                    self.landmark_inserted.add(landmark_id)

                    # **Add a prior factor on the landmark with very low covariance**
                    landmark_prior_noise = gtsam.noiseModel.Diagonal.Variances([1e-6, 1e-6])  # Very small variances
                    print(f"Adding Prior Factor for Landmark: {landmark_key}")
                    self.graph.add(gtsam.PriorFactorPoint2(landmark_key, landmark_pose, landmark_prior_noise))
                    # self.get_logger().info(f"Added prior factor for landmark {landmark_id}")

                
                # Add measurement factor between current pose and landmark
                bearing = odom_pose.bearing(landmark_pose)
                range_measurement = odom_pose.range(landmark_pose)
                print(f"Adding Landmark Observation Factor: Between {pose_key} and {landmark_key}")
                self.graph.add(gtsam.BearingRangeFactor2D(
                    pose_key, landmark_key, bearing, range_measurement, self.landmark_noise_model))

            # Increment pose count
            self.pose_num += 1
            print(f"Pose count: {self.pose_num}")

            # Trigger optimization after threshold
            if self.pose_num % self.pose_threshold == 0:
                print("We entered optimization")
                
                self.optimize_pose_graph()

            # Update the previous odometry data
            self.previous_odom_data = odom_pose

    def transform_odometry_to_lidar(self, odom_pose):
        try:
            if not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                return  # Skip if transformation not available

            transform_stamped = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
            return do_transform_pose(odom_pose, transform_stamped)
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform odometry to lidar frame: {ex}")
            return None

    def extract_pose2_from_msg(self, pose_msg):
        """Convert geometry_msgs/Pose to gtsam.Pose2 (x, y, yaw)"""
        x = round(pose_msg.position.x,3)
        y = round(pose_msg.position.y,3)
        yaw = round(self.quaternion_to_yaw(pose_msg.orientation),3)
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

        print("\nFinal Result:\n{}".format(result))

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


        # Plot the updated trajectories
        self.plot_trajectories()

    def plot_trajectories(self):
        """Plot odometry and optimized trajectories"""
        # Clear the current axis
        self.ax.cla()

        # Plot the odometry and optimized trajectories
        # print("Odometry values", self.odometry_poses_x, self.odometry_poses_y)
        self.ax.plot(self.odometry_poses_x, self.odometry_poses_y, 'r--', label='Odometry Trajectory')

        print("Optimized values", self.optimized_poses_x, self.optimized_poses_y)
        # self.ax.plot(self.optimized_poses_x, self.optimized_poses_y, 'b-', label='Optimized Trajectory')

        # Plot the landmark position
        for landmark_id in self.landmark_inserted:
            landmark_key = gtsam.symbol('L', landmark_id)
            optimized_landmark = self.initial_estimates.atPoint2(landmark_key)
            print(optimized_landmark[0], optimized_landmark[1])
            self.ax.plot(optimized_landmark[0], optimized_landmark[1], 'go', label='Landmark' if landmark_id == 0 else '')

        # Set labels and title
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Robot Trajectory')
        self.ax.grid(True)
        self.ax.legend()

        # Draw and pause for real-time update
        plt.draw()
        plt.pause(0.001)
        # plt.show()





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
