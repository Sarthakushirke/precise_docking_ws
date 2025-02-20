#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import gtsam
import math
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
        self.odometry_sub = self.create_subscription(Odometry, '/noisy_odom', self.odometry_callback, 10)
        self.true_odom_sub = self.create_subscription(Odometry, '/odom', self.true_odometry_callback, 10)  # True odometry
        
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
        self.last_odometry_poses = []
        self.last_optimized_poses = []
        self.noisy_odom_x, self.noisy_odom_y = [], []
        self.true_odom_x, self.true_odom_y = [], []

        self.previous_landmarks = {}  # Dictionary to store previous landmark positions
        self.landmark_change_threshold = 4  # Adjust threshold as needed (meters)

        # Initialize the plot
        plt.ion()
        self.figure, self.ax = plt.subplots()

    def odometry_callback(self, msg):

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # If we have a previous position, calculate the distance moved
        if hasattr(self, 'prev_x') and hasattr(self, 'prev_y'):
            dx = current_x - self.prev_x
            dy = current_y - self.prev_y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance > 1.0:
                self.get_logger().warn(
                    f"Large jump detected (distance: {distance:.2f}). Skipping this odom update."
                )
                return  # Skip processing this message

        # Update the previous position
        self.prev_x = current_x
        self.prev_y = current_y

        # Store noisy odometry position
        self.noisy_odom_x.append(current_x)
        self.noisy_odom_y.append(current_y)


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


    def true_odometry_callback(self, msg):
        """Handles incoming true odometry data (ground truth)."""



        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # If we have a previous position, calculate the distance moved
        if hasattr(self, 'prev_x') and hasattr(self, 'prev_y'):
            dx = current_x - self.prev_x
            dy = current_y - self.prev_y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance > 1.0:
                self.get_logger().warn(
                    f"Large jump detected (distance: {distance:.2f}). Skipping this odom update."
                )
                return  # Skip processing this message

        # Update the previous position
        self.prev_x = current_x
        self.prev_y = current_y


        true_x = msg.pose.pose.position.x
        true_y = msg.pose.pose.position.y

        # Store true odometry position
        self.true_odom_x.append(true_x)
        self.true_odom_y.append(true_y)

    # def aruco_callback(self, msg):
    #     """Handles incoming ArUco markers in map frame"""
    #     for pose in msg.poses:
    #         yaw = self.quaternion_to_yaw(pose.orientation)  # Extract yaw (θ)
    #         z_distance = pose.position.z*10  # Distance to marker

    #         print("z_distance",z_distance)

    #         # Compute global landmark position using robot's current pose
    #         if self.odom_data:
    #             landmark_x = self.odom_data.x() + z_distance * np.cos(yaw)
    #             landmark_y = self.odom_data.y() + z_distance * np.sin(yaw)

    #             print("Odom data", self.odom_data)

    #             print("Landmark data",landmark_x, landmark_y)

    #             self.add_landmark(landmark_x, landmark_y)


    # def aruco_callback(self, msg):
    #     """Handles incoming ArUco markers in the map frame and adds only if position change is significant."""
    #     for pose in msg.poses:
    #         yaw = self.quaternion_to_yaw(pose.orientation)  # Extract yaw (θ)
    #         z_distance = pose.position.z * 10  # Distance to marker

    #         print("z_distance", z_distance)

    #         # Compute global landmark position using robot's current pose
    #         if self.odom_data:
    #             landmark_x = self.odom_data.x() + z_distance * np.cos(yaw)
    #             landmark_y = self.odom_data.y() + z_distance * np.sin(yaw)

    #             print("Odom data", self.odom_data)
    #             print("Landmark data", landmark_x, landmark_y)

    #             # Check if the landmark has changed significantly
    #             landmark_key = f"{round(landmark_x, 2)}_{round(landmark_y, 2)}"

    #             if landmark_key in self.previous_landmarks:
    #                 prev_x, prev_y = self.previous_landmarks[landmark_key]
    #                 distance_change = np.linalg.norm([landmark_x - prev_x, landmark_y - prev_y])

    #                 print("Distance change", distance_change)

    #                 if distance_change < self.landmark_change_threshold:
    #                     print(f"Landmark change ({distance_change:.2f}m) is below threshold. Skipping update.")
    #                     continue  # Skip this landmark update

    #             # Store the new landmark position
    #             self.previous_landmarks[landmark_key] = (landmark_x, landmark_y)

    #             print("Landmark added ", self.previous_landmarks )

    #             # Add landmark to the pose graph
    #             self.add_landmark(landmark_x, landmark_y)


    def aruco_callback(self, msg):


        if not (self.odom_data and self.cmd_velocity):

            print("In arcuo marker")
            return


        """Handles incoming ArUco markers in the map frame and adds only if position change is significant."""
        for pose in msg.poses:
            yaw = self.quaternion_to_yaw(pose.orientation)  # Extract yaw (θ)
            z_distance = pose.position.z * 10  # Distance to marker

            print("z_distance", z_distance)

            # Compute global landmark position using robot's current pose
            if self.odom_data:
                landmark_x = self.odom_data.x() + z_distance * np.cos(yaw)
                landmark_y = self.odom_data.y() + z_distance * np.sin(yaw)

                # Compute distance between robot and landmark
                distance_to_robot = np.linalg.norm([landmark_x - self.odom_data.x(), landmark_y - self.odom_data.y()])

                print("Odom data", self.odom_data)
                print("Landmark data", landmark_x, landmark_y)

                    # Check if the landmark is within a 5-meter range
                if distance_to_robot > 11.0:  # Change 5.0 to your preferred threshold
                    print(f"Landmark at ({landmark_x}, {landmark_y}) is out of range ({distance_to_robot:.2f}m). Skipping.")
                    continue  # Skip adding this landmark


                # Compare with all previous landmarks
                add_landmark = True  # Assume the landmark should be added

                for prev_x, prev_y in self.previous_landmarks.values():
                    distance_change = np.linalg.norm([landmark_x - prev_x, landmark_y - prev_y])

                    print("Distance change:", distance_change)

                    if distance_change < self.landmark_change_threshold:
                        print(f"Landmark change ({distance_change:.2f}m) is below threshold. Skipping update.")
                        add_landmark = False
                        break  # No need to check further, already below threshold

                if add_landmark:
                    # Store the new landmark position
                    landmark_key = len(self.previous_landmarks)  # Assign a new ID for tracking
                    self.previous_landmarks[landmark_key] = (landmark_x, landmark_y)

                    print("Landmark added:", self.previous_landmarks)

                    # Add landmark to the pose graph
                    self.add_landmark(landmark_x, landmark_y)


    def process_data(self):
        """Process odometry data and velocity"""
        if not (self.odom_data and self.cmd_velocity):

            print("Odom and velocity not yet")
            return
        
        if self.cmd_velocity.linear.x == 0 and self.cmd_velocity.angular.z == 0:
            print("velocity not yet ")
            return  # Skip if the robot is stationary

        # Ensure new pose is different from previous pose
        if self.previous_odom_data and np.isclose(self.previous_odom_data.x(), self.odom_data.x(), atol=1e-3) and np.isclose(self.previous_odom_data.y(), self.odom_data.y(), atol=1e-3):
            print("Ensuring previous and not pose is slightly different")
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

        print("landmark_pose", landmark_pose)
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
        # self.ax.plot(self.odometry_poses_x, self.odometry_poses_y, 'r--', label='Odometry Trajectory')
        self.ax.plot(self.true_odom_x, self.true_odom_y, 'g-', label='True Odometry')
        
        # # Plot optimized trajectory
        self.ax.plot(self.optimized_poses_x, self.optimized_poses_y, 'b-', label='Pose graph Optimized Trajectory')

        # Plot true odometry (Ground Truth)
        
        
        # self.ax.plot(self.noisy_odom_x, self.noisy_odom_y, 'r--', label='Noisy Odometry')

            # Store the last N odometry poses
        # if self.odometry_poses_x and self.odometry_poses_y:
        #     last_odom = (self.odometry_poses_x[-1], self.odometry_poses_y[-1])
        #     self.last_odometry_poses.append(last_odom)


        # if self.optimized_poses_x and self.optimized_poses_y:
        #     last_pose = (self.optimized_poses_x[-1], self.optimized_poses_y[-1])
        #     self.last_optimized_poses.append(last_pose)


        # for pose_x, pose_y in self.last_optimized_poses:
        #     self.ax.scatter(pose_x, pose_y, color='magenta', marker='D', s=1, label='Last Optimized Pose' if pose_x == self.last_optimized_poses[-1][0] else "")



        # for odom_x, odom_y in self.last_odometry_poses:
        #     self.ax.scatter(odom_x, odom_y, color='cyan', marker='s', s=100, label='Last Odom' if odom_x == self.last_odometry_poses[-1][0] else "")


        # Plot landmark positions
        # for landmark_id in self.landmark_inserted:
        #     landmark_key = gtsam.symbol('L', landmark_id)
        #     optimized_landmark = self.initial_estimates.atPoint2(landmark_key)

        #     print("Landmark pose",optimized_landmark[0], optimized_landmark[1])

        #     # Plot landmark position as a green dot
        #     self.ax.scatter(optimized_landmark[0], optimized_landmark[1], color='g', marker='o', label='Landmark' if landmark_id == 0 else "")

        #     # Annotate landmark ID on the plot
        #     self.ax.text(optimized_landmark[0], optimized_landmark[1], f"L{landmark_id}", fontsize=10, ha='right')


        for landmark_id in self.landmark_inserted:
            landmark_key = gtsam.symbol('L', landmark_id)

            # Directly retrieve and plot landmark
            if self.initial_estimates.exists(landmark_key):
                landmark_pose = self.initial_estimates.atPoint2(landmark_key)

                print("Landmark plot pose",landmark_pose[0], landmark_pose[1])
                self.ax.scatter(landmark_pose[0], landmark_pose[1], color='g', marker='o', label='Landmark')
                self.ax.text(landmark_pose[0], landmark_pose[1], f"L{landmark_id}", fontsize=10, ha='right')


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
