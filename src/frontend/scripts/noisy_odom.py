#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R
import math

class NoisyOdomNode(Node):
    def __init__(self):
        super().__init__('noisy_odom_node')

        # Subscribe to the original odometry topic
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher for noisy odometry
        self.noisy_odom_pub = self.create_publisher(Odometry, '/noisy_odom', 10)

        # Noise parameters
        self.position_noise_std = 0.0001  # Standard deviation for position noise (meters)
        self.orientation_noise_std = 0  # Standard deviation for orientation noise (radians)

        # Store trajectories for plotting
        self.original_x, self.original_y = [], []
        self.noisy_x, self.noisy_y = [], []

        # Initialize Matplotlib for live plotting
        plt.ion()
        self.figure, self.ax = plt.subplots()
        
        self.get_logger().info("Noisy Odometry Node Started, listening to /odom")

    def odom_callback(self, msg):
        """Callback function that adds noise to odometry data and publishes it."""


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
        
        noisy_odom = Odometry()
        noisy_odom.header = msg.header  # Keep original header

        # Add Gaussian noise to position
        noisy_x = msg.pose.pose.position.x + np.random.normal(0, self.position_noise_std)
        noisy_y = msg.pose.pose.position.y + np.random.normal(0, self.position_noise_std)
        noisy_odom.pose.pose.position.x = noisy_x
        noisy_odom.pose.pose.position.y = noisy_y
        noisy_odom.pose.pose.position.z = msg.pose.pose.position.z  # No noise in Z (assuming 2D motion)

        # Convert quaternion to Euler angles
        quat = msg.pose.pose.orientation
        r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = r.as_euler('xyz', degrees=False)

        # Add noise to yaw (assuming 2D motion)
        noisy_yaw = euler[2] + np.random.normal(0, self.orientation_noise_std)

        # Convert back to quaternion
        noisy_quat = R.from_euler('xyz', [euler[0], euler[1], noisy_yaw], degrees=False).as_quat()
        noisy_odom.pose.pose.orientation = Quaternion(x=noisy_quat[0], y=noisy_quat[1], z=noisy_quat[2], w=noisy_quat[3])

        # Copy twist (velocity) without adding noise
        noisy_odom.twist = msg.twist

        # Publish noisy odometry
        self.noisy_odom_pub.publish(noisy_odom)

        # Store positions for plotting
        self.original_x.append(msg.pose.pose.position.x)
        self.original_y.append(msg.pose.pose.position.y)
        self.noisy_x.append(noisy_x)
        self.noisy_y.append(noisy_y)

        # Plot trajectories
        # self.plot_trajectories()

    def plot_trajectories(self):
        """Plots the original and noisy odometry paths in real-time."""
        self.ax.cla()  # Clear previous plot

        # Plot original odometry trajectory
        self.ax.plot(self.original_x, self.original_y, 'b-', label="Original Odometry")

        # Plot noisy odometry trajectory
        self.ax.plot(self.noisy_x, self.noisy_y, 'r--', label="Noisy Odometry")

        # Labels, title, and legend
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Noisy vs. Original Odometry')
        self.ax.legend()
        self.ax.grid(True)

        # Refresh plot
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
