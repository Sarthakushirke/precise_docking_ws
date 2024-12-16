#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from gtsam import NonlinearFactorGraph, Values, Pose3, Point3, BetweenFactor, noiseModel, LevenbergMarquardtOptimizer, PriorFactorPose3
import numpy as np

class PoseGraphNode(Node):
    def __init__(self):
        super().__init__('pose_graph_node')

        # GTSAM setup
        self.graph = NonlinearFactorGraph()
        self.initial_estimate = Values()

        # Initial Pose (Pose3 with zero rotation and translation)
        initial_pose = Pose3()
        self.initial_estimate.insert(0, initial_pose)
        self.graph.add(PriorFactorPose3(0, initial_pose, noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))))

        # Subscriber to the ArUco marker position (PointStamped)
        self.marker_sub = self.create_subscription(
            PointStamped, 
            '/marker_in_robot_frame', 
            self.marker_callback, 
            10
        )
        
        # Odometry data (Example)
        self.add_odometry_data()

    def marker_callback(self, msg):
        # Convert the PointStamped message into a GTSAM Point3
        marker_position = Point3(msg.point.x, msg.point.y, msg.point.z)
        
        # Add the marker as a landmark in the graph
        marker_id = 1  # Arbitrary marker ID
        self.initial_estimate.insert(marker_id, marker_position)
        
        # Add a factor connecting the robot's pose to the marker position
        noise = noiseModel.Isotropic.Sigma(3, 0.1)
        self.graph.add(GenericProjectionFactorPose3Point3(marker_position, noise, 0, marker_id, PinholeCamera.Cal3_S2()))

        # Perform optimization
        self.optimize_graph()

    def add_odometry_data(self):
        # Example of adding odometry between two poses
        odometry_pose = Pose3()  # Example relative pose
        self.initial_estimate.insert(1, self.initial_estimate.at(0) * odometry_pose)
        self.graph.add(BetweenFactorPose3(0, 1, odometry_pose, noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.2]))))

    def optimize_graph(self):
        # Optimize the pose-graph using Levenberg-Marquardt
        optimizer = LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
        result = optimizer.optimize()
        
        # Log the optimized poses
        for i in range(len(self.initial_estimate.keys())):
            optimized_pose = result.atPose3(i)
            self.get_logger().info(f"Optimized Pose {i}: {optimized_pose}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
