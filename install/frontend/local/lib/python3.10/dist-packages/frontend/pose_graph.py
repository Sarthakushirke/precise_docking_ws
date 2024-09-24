import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import gtsam
import numpy as np

class PoseGraphOptimization(Node):
    def __init__(self):
        super().__init__('pose_graph_optimization')
        
        # Subscriptions for odometry and ICP data
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        # self.icp_sub = self.create_subscription(PoseStamped, '/icp', self.icp_callback, 10)
        
        # Noise models for the factors (adjust based on your data)
        self.noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, np.radians(5.0)]))  # [x, y, theta]
        
        # Create a factor graph
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()

        # Initialize the odometry and ICP data containers
        self.odom_data = None
        # self.icp_data = None
        
        # Track the pose number (start from pose 0)
        self.pose_num = 0
        
        # Set the number of poses after which to trigger optimization
        self.pose_threshold = 10
    
    def odometry_callback(self, msg):
        """Handles incoming odometry data"""
        self.odom_data = msg.pose.pose
        self.process_data()

    def icp_callback(self, msg):
        """Handles incoming ICP data"""
        self.icp_data = msg.pose
        self.process_data()

    def process_data(self):
        """Process odometry and ICP data if both are available"""
        if self.odom_data and self.icp_data:
            # Extract odometry pose (convert to 2D pose)
            odom_pose = self.extract_pose2_from_msg(self.odom_data)
            
            # Extract ICP pose (convert to 2D pose)
            # icp_pose = self.extract_pose2_from_msg(self.icp_data)
            
            # Add odometry factor to the graph
            if self.pose_num > 0:
                self.graph.add(gtsam.BetweenFactorPose2(self.pose_num - 1, self.pose_num, odom_pose, self.noise_model))
                # self.graph.add(gtsam.BetweenFactorPose2(self.pose_num - 1, self.pose_num, icp_pose, self.noise_model))

            # Insert initial guess for this pose
            self.initial_estimates.insert(self.pose_num, odom_pose)

            # Increment pose count
            self.pose_num += 1

            # Reset the odometry and ICP data after processing
            self.odom_data = None
            # self.icp_data = None

            # Trigger optimization after 10 poses=
            if self.pose_num >= self.pose_threshold:
                self.optimize_pose_graph()

    def extract_pose2_from_msg(self, pose_msg):
        """Convert geometry_msgs/Pose to gtsam.Pose2 (x, y, yaw)"""
        x = pose_msg.position.x
        y = pose_msg.position.y
        # Convert quaternion to yaw (only yaw for 2D case)
        yaw = self.quaternion_to_yaw(pose_msg.orientation)
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

        # Print optimized poses
        for i in range(self.pose_num):
            optimized_pose = result.atPose2(i)
            self.get_logger().info(f'Optimized Pose {i}: x={optimized_pose.x()}, y={optimized_pose.y()}, theta={optimized_pose.theta()}')

        # Reset the graph and initial estimates for the next batch of poses
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.pose_num = 0  # Reset pose count for next optimization

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphOptimization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Pose graph optimization completed.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
