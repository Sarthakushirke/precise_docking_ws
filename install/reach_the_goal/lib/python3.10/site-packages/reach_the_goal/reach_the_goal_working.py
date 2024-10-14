import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

class PublishThePose(Node):
    def __init__(self):
        super().__init__('pose_sending_to_nav2')
        
        # Create a publisher for /goal_pose
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Define and publish a pose based on terminal input
        self.publish_pose_from_input()

    def publish_pose_from_input(self):
        # Check if enough arguments are provided
        if len(sys.argv) < 8:
            self.get_logger().error('Not enough input arguments provided. Usage: <x> <y> <z> <qx> <qy> <qz> <qw>')
            return
        
        try:
            # Extract pose information from command line arguments
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
            qx = float(sys.argv[4])
            qy = float(sys.argv[5])
            qz = float(sys.argv[6])
            qw = float(sys.argv[7])

            # Create and populate the PoseStamped message with input values
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'  # Adjust frame_id if necessary
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            
            # Publish the PoseStamped message
            self.goal_pose_publisher.publish(pose_msg)
            self.get_logger().info(f'Published goal pose: {pose_msg.pose.position}, {pose_msg.pose.orientation}')
        
        except ValueError:
            self.get_logger().error('Invalid input: Ensure all inputs are numbers.')

def main(args=None):
    rclpy.init(args=args)
    reach_node = PublishThePose()
    rclpy.spin(reach_node)
    reach_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
