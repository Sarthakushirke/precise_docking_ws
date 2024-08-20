import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class publish_the_pose(Node):
    def __init__(self):
        super().__init__('pose_sending_to_nav2')
        
        # Create a publisher for /goal_pose
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Define and publish a sample pose at the start
        self.publish_sample_pose()

    def publish_sample_pose(self):
        # Create and populate the PoseStamped message with a sample pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Adjust frame_id if necessary
        pose_msg.pose.position.x = -5.0
        pose_msg.pose.position.y = 4.0
        pose_msg.pose.position.z = 0.0
        
        # Orientation in quaternion (example values)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        # Publish the PoseStamped message
        self.goal_pose_publisher.publish(pose_msg)
        self.get_logger().info(f'Published goal pose: {pose_msg.pose.position}, {pose_msg.pose.orientation}')

def main(args=None):
    rclpy.init(args=args)
    reach_node = publish_the_pose()
    rclpy.spin(reach_node)
    reach_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
