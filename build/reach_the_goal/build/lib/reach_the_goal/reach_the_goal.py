import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class publish_the_pose(Node):
    def __init__(self):
        super().__init__('pose_sending_to_nav2')

        # Declare parameters
        self.declare_parameter('x', 1.0)  # default value of x
        self.declare_parameter('y', 2.0)   # default value of y
        self.declare_parameter('z', 0.0)   # default value of z
        self.declare_parameter('qx', 0.0)  # default value of quaternion x
        self.declare_parameter('qy', 0.0)  # default value of quaternion y
        self.declare_parameter('qz', 0.0)  # default value of quaternion z
        self.declare_parameter('qw', 0.707)  # default value of quaternion w
        
        # Create a publisher for /goal_pose
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Get parameters
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        qx = self.get_parameter('qx').value
        qy = self.get_parameter('qy').value
        qz = self.get_parameter('qz').value
        qw = self.get_parameter('qw').value

        # Publish the pose
        self.publish_pose(x, y, z, qx, qy, qz, qw)

    def publish_pose(self, x, y, z, qx, qy, qz, qw):
        # Create and populate the PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
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

def main(args=None):
    rclpy.init(args=args)
    reach_node = publish_the_pose()
    rclpy.spin(reach_node)
    reach_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
