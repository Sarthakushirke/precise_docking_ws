#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

class OdomToMap(Node):
    def __init__(self):
        super().__init__('odom_to_map')
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # Extract position and orientation from odometry data
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose  # Use the position and orientation from odom


        # Print the pose (position and orientation) from odometry
        self.print_pose(odom_pose.pose)

        # Transform odometry pose to the map frame
        self.transform_odometry_to_map(odom_pose.pose)


    def print_pose(self, pose):
        # Print position
        self.get_logger().info(f"Odometry Position in odom frame: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
        # Print orientation
        # self.get_logger().info(f"Odometry Orientation: x={pose.orientation.x}, y={pose.orientation.y}, "
        #                        f"z={pose.orientation.z}, w={pose.orientation.w}")

    def transform_odometry_to_map(self, odom_pose):
        try:
            # Wait for the transform from odom to map with a timeout (e.g., 1 second)
            if not self.tf_buffer.can_transform('diff_drive/lidar_link', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                self.get_logger().warn('Transform from odom to map is not available yet. Retrying...')
                return  # Skip this callback if transform is not available

            # Look up the transformation from odom to map
            transform_stamped = self.tf_buffer.lookup_transform(
                'diff_drive/lidar_link',  # Target frame
                'odom',  # Source frame
                rclpy.time.Time()  # Get the latest available transform
            )

            # Transform the odometry pose from odom frame to map frame
            transformed_pose = do_transform_pose(odom_pose, transform_stamped)

            # self.get_logger().info("Here is the data")

            # # Log the transformed position and orientation in the map frame
            self.get_logger().info(f"Transformed Odometry in lidar  Frame: {transformed_pose.position.x}, "
                                        f"{transformed_pose.position.y}, {transformed_pose.position.z}")
            # self.get_logger().info(f"Transformed Orientation: {transformed_pose.orientation.x}, "
            #                             f"{transformed_pose.orientation.y}, {transformed_pose.orientation.z}, "
            #                             f"{transformed_pose.orientation.w}")

            
            self.get_logger().info(f"transforming")

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform odometry to map frame: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomToMap()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
