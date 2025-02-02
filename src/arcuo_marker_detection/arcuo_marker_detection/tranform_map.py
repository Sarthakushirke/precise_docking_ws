import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, TransformStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import tf_transformations  # Helps with quaternion transformations
from scipy.spatial.transform import Rotation as R  # Used for yaw extraction

class ArucoPoseTransformer(Node):
    def __init__(self):
        super().__init__('aruco_pose_transformer')

        # Subscribe to the /aruco_poses topic
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.pose_array_callback,
            10
        )

        # Publisher for transformed poses in map frame
        self.publisher = self.create_publisher(
            PoseArray,
            '/aruco_poses_in_map',
            10
        )

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Aruco Pose Transformer Node Started")

    def pose_array_callback(self, msg):
        try:
            # Lookup transformation from camera frame to map frame
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())

            # Debug: Log received transformation
            self.get_logger().info(f"Transform from {msg.header.frame_id} to map: {transform}")

            # Create transformed PoseArray
            transformed_pose_array = PoseArray()
            transformed_pose_array.header.stamp = self.get_clock().now().to_msg()
            transformed_pose_array.header.frame_id = "map"

            for pose in msg.poses:
                # Create PoseStamped message
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = pose  # ✅ Correctly assigning the pose

                # Debug: Print the received pose
                self.get_logger().info(f"Received pose: x={pose_stamped.pose.position.x}, y={pose_stamped.pose.position.y}, z={pose_stamped.pose.position.z}")

                try:
                    # Convert transform to 4x4 transformation matrix
                    transform_matrix = self.transform_to_matrix(transform)

                    # Convert pose to homogeneous coordinates
                    pose_matrix = np.array([pose_stamped.pose.position.x,
                                            pose_stamped.pose.position.y,
                                            pose_stamped.pose.position.z,
                                            1.0])

                    # Apply transformation
                    transformed_position = transform_matrix @ pose_matrix

                    # Extract quaternion from transform
                    quat = transform.transform.rotation
                    transformed_orientation = [quat.x, quat.y, quat.z, quat.w]

                    # Convert quaternion to Euler angles (roll, pitch, yaw)
                    rotation = R.from_quat(transformed_orientation)
                    _, _, yaw_map = rotation.as_euler('xyz', degrees=False)  # Get yaw (radians)

                    # Create transformed PoseStamped manually
                    transformed_pose = Pose()
                    transformed_pose.position.x = transformed_position[0]
                    transformed_pose.position.y = transformed_position[1]
                    transformed_pose.position.z = transformed_position[2]
                    transformed_pose.orientation.x = transformed_orientation[0]
                    transformed_pose.orientation.y = transformed_orientation[1]
                    transformed_pose.orientation.z = transformed_orientation[2]
                    transformed_pose.orientation.w = transformed_orientation[3]

                    # Debug: Log transformed pose and yaw
                    self.get_logger().info(f"Transformed pose: x={transformed_pose.position.x}, y={transformed_pose.position.y}, z={transformed_pose.position.z}")
                    self.get_logger().info(f"Yaw (theta) in map frame: {yaw_map} radians")

                    # Add transformed pose to PoseArray
                    transformed_pose_array.poses.append(transformed_pose)

                except Exception as e:
                    self.get_logger().error(f"TF2 transformation error: {e}")
                    continue  # Skip this pose if transformation fails

            # Publish transformed poses
            self.publisher.publish(transformed_pose_array)
            self.get_logger().info("Published transformed ArUco poses in map frame")

        except Exception as e:
            self.get_logger().error(f"Lookup transform failed: {e}")

    def transform_to_matrix(self, transform):
        """Convert TransformStamped message to a 4x4 transformation matrix."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Convert quaternion to rotation matrix
        rot_matrix = tf_transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

        # Apply translation
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rot_matrix[:3, :3]
        transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        return transform_matrix

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
