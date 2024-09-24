#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import struct
import numpy as np  # Import numpy for array operations
# import matplotlib.pyplot as plt  # For plotting the graphs
import tf2_ros
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
# from frontend.icp import icp
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2


class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Topic from which to read LaserScan messages
            self.listener_callback,
            10
        )
        
        self.publisher = self.create_publisher(PointCloud2, 'transform_cloud', 10)
        self.laser_projector = LaserProjection()

        self.prev_scan_pcd = None
        self.current_scan_pcd = None

        

    def listener_callback(self, msg):
        # Convert the LaserScan to PointCloud2
        pointcloud = self.laser_projector.projectLaser(msg)


        # self.get_logger().info(f"Original points in lidar frame: {pointcloud}")

        
        current_scan = self.convert_to_2D_points(pointcloud)

        hit_points_arr_3d = np.c_[current_scan, np.zeros(current_scan.shape[0])]

        # self.get_logger().info(f"lidar points: {hit_points_arr_3d }")

        self.current_scan_pcd = o3d.geometry.PointCloud()
        self.current_scan_pcd.points = o3d.utility.Vector3dVector(hit_points_arr_3d)

        if self.prev_scan_pcd is not None:
            pass
            # icp_result = self.perform_icp(self.current_scan_pcd, self.prev_scan_pcd, threshold=0.5, max_iteration=1000)

            # T_delta_icp = icp_result.transformation

        self.prev_scan_pcd = self.current_scan_pcd

        # pointcloud_ros = self.create_pointcloud2(pointcloud, msg.header)

        # Print the point cloud data (actual points)
        # self.print_pcd_data(pcd_data)

        # self.get_logger().info(f"Original points in lidar frame: {pcd_data}")

        # try:
        #     # Wait for the transform from odom to map with a timeout (e.g., 1 second)
        #     if not self.tf_buffer.can_transform('diff_drive/lidar_link', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
        #         self.get_logger().warn('Transform from diff_drive/lidar_link to map is not available yet. Retrying...')
        #         return  # Skip this callback if transform is not available

        #     # Look up the transformation from odom to map
        #     transform_stamped = self.tf_buffer.lookup_transform(
        #         'map',  # Target frame
        #         'diff_drive/lidar_link',  # Source frame
        #         rclpy.time.Time()  # Get the latest available transform
        #     )


            # points_before = self.pointcloud2_to_xy(pointcloud)

            # self.get_logger().info(f"Original points in lidar frame: {points_before}")

            # self.get_logger().info(f"Original points counts: {len(points_before)}")

            # Transform the point cloud data (2D version)
            # transformed_cloud = self.transform_pointcloud_2d(pointcloud, transform_stamped)

            # transformed_cloud_msg = self.create_pointcloud2(transformed_cloud, transform_stamped.header)
            

                # transformed_cloud = icp_aligned_cloud  # Use the aligned points from ICP

                # self.get_logger().info(f"ICP Transformation Matrix:\n{icp_transform}")

            # Store the current cloud for the next ICP iteration
            

            # self.get_logger().info(f"Transformed Points in map frame: {transformed_cloud}")

            # self.get_logger().info(f"Transformed Points Count: {len(transformed_cloud)}")
            # pass


        # except tf2_ros.TransformException as ex:
        #     self.get_logger().warn(f"Could not transform point cloud to map frame: {ex}")


        # Publish the PointCloud2 data
        self.publisher.publish(pointcloud)

    def convert_to_2D_points(self, cloud_out):
        # Convert ROS2 PointCloud2 to Open3D point cloud (2D, set z=0)
        points_list = []
        for p in pc2.read_points(cloud_out, skip_nans=True):
            points_list.append([p[0], p[1]]) 

        points_cloud = np.array(points_list)

        return points_cloud
    
    def perform_icp(self, source, target, threshold=5.0, max_iteration=100):
        print("++++++++++++++++++++Performing ICP++++++++++++++++++++")
        icp_result = o3d.pipelines.registration.registration_icp(
            source, target, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration)
        )
        return icp_result
    

    def transform_pointcloud_2d(self, cloud_msg, transform_stamped):
        # Extract the transformation matrix from TransformStamped
        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation

        # Convert the quaternion to a 2D rotation matrix (yaw only)
        yaw = self.quaternion_to_yaw(rotation)

        # Create the transformation matrix (3x3 for 2D)
        T = np.eye(3)
        T[0:2, 0:2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
        T[0:2, 2] = [translation.x, translation.y]  # Set translation (x, y)

        # Extract the point cloud data (x, y only for 2D)
        points = self.pointcloud2_to_xy(cloud_msg)

        # Apply the transformation to each point
        points_hom = np.hstack([points, np.ones((points.shape[0], 1))])  # Convert to homogeneous coordinates
        transformed_points_hom = (T @ points_hom.T).T  # Apply the transformation

        # Extract transformed xy
        transformed_points = transformed_points_hom[:, 0:2]

        return transformed_points
    


    # def transform_pointcloud_2d(self, cloud_msg, transform_stamped):
    #     # Extract the transformation matrix from TransformStamped
    #     translation = transform_stamped.transform.translation
    #     rotation = transform_stamped.transform.rotation

    #     # Convert the quaternion to a 2D rotation matrix (yaw only)
    #     yaw = self.quaternion_to_yaw(rotation)

    #     # Create the transformation matrix (3x3 for 2D)
    #     T = np.eye(3)
    #     T[0:2, 0:2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    #     T[0:2, 2] = [translation.x, translation.y]  # Set translation (x, y)

    #     # Extract the point cloud data (x, y only for 2D)
    #     points = self.pointcloud2_to_xy(cloud_msg)

    #     # Apply the transformation to each point
    #     points_hom = np.hstack([points, np.ones((points.shape[0], 1))])  # Convert to homogeneous coordinates
    #     transformed_points_hom = (T @ points_hom.T).T  # Apply the transformation

    #     # Extract transformed xy and add z = 0 for all points
    #     transformed_points = np.hstack([transformed_points_hom[:, 0:2], np.zeros((transformed_points_hom.shape[0], 1))])

    #     return transformed_points




    def create_pointcloud2(self, points, header):
        """Convert the transformed points into PointCloud2 format."""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Prepare the byte array for the cloud data
        cloud_data = []

        # Pack each (x, y, z) point into bytes
        for point in points:
            cloud_data.append(struct.pack('fff', point[0], point[1], 0.0))  # Z is set to 0 for 2D

        # Convert the list of packed bytes into a single byte array
        cloud_data_bytes = b''.join(cloud_data)

        # Create the PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 12  # Each point has 3 floats (x, y, z), 4 bytes each
        pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
        pointcloud_msg.is_dense = True  # Assume all points are valid
        pointcloud_msg.data = cloud_data_bytes  # Assign the byte array to the 'data' field

        return pointcloud_msg



    
    def quaternion_to_yaw(self, q):
        """Convert a quaternion into a yaw (2D rotation around the z-axis)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def pointcloud2_to_xy(self, cloud_msg):
        """Extract the (x, y) points from a PointCloud2 message."""
        points = []
        for p in self.read_points(cloud_msg, field_names=("x", "y"), skip_nans=True):
            points.append([p[0], p[1]])  # Only (x, y) for 2D
        return np.array(points)

    def read_points(self, cloud, field_names=None, skip_nans=False):
        """Yield namedtuples for each point in a PointCloud2."""
        fmt = "<ff"  # Only 2 floats: x, y
        width, height = cloud.width, cloud.height
        point_step, row_step = cloud.point_step, cloud.row_step

        # Iterate through the point cloud data
        for v in range(height):
            for u in range(width):
                point_offset = row_step * v + u * point_step
                if skip_nans:
                    x, y = struct.unpack_from(fmt, cloud.data, offset=point_offset)
                    if not (np.isnan(x) or np.isnan(y)):
                        yield (x, y)
                else:
                    yield struct.unpack_from(fmt, cloud.data, offset=point_offset)


def main(args=None):
    rclpy.init(args=args)
    laser_to_pointcloud = LaserToPointCloud()
    rclpy.spin(laser_to_pointcloud)
    # Destroy the node explicitly
    laser_to_pointcloud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
