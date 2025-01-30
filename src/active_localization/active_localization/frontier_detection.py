import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseArray, Pose
import matplotlib.pyplot as plt
import matplotlib.patches as patches


import tf_transformations
import tf2_ros
import numpy as np
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

class OccupancyGridUpdater(Node):
    def __init__(self):
        super().__init__('occupancy_grid_updater')

        self.get_logger().info('Node initialized and ready')

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/local_map',
            self.map_callback,
            10)  # Use default QoS settings
        self.get_logger().info('Subscribed to /local_map topic with default QoS settings')

        # Odometry subscription (uses default QoS)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     '/rosbot_base_controller/odom',
        #     self.odom_callback,
        #     10)



        self.get_logger().info('Subscribed to /odom topic')

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info('Subscribed to /scan topic')

        # Publisher for frontier markers
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontier_markers_original', 10)
        self.get_logger().info('Publishing frontier markers on /frontier_markers')

        self.centroid_pub = self.create_publisher(MarkerArray, '/centroid_markers_original', 10)
        self.get_logger().info('Publishing centroid markers on /centroid_markers')

        # Publisher for centroids
        self.centroids_pub = self.create_publisher(PoseArray, '/frontier_centroids_original', 10)
        self.get_logger().info('Publishing centroids on /frontier_centroids')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_data = None
        self.map_info = None
        self.expansion_size = 3
        self.min_group_size = 40

        # # Robot size (matches RViz dimensions)
        # self.robot_width = 0.3  # X scale
        # self.robot_height = 0.3  # Y scale

        # # Start visualization loop
        # self.create_timer(1.0, self.plot_map)  # Update plot every 1 second

    def costmap(self, data, width, height, resolution):
        data = np.array(data).reshape(height, width)
        wall = np.where(data == 100)
        for i in range(-self.expansion_size, self.expansion_size + 1):
            for j in range(-self.expansion_size, self.expansion_size + 1):
                if i == 0 and j == 0:
                    continue
                x = wall[0] + i
                y = wall[1] + j
                x = np.clip(x, 0, height - 1)
                y = np.clip(y, 0, width - 1)
                data[x, y] = 100
        data = data * resolution
        return data

    def frontierB(self, matrix):
        for i in range(len(matrix)):
            for j in range(len(matrix[i])):
                if matrix[i][j] == 0.0:
                    if i > 0 and matrix[i - 1][j] < 0:
                        matrix[i][j] = 2
                    elif i < len(matrix) - 1 and matrix[i + 1][j] < 0:
                        matrix[i][j] = 2
                    elif j > 0 and matrix[i][j - 1] < 0:
                        matrix[i][j] = 2
                    elif j < len(matrix[i]) - 1 and matrix[i][j + 1] < 0:
                        matrix[i][j] = 2
        return matrix

    def assign_groups(self, matrix):
        group = 1
        groups = {}
        for i in range(len(matrix)):
            for j in range(len(matrix[0])):
                if matrix[i][j] == 2:
                    group = self.dfs_iterative(matrix, i, j, group, groups)
        return matrix, groups

    def dfs_iterative(self, matrix, i, j, group, groups):
        stack = [(i, j)]
        while stack:
            i, j = stack.pop()
            if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
                continue
            if matrix[i][j] != 2:
                continue
            if group in groups:
                groups[group].append((i, j))
            else:
                groups[group] = [(i, j)]
            matrix[i][j] = 0  # Mark as visited
            # Add neighboring cells to the stack
            stack.extend([
                (i + 1, j),
                (i - 1, j),
                (i, j + 1),
                (i, j - 1),
                (i + 1, j + 1),
                (i - 1, j - 1),
                (i - 1, j + 1),
                (i + 1, j - 1),
            ])
        return group + 1
    
    def calculate_centroid(self, points):
        """Calculate the centroid (middle point) of a list of points."""
        if not points:
            return None
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        mean_x = sum(x_coords) / len(points)
        mean_y = sum(y_coords) / len(points)
        return (int(mean_x), int(mean_y))

    def exploration(self, data, width, height, resolution, column, row, originX, originY):
            data = self.costmap(data, width, height, resolution)  # Expand barriers
            data[row][column] = 0  # Robot's current location
            data[data > 5] = 1  # Set obstacles
            data = self.frontierB(data)  # Find frontier points
            data, groups = self.assign_groups(data)  # Group frontier points

            # Filter out small groups
            filtered_groups = {gid: points for gid, points in groups.items() if len(points) >= self.min_group_size}

            # Calculate centroids for all valid groups
            centroids = {}

            for group_id, points in filtered_groups.items():
                centroid = self.calculate_centroid(points)
                centroids[group_id] = centroid

            # Publish the frontier markers
            self.publish_frontier_markers(filtered_groups, resolution, originX, originY)

            # Publish the centroid markers (spheres)
            self.publish_centroid_markers(centroids, resolution, originX, originY)

            # Publish the centroids to a topic
            self.publish_centroids(centroids, resolution, originX, originY)

            return centroids
    
    def publish_centroids(self, centroids, resolution, originX, originY):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"  # Set to your map frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for centroid in centroids.values():
            # Convert grid indices to world coordinates
            x = originX + centroid[1] * resolution + resolution / 2
            y = originY + centroid[0] * resolution + resolution / 2

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0  # Assuming a flat ground
            pose.orientation.w = 1.0  # Neutral orientation (no rotation)

            pose_array.poses.append(pose)

        # Publish the centroids
        self.centroids_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} centroids to frontier_centroids_original")


    def publish_frontier_markers(self, groups, resolution, originX, originY):
        marker_array = MarkerArray()
        marker_id = 0

        # Clear previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for group_id, points in groups.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = marker_id
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            # Set the scale of the marker
            marker.scale.x = resolution * 1.5  # Adjust as needed
            marker.scale.y = resolution * 1.5  # Adjust as needed

            # Set a unique color for each group
            color = self.get_color(group_id)
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0

            # Add the points to the marker
            for (i, j) in points:
                x = originX + j * resolution
                y = originY + i * resolution

                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                marker.points.append(point)

            marker_array.markers.append(marker)
            marker_id += 1

        # Publish the markers
        self.frontier_pub.publish(marker_array)


    def publish_centroid_markers(self, centroids, resolution, originX, originY):
        """
        Publish centroid markers as spheres in RViz.
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Clear previous centroid markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for group_id, centroid in centroids.items():
            x = originX + centroid[1] * resolution
            y = originY + centroid[0] * resolution

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "centroids"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2  # Slightly elevate for better visibility
            marker.pose.orientation.w = 1.0

            # Set the scale of the marker
            marker.scale.x = resolution * 2.0  # Adjust size as needed
            marker.scale.y = resolution * 2.0
            marker.scale.z = resolution * 2.0

            # Set the color of the centroid marker
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Yellow
            marker.color.a = 1.0  # Fully opaque

            marker_array.markers.append(marker)
            marker_id += 1

        # Publish the centroid markers
        self.centroid_pub.publish(marker_array)
    


    def get_color(self, group_id):
        # Generate colors based on group_id
        np.random.seed(group_id)
        color = np.random.rand(3)
        return color

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

        # Ensure self.x and self.y are defined
        if not hasattr(self, 'x') or not hasattr(self, 'y'):
            self.get_logger().warn('Robot position not yet received.')
            return

        column = int((self.x - self.originX) / self.resolution)
        row = int((self.y - self.originY) / self.resolution)

        frontiers_middle = self.exploration(self.data, self.width, self.height, self.resolution, column, row, self.originX, self.originY)

        print("Frontiers ", frontiers_middle)

    # def plot_map(self):
    #     """Visualize the occupancy grid, robot position, and frontiers"""
    #     if self.map_data is None:
    #         self.get_logger().warn("Map data not received yet.")
    #         return
        
    #     # Extract map properties
    #     resolution = self.map_data.info.resolution
    #     width = self.map_data.info.width
    #     height = self.map_data.info.height
    #     origin_x = self.map_data.info.origin.position.x
    #     origin_y = self.map_data.info.origin.position.y
    #     data = np.array(self.map_data.data).reshape((height, width))

    #     print("Map data",self.map_data)

    #     # Convert occupancy grid to an image
    #     map_image = np.copy(data)
    #     map_image[map_image == -1] = 0  # Unknown space (gray)
    #     map_image[map_image == 100] = 150   # Obstacles (black)
    #     map_image[map_image == 0] = 255   # Free space (white)

    #     fig, ax = plt.subplots(figsize=(6, 6))
    #     ax.imshow(map_image, cmap='gray', origin='lower', extent=[origin_x, origin_x + width * resolution, 
    #                                                               origin_y, origin_y + height * resolution])
        

    #     # Plot robot as a yellow filled rectangle
    #     if self.x is not None and self.y is not None:
    #         robot_rect = patches.Rectangle(
    #             (self.x - self.robot_width / 2, self.y - self.robot_height / 2),  # Center the rectangle
    #             self.robot_width, self.robot_height,  # Width and height
    #             edgecolor='black', facecolor='yellow', linewidth=2, label="Robot"  # Yellow filled with black border
    #         )
    #         ax.add_patch(robot_rect)
        

    #     # Labels and legend
    #     ax.set_title("Occupancy Grid & Frontiers")
    #     ax.set_xlabel("X Position (m)")
    #     ax.set_ylabel("Y Position (m)")
    #     ax.legend(loc='upper right')
    #     plt.show()


    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.get_logger().debug(f'Odom callback triggered, x: {self.x}, y: {self.y}')

    def lidar_callback(self, msg):
        self.scan_data = msg
        self.scan = msg.ranges

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
