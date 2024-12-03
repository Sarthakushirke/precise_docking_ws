import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from .next_best_view import check_visible_objects_from_centroid_simple
from geometry_msgs.msg import PoseArray, Pose

import tf_transformations
import tf2_ros
import numpy as np
import heapq , math , random , yaml
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

class OccupancyGridUpdater(Node):
    def __init__(self):
        super().__init__('occupancy_grid_updater')

        self.get_logger().info('Node initialized and ready')


        # Define a QoS profile that matches the publisher's QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile)  # Use default QoS settings

        # Odometry subscription (uses default QoS)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.get_logger().info('Subscribed to /odom topic')

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info('Subscribed to /scan topic')

        # Subscribe to frontier centroids
        self.centroids_sub = self.create_subscription(
            PoseArray,
            '/frontier_centroids',
            self.frontier_centroids_callback,
            10)
        self.get_logger().info('Subscribed to /frontier_centroids topic')

        # Publisher for the path marker
        self.path_pub = self.create_publisher(Marker, '/path_marker', 10)
        self.get_logger().info('Publishing path markers on /path_marker')

        # TF buffer and listener  
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_data = None
        self.map_info = None
        self.expansion_size = 5
        # Initialize robot position attributes
        self.x = None
        self.y = None

    def euler_from_quaternion(self,x,y,z,w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

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


    def heuristic(self,a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def astar(self,array, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data = data + [start]
                data = data[::-1]
                return data
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:                
                        if array[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        # If no path to goal was found, return closest path to goal
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                dist = self.heuristic(node, goal)
                if dist < closest_dist:
                    closest_node = node
                    closest_dist = dist
            if closest_node is not None:
                data = []
                while closest_node in came_from:
                    data.append(closest_node)
                    closest_node = came_from[closest_node]
                data = data + [start]
                data = data[::-1]
                return data
        return False


    def path_to_destination(self, data, width, height, resolution, column, row, originX, originY):
            data = self.costmap(data, width, height, resolution)  # Expand barriers
            data[row][column] = 0  # Robot's current location
            data[data > 5] = 1  # Set obstacles
            # Convert goal coordinates to grid indices
            goal_x = 7  # Goal x-coordinate in meters
            goal_y = 0   # Goal y-coordinate in meters
            goal_column = int((goal_x - originX) / resolution)
            goal_row = int((goal_y - originY) / resolution)
            goal_indices = (goal_row, goal_column)
            path = self.astar(data, (row,column), goal_indices)
            if path:
                path_coords = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
                self.publish_path_marker(path_coords)
                # self.publish_test_marker()

            else:
                self.get_logger().warn("No path found to the goal!")

            return path
    

    def publish_path_marker(self, path_coords):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the scale of the marker
        marker.scale.x = 0.05  # Line thickness

        # Set the color of the marker
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add the path points to the marker
        for x, y in path_coords:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)

        # Publish the marker
        self.path_pub.publish(marker)
        self.get_logger().info("Path marker published!")


    def publish_test_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test_marker"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # Set the scale of the marker
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.3

        # Set the color of the marker
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the lifetime to infinite
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        # Publish the marker
        self.path_pub.publish(marker)
        self.get_logger().info("Test marker published!")


    

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
            
            self.x = -8
            self.y = 5

            column = int((self.x - self.originX) / self.resolution)
            row = int((self.y - self.originY) / self.resolution)

            # path = self.path_to_destination(self.data, self.width, self.height, self.resolution, column, row, self.originX, self.originY)

            # print(path)

    def frontier_centroids_callback(self, msg):

        centroids_info = []

        if self.map_data is None:
            self.get_logger().warn("Map data not yet received.")
            return

        # Prepare the occupancy grid
        data = self.map_data.data
        data = self.costmap(data, self.width, self.height, self.resolution)
        data[data > 5] = 1  # Set obstacles
        data[data <= 5] = 0  # Set free space

        # Convert goal position to grid indices
        goal_x = 7  # Goal x-coordinate in meters
        goal_y = 0  # Goal y-coordinate in meters
        goal_column = int((goal_x - self.originX) / self.resolution)
        goal_row = int((goal_y - self.originY) / self.resolution)
        goal_indices = (goal_row, goal_column)

        # Initialize variables to store the best centroid
        shortest_path_length = float('inf')
        best_centroid = None
        best_path = None

        # Process each centroid
        for pose in msg.poses:
            centroid_x = pose.position.x
            centroid_y = pose.position.y

            centroid_column = int((centroid_x - self.originX) / self.resolution)
            centroid_row = int((centroid_y - self.originY) / self.resolution)
            centroid_indices = (centroid_row, centroid_column)

            # Compute the path from centroid to goal
            path = self.astar(data, centroid_indices, goal_indices)

            # Calculate visible objects from this centroid
            visible_objects = check_visible_objects_from_centroid_simple(centroid_x, centroid_y)
            num_visible_objects = len(visible_objects)

            if path:
                path_length = len(path) * self.resolution
                self.get_logger().info(f"Path length from centroid at ({centroid_x:.2f}, {centroid_y:.2f}) to goal: {path_length:.2f}")

                # if path_length < shortest_path_length:
                #     shortest_path_length = path_length
                #     best_centroid = (centroid_x, centroid_y)
                #     best_path = path

                        # Store information for utility calculation
                centroids_info.append({
                    'centroid_x': centroid_x,
                    'centroid_y': centroid_y,
                    'path': path,
                    'path_length': path_length,
                    'visible_objects': visible_objects,
                    'num_visible_objects': num_visible_objects
                })
            else:
                self.get_logger().warn(f"No path found from centroid at ({centroid_x:.2f}, {centroid_y:.2f}) to goal.")


        # Weights for utility calculation
        weight_info_gain = 1.0
        weight_path_length = 1.0

        # Initialize variables to store the best centroid based on utility
        best_centroid_info = None
        highest_utility = -float('inf')

        # Calculate utility for each centroid
        for info in centroids_info:
            information_gain = info['num_visible_objects']
            path_length = info['path_length']

            # Compute utility
            utility_value = (weight_info_gain * information_gain) - (weight_path_length * path_length)

            # Store utility in the info dictionary
            info['utility'] = utility_value

            self.get_logger().info(f"Centroid at ({info['centroid_x']:.2f}, {info['centroid_y']:.2f}): Utility = {utility_value:.2f}")

            # Determine if this centroid has the highest utility so far
            if utility_value > highest_utility:
                highest_utility = utility_value
                best_centroid_info = info


        
        if best_centroid_info is not None:
            self.get_logger().info(f"Best centroid is at ({best_centroid_info['centroid_x']:.2f}, {best_centroid_info['centroid_y']:.2f}) with utility {best_centroid_info['utility']:.2f}")
            # Convert path indices to coordinates
            path_coords = [(p[1] * self.resolution + self.originX, p[0] * self.resolution + self.originY) for p in best_centroid_info['path']]
            # Publish the best path
            self.publish_path_marker(path_coords)
        else:
            self.get_logger().warn("No valid paths found from any centroid to the goal.")




    def lidar_callback(self, msg):
        self.scan_data = msg
        self.scan = msg.ranges

    def odom_callback(self,msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 