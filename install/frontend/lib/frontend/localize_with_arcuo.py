#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
import heapq , math , random , yaml

from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from active_localization.next_best_view import check_visible_objects_from_centroid_simple


class MultiLocationMarkerNode(Node):
    def __init__(self):
        super().__init__('multi_location_marker_node')

        # marker_map[marker_id] => list of (x_map, y_map, theta_map)
        self.marker_map = {
            0: [
                (3.0, -1.0, 0.0),
                (-6.0, 5.0, 0.0),
                (-3.0,-4.0,0.0),
            ],
        }

        # Subscribe to a topic that just gives a PointStamped in the robot frame
        self.marker_sub = self.create_subscription(
            PointStamped,
            '/marker_in_robot_frame',
            self.marker_callback,
            10
        )

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
            qos_profile)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/hypothesis_map', 10)

        # Store current hypotheses (marker_id => list of (x_r, y_r, theta_r))
        self.hypotheses_dict = {}

        # Hardcode a single marker ID for demonstration
        self.current_marker_id = 0

        # Publisher for PoseArray (arrows in RViz)
        self.pose_array_pub = self.create_publisher(
            PoseArray, 
            '/hypothesis_poses', 
            10
        )

        # NEW: Publisher for MarkerArray (squares in RViz)
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            '/hypothesis_squares',
            10
        )

        self.frontier_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.get_logger().info('Publishing frontier markers on /frontier_markers')

        self.map_data = None
        self.map_info = None
        self.column = None
        self.row = None
        self.goal_column = None
        self.goal_row = None
        self.global_centroid = []

        self.goal_x = 7.0  # Goal x-coordinate in meters
        self.goal_y = 0.0  # Goal y-coordinate in meters


        

        self.get_logger().info('MultiLocationMarkerNode started.')


    def map_callback(self, msg):
        self.map_info = msg.info

        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_info.height, self.map_info.width))
        

        # Ensure self.x and self.y are defined
        if not hasattr(self, 'x') or not hasattr(self, 'y'):
            self.get_logger().warn('Robot position not yet received.')
            return

        # self.column = int((self.x - self.originX) / self.resolution)
        # self.row = int((self.y - self.originY) / self.resolution)



    def get_color(self, group_id):
        # Generate colors based on group_id
        np.random.seed(group_id)
        color = np.random.rand(3)
        return color

    def marker_callback(self, msg: PointStamped):
        """
        We only receive the marker's position (x,y,z) in the robot frame.
        We'll assume orientation=0 (phi_marker_relative=0).
        """

        if self.map_data is None:
            self.get_logger().warn('Map data not available yet')
            return

        # Hardcode the marker ID in this example
        marker_id = self.current_marker_id

        # If the marker does not exist in the map dictionary, ignore
        if marker_id not in self.marker_map:
            self.get_logger().warn(
                f"Marker ID {marker_id} not found in marker_map. Ignoring.")
            return

        # For demonstration, let's just assume a fixed distance=2
        distance = 2.0

        self.expansion_size = 2
        self.min_group_size = 40

        # We assume zero relative orientation
        phi_marker_relative = 0.0

        # Retrieve the list of known marker positions/orientations in the map
        marker_locations = self.marker_map[marker_id]
        new_hypotheses = []
        # Filter hypotheses based on the frontier condition
        filtered_hypotheses = []

        for (x_m, y_m, theta_m) in marker_locations:
            # Robot orientation in map
            theta_r = theta_m - phi_marker_relative  # => just theta_m if phi=0

            # Robot position in map
            x_r = x_m - distance * math.cos(theta_r)
            y_r = y_m - distance * math.sin(theta_r)

            new_hypotheses.append((x_r, y_r, theta_r))

        # Save these hypotheses
        self.hypotheses_dict[marker_id] = new_hypotheses

        # Log them
        for i, (x_r, y_r, theta_r) in enumerate(new_hypotheses, start=1):
            self.get_logger().info(
                f"Marker {marker_id} - Hypothesis #{i}: "
                f"x={x_r:.2f}, y={y_r:.2f}, theta={theta_r:.2f}"
            )

        # # --- PUBLISH POSE ARRAY (Arrows) FOR RVIZ ---
        # self.publish_pose_array(new_hypotheses)

        # # --- PUBLISH MARKER ARRAY (Squares) FOR RVIZ ---
        # self.publish_square_markers(new_hypotheses)

        num_beams = 16000
        angle_increment = 0.0003927233046852052
        scan_ranges = []
        start_angle = -3.14
        max_range = 10

        # reachable = np.zeros_like(self.map_data, dtype=bool)

        for i, (x_r, y_r, theta_r) in enumerate(new_hypotheses, start=1):

            # Re-init arrays for each hypothesis
            reachable = np.zeros_like(self.map_data, dtype=bool)
            updated_map = self.map_data.copy()
            for i in range(num_beams):
                beam_angle = start_angle + i * angle_increment
                reachable = self.get_ray_distance_bresenham(x_r, y_r, beam_angle, max_range,reachable)
                # scan_ranges.append(dist)

            # print("Reachable:", np.array2string(reachable, threshold=np.inf))
            # Update the map
            updated_map = self.map_data.copy()

            # Identify free cells in the map
            free_cells = (updated_map == 0)
        
            # Identify unreachable cells
            unreachable = ~reachable


            # Identify free and unreachable cells
            free_and_unreachable = free_cells & unreachable

            # Mark free, unreachable cells as unknown (-1)
            updated_map[free_and_unreachable] = -1
            
            # Ensure reachable free cells remain as 0
            reachable_free = free_cells & reachable
            updated_map[reachable_free] = 0  # This line may not be necessary but ensures clarity

            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            resolution = self.map_info.resolution

            self.column = int((x_r - origin_x) / resolution)
            self.row = int((y_r - origin_y) / resolution)

            # Convert goal position to grid indices
            self.goal_column = int((self.goal_x - origin_x) / resolution)
            self.goal_row = int((self.goal_y - origin_y) / resolution)
            self.goal_indices = (self.goal_row, self.goal_column)

            frontiers_middle = self.exploration(updated_map, self.map_info.width, self.map_info.height, resolution, self.column, self.row, origin_x, origin_y)

            print("Frontiers ", frontiers_middle)

            ######################

            # Initialize variables to store the best centroid
            shortest_path_length = float('inf')
            best_centroid = None
            best_path = None

            centroids_info = []


            orginal_scan_centroids = 3
          
            for frontier_id, (centroid_x, centroid_y) in frontiers_middle.items():

                # print("Length of frontiers",len(frontiers_middle))

                if orginal_scan_centroids == len(frontiers_middle):

                    filtered_hypotheses.append((x_r, y_r, theta_r))

                    # Update the hypotheses with the filtered list
                    new_hypotheses = filtered_hypotheses

                    centroid_column = int((centroid_x - origin_x) / resolution)
                    centroid_row = int((centroid_y - origin_y) / resolution)
                    centroid_indices = (centroid_row, centroid_column)

                    # Compute the path from centroid to goal
                    path = self.astar(self.map_data, centroid_indices, self.goal_indices)

                    # Calculate visible objects from this centroid
                    visible_objects = check_visible_objects_from_centroid_simple(centroid_x, centroid_y)
                    num_visible_objects = len(visible_objects)

                    if path:
                        path_length = len(path) * resolution
                        self.get_logger().info(f"Path length from centroid at ({centroid_x:.2f}, {centroid_y:.2f}) to goal: {path_length:.2f}")

                    
                        # Store information for utility calculation
                        centroids_info.append({
                            'centroid_x': centroid_x,
                            'centroid_y': centroid_y,
                            'path': path,
                            'path_length': path_length,
                            'visible_objects': visible_objects,
                            'num_visible_objects': num_visible_objects,
                            'centroid_indices': centroid_indices
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
                
                # Compute path from robot to best centroid
                best_centroid_indices = best_centroid_info['centroid_indices']

                self.global_centroid.append(best_centroid_indices)
                
                # path_robot_to_centroid = self.astar(self.map_data, robot_indices, best_centroid_indices)


            print("GLobal centroids", self.global_centroid)
 #####################################
            # --- PUBLISH POSE ARRAY (Arrows) FOR RVIZ ---
            self.publish_pose_array(new_hypotheses)

            # --- PUBLISH MARKER ARRAY (Squares) FOR RVIZ ---
            self.publish_square_markers(new_hypotheses)

            # print("Updated map",updated_map)

            # Log the number of free cells after update
            # num_free_after = np.count_nonzero(updated_map == 0)
            # self.get_logger().info(f"Number of free cells after update: {num_free_after}")

            # Publish the updated map
            updated_occupancy_grid = OccupancyGrid()
            updated_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            updated_occupancy_grid.header.frame_id = "map"
            updated_occupancy_grid.info = self.map_info
            updated_occupancy_grid.data = updated_map.flatten().tolist()
            self.map_pub.publish(updated_occupancy_grid)


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
        # self.publish_centroid_markers(centroids, resolution, originX, originY)

        # # Publish the centroids to a topic
        # self.publish_centroids(centroids, resolution, originX, originY)

        return centroids
    
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

    def heuristic(self,a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_ray_distance_bresenham(self,robot_x, robot_y, angle, max_range,reachable):
        """
        Returns the distance to obstacle along a single beam or max_range if no obstacle is found.
        
        :param robot_x, robot_y: Robot's position in world coords (meters)
        :param angle: Beam angle relative to the robot (radians)
        :param max_range: LiDAR max range (meters)
        :param map_info: OccupancyGrid.info (contains map origin, resolution, etc.)
        :param occupancy_grid: 2D array with 0=free, 1=occupied, -1=unknown, etc.
        :return: distance in meters
        """
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        # 1) Compute end-of-ray (x_end, y_end) ignoring obstacles
        x_end = robot_x + max_range * math.cos(angle)
        y_end = robot_y + max_range * math.sin(angle)

        # 2) Convert robot and end positions to map indices
        map_x0 = int((robot_x - origin_x) / resolution)
        map_y0 = int((robot_y - origin_y) / resolution)
        map_x1 = int((x_end   - origin_x) / resolution)
        map_y1 = int((y_end   - origin_y) / resolution)


   

        # 3) Run Bresenham
        line_cells = self.bresenham_line(map_x0, map_y0, map_x1, map_y1)

        # print(line_cells)

        for map_x, map_y in line_cells:
            # Ensure map_x and map_y are within the grid boundaries
            if 0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height:

                cell_value = self.map_data[map_y, map_x]
                # If the cell is occupied (e.g., 100 or >= 50) or unknown (-1),
                # stop the ray right here, so it doesn't go beyond.
                if cell_value == 100:
                    # Optional: you might also treat -1 as blocking:
                    # if cell_value == 100 or cell_value == -1:
                    break
                
                # Mark as reachable if it's free (0)
                if cell_value == 0:
                    reachable[map_y, map_x] = True


        return reachable
        






    def bresenham_line(self,x0, y0, x1, y1):
        """Bresenham's Line Algorithm.
        Produces a list of tuples from start and end.

        Parameters:
        x0, y0 - start point in grid indices
        x1, y1 - end point in grid indices

        Returns:
        List of (x, y) tuples representing the grid cells along the line
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0

        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
            points.append((x, y))
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            points.append((x, y))
        return points



    def publish_pose_array(self, hypotheses):
        """
        Publishes a PoseArray where each pose corresponds to one hypothesis.
        These will appear as arrows in RViz if you add a 'PoseArray' display.
        """
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "map"  # Hypotheses are in the 'map' frame

        for (x_r, y_r, theta_r) in hypotheses:
            pose = Pose()
            pose.position.x = x_r
            pose.position.y = y_r
            pose.position.z = 0.0

            # Convert theta_r (yaw) to quaternion
            quat = quaternion_from_euler(0.0, 0.0, theta_r)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            pose_array_msg.poses.append(pose)

        self.pose_array_pub.publish(pose_array_msg)


    def publish_square_markers(self, hypotheses):
        """
        Publishes a MarkerArray with squares (cubes) to represent each hypothesis.
        Each marker is a small, flat cube.
        """
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()

        for i, (x_r, y_r, theta_r) in enumerate(hypotheses):
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = "map"  # same as the pose array
            marker.ns = "hypothesis_squares"
            marker.id = i  # unique ID for each marker

            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = x_r
            marker.pose.position.y = y_r
            marker.pose.position.z = 0.0

            # Orientation (convert yaw to quaternion)
            quat = quaternion_from_euler(0.0, 0.0, theta_r)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            # Scale (make it look like a 2D square, e.g. 0.5 x 0.5 x 0.01)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.1

            # Color (e.g. green squares)
            marker.color.r = 1.0  # Adjust for lighter or darker grey (0.5 is medium grey)
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully opaque


            # Lifetime (0 = forever)
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            marker_array.markers.append(marker)

        self.marker_array_pub.publish(marker_array)


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


def main(args=None):
    rclpy.init(args=args)
    node = MultiLocationMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
