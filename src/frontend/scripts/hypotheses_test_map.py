#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point, Twist
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
import math


class HypothesisMapSubscriber(Node):
    def __init__(self):
        super().__init__('hypothesis_map_subscriber')

        # Subscribe to hypotheses map topics (each hypothesis will be in separate topic)
        self.local_map_data = {}  # Store maps by hypothesis ID
        self.hypotheses_dict = {}  # Store poses from PoseArray
        self.map_info = None  # Placeholder for map metadata

        # Subscribe to the LIDAR scan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10
        )

        # Subscribe to PoseArray for hypotheses
        self.pose_sub = self.create_subscription(
            PoseArray,
            "/hypothesis_poses",
            self.pose_callback,
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
        

        self.frontier_pub = self.create_publisher(MarkerArray, '/hyp_frontier_markers', 10)

        self.centroids_pub = self.create_publisher(PoseArray, '/hyp_frontier_centroids', 10)
        self.get_logger().info('Publishing centroids on /hyp_frontier_centroids')


        self.centroid_pub = self.create_publisher(MarkerArray, '/hyp_centroid_markers', 10)
        self.get_logger().info('Publishing centroid markers on /hyp_centroid_markers')

        # Subscribe to hypothesis maps dynamically (will be updated in pose callback)
        self.map_subs = {}
        self.hypothesis_id = 6
        self.map_publishers = {}  # Dictionary to store publishers for hypothesis maps
        self.start = True

        #Goal for simulation
        self.goal_x = 13.0  # Goal x-coordinate in meters
        self.goal_y = 0.0  # Goal y-coordinate in meters

        self.expansion_size = 3
        self.min_group_size = 50


    def pose_callback(self, msg):
        """ Receives hypotheses poses from /pose_array and updates self.hypotheses_dict """

        

        self.hypotheses_dict.clear()

        # Latched QoS (transient local) to get last published map even if published before subscribing
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        print("In the pose callback now I am")

        for i, pose in enumerate(msg.poses):  # Loop through all poses with an index
            x = pose.position.x
            y = pose.position.y
            theta = self.yaw_from_quaternion(pose.orientation)


            

            self.hypotheses_dict[i] = (x, y, theta)  # Store each pose uniquely

        print("Hypotheses:", self.hypotheses_dict)


        # print(f"In the pose callback for hypothesis {self.hypothesis_id}")

        # Subscribe to each hypothesis map if not already subscribed
        topic_name = f"/hypothesis_map_{self.hypothesis_id}"

        
        if self.start is True:
            self.map_subs[self.hypothesis_id] = self.create_subscription(
                OccupancyGrid,
                topic_name,
                self.map_callback_local,  # No need for lambda, since hypothesis_id is in frame_id
                latched_qos
            )
            self.get_logger().info(f"Subscribed to {topic_name}")

        self.start = False


    def map_callback_local(self, msg):
        """ Receives local maps for each hypothesis and stores them """

        # Extract hypothesis ID from the frame_id (which is set as "hypothesis_X")
        # self.hypothesis_id = int(msg.header.frame_id.split("_")[1])

        self.get_logger().info(f"Received map for Hypothesis ID: {self.hypothesis_id}")

        # Convert OccupancyGrid data to numpy array and store
        self.local_map_data[self.hypothesis_id] = np.array(msg.data, dtype=int).reshape(
            msg.info.height, msg.info.width
        )


        self.local_map_width = self.map_info.width 
        self.local_map_height = self.map_info.height 
        self.local_map_resolution = self.map_info.resolution
        self.origin_x = self.map_info.origin.position.x
        self.origin_y = self.map_info.origin.position.y

        # Store map metadata (assuming all maps share the same info)
        self.map_info = msg.info


    def map_callback(self, msg):
        self.map_info = msg.info

        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_info.height, self.map_info.width))
        


    def lidar_callback(self, msg):
        """ Processes LIDAR scan data and updates hypothesis maps """

        print("I am here ")
        if not self.hypotheses_dict or not self.local_map_data:
            self.get_logger().warn("No hypotheses or local maps available yet.")
            return
        

        # Ensure at least one hypothesis exists before checking
        if 1 in self.hypotheses_dict:
            x, y, _ = self.hypotheses_dict[1]  # Extract x, y from the first hypothesis

            # If we have a previous position, calculate the distance moved
            if hasattr(self, 'prev_x') and hasattr(self, 'prev_y'):
                dx = x - self.prev_x
                dy = y - self.prev_y
                distance = math.sqrt(dx * dx + dy * dy)

                if distance > 1.0:
                    self.get_logger().warn(
                        f"Large jump detected (distance: {distance:.2f}). Skipping this odom update."
                    )

                    self.hypotheses_dict.clear()
                    return  # Skip processing this message

            #Update the previous position using hypotheses[0]
            self.prev_x = x
            self.prev_y = y

        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        max_range = msg.range_max

        # Replace 'inf' values with max_range
        ranges = np.where(np.isinf(ranges), max_range, ranges)

        h = 6

        # for d, (x_r, y_r, theta_r) in self.hypotheses_dict.items():

        if self.hypotheses_dict:
            (robot_x_map, robot_y_map, yaw) = self.hypotheses_dict[1]
            
            print("Pose", robot_x_map, robot_y_map, yaw)


            # print("Pose",x_r, y_r, theta_r)

            # robot_x_map = x_r
            # robot_y_map = y_r
            # yaw = theta_r


            print("Robot position", robot_x_map,robot_y_map)


            # Helper: convert world (map) coords -> local map indices
            def world_to_map_ixyz(wx, wy):
                # mx = int((wx - self.local_origin_x) / self.local_map_resolution)
                # my = int((wy - self.local_origin_y) / self.local_map_resolution)

                mx = int((wx - self.origin_x) / self.local_map_resolution)
                my = int((wy - self.origin_y) / self.local_map_resolution)
                return mx, my

            # 4. For each beam, compute the end coordinates
            angle = msg.angle_min

            print("Minimum range", msg.range_min)
            for r in msg.ranges:

                if r < msg.range_min:
                    continue  # skip this beam

                if np.isinf(r):
                    r = msg.range_max  # treat inf as max range
                beam_angle = angle + yaw

                # End of beam in world
                end_x = robot_x_map + r * np.cos(beam_angle)
                end_y = robot_y_map + r * np.sin(beam_angle)

                # Convert to map indices
                rx, ry = world_to_map_ixyz(robot_x_map, robot_y_map)
                ex, ey = world_to_map_ixyz(end_x, end_y)

                # Get cells along the line from (rx, ry) to (ex, ey)
                line_cells = self.bresenham_line(rx, ry, ex, ey)

                # If r < msg.range_max * some_factor, we consider the last cell an obstacle
                beam_hit_obstacle = (r < msg.range_max * 0.99)

                # Mark free along the line, except possibly the last cell
                for (cx, cy) in line_cells[:-1]:
                    if 0 <= cx < self.local_map_width and 0 <= cy < self.local_map_height:
                        old_val = self.local_map_data[self.hypothesis_id][cy, cx]
                        # Only set to free if it wasn't already occupied
                        if old_val != 100:
                            self.local_map_data[self.hypothesis_id][cy, cx] = 0

                # Mark the last cell
                last_cx, last_cy = line_cells[-1]
                if (0 <= last_cx < self.local_map_width) and (0 <= last_cy < self.local_map_height):
                    if beam_hit_obstacle:
                        # discovered an obstacle
                        self.local_map_data[self.hypothesis_id][last_cy, last_cx] = 100
                    else:
                        old_val = self.local_map_data[self.hypothesis_id][last_cy, last_cx]
                        # Only set free if it wasn't already occupied
                        if old_val != 100:
                            self.local_map_data[self.hypothesis_id][last_cy, last_cx] = 0


                angle += msg.angle_increment


            # Publish the updated map for this hypothesis
            self.publish_updated_map(h, self.local_map_data[h])


            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            resolution = self.map_info.resolution

            # self.column = int((x_r - origin_x) / resolution)
            # self.row = int((y_r - origin_y) / resolution)

            self.column = int((robot_x_map - origin_x) / resolution)
            self.row = int((robot_y_map - origin_y) / resolution)

            # Convert goal position to grid indices
            self.goal_column = int((self.goal_x - origin_x) / resolution)
            self.goal_row = int((self.goal_y - origin_y) / resolution)
            self.goal_indices = (self.goal_row, self.goal_column)

            updated_exploration_map, frontiers_middle = self.exploration(self.local_map_data[h], self.map_info.width, self.map_info.height, resolution, self.column, self.row, origin_x, origin_y)

            print("Frontiers ", frontiers_middle)


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

        # # Publish the centroids to a topic
        self.publish_centroids(centroids, resolution, originX, originY)

        return data, centroids



    def merge_cell(self,old_val, new_val):
        """
        Example rules for merging one cell from local_map_data (old_val)
        with updated_map (new_val).
        Return the merged value.
        """
        # If the new cell is -1 => unknown, keep old value (unless old was also -1).
        if new_val == -1:
            # For example, keep the old_val if it is known
            return old_val  

        # If new cell is free (0) and old_val was unknown (-1) => now it's discovered free
        if new_val == 0:
            if old_val == -1:
                return 0  # we discovered it is free
            # If old_val is 0 or 100, you can decide how to handle:
            # e.g., if old_val was 100 but new says 0 => conflict, pick a policy
            # For now, let's trust new_val if old_val != 100
            if old_val == 100:
                # conflict; do we trust old or new? 
                # let's suppose we trust old if we previously had it as occupied
                return 100
            return 0

        # If new cell is 100 => discovered occupied
        if new_val == 100:
            # If old was -1 or 0, override with 100
            return 100

        # Otherwise, fallback
        return new_val

    def yaw_from_quaternion(self, quaternion):
        """ Converts a quaternion to yaw (theta) """
        import math
        import tf_transformations
        euler = tf_transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return euler[2]  # Yaw

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

        # print("Pose array", pose_array)
        self.get_logger().info("Publishing centroids PoseArray:")
        for i, p in enumerate(pose_array.poses, start=1):
            self.get_logger().info(
                f" - Pose #{i}: x={p.position.x:.2f}, y={p.position.y:.2f}, "
                f"qw={p.orientation.w:.2f}"
        )

        # Publish the centroids
        self.centroids_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} centroids to /frontier_centroids")


    def get_color(self, group_id):
        # Generate colors based on group_id
        np.random.seed(group_id)
        color = np.random.rand(3)
        return color
    


    def publish_updated_map(self, hypothesis_id, updated_map):
        """ Publishes the updated local map as an OccupancyGrid for a given hypothesis """

        updated_occupancy_grid = OccupancyGrid()
        updated_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        updated_occupancy_grid.header.frame_id = f"hypothesis_{hypothesis_id}"  # Include hypothesis ID
        updated_occupancy_grid.header.frame_id = "map"


        updated_occupancy_grid.info = self.map_info  # Use the existing map metadata
        updated_occupancy_grid.data = updated_map.flatten().astype(int).tolist()  # Convert numpy to list

        # Ensure we have a publisher for this hypothesis
        topic_name = f"/updated_hypothesis_map_{hypothesis_id}"
        if hypothesis_id not in self.map_publishers:

            self.map_publishers[hypothesis_id] = self.create_publisher(OccupancyGrid, topic_name, 10)
            self.get_logger().info(f"Created publisher for {topic_name}")

        # Publish the updated map
        self.map_publishers[hypothesis_id].publish(updated_occupancy_grid)
        self.get_logger().info(f"Published updated local map for hypothesis {hypothesis_id} on {topic_name}")



def main(args=None):
    rclpy.init(args=args)
    node = HypothesisMapSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
