#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import PointStamped, PoseArray, Pose
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class MultiLocationMarkerNode(Node):
    def __init__(self):
        super().__init__('multi_location_marker_node')

        # marker_map[marker_id] => list of (x_map, y_map, theta_map)
        self.marker_map = {
            0: [
                # (3.0, -1.0, 0.0),
                (-6.0, 5.0, 0.0),
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

        self.map_data = None
        self.map_info = None
        # self.resolution = None
        # self.originX = None
        # self.originY = None
        # self.width = None
        # self.height = None

        self.get_logger().info('MultiLocationMarkerNode started.')


    def map_callback(self, msg):
        self.map_info = msg.info

        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_info.height, self.map_info.width))

        # Ensure self.x and self.y are defined
        if not hasattr(self, 'x') or not hasattr(self, 'y'):
            self.get_logger().warn('Robot position not yet received.')
            return

        # column = int((self.x - self.originX) / self.resolution)
        # row = int((self.y - self.originY) / self.resolution)



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

        # We assume zero relative orientation
        phi_marker_relative = 0.0

        # Retrieve the list of known marker positions/orientations in the map
        marker_locations = self.marker_map[marker_id]
        new_hypotheses = []

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

        # --- PUBLISH POSE ARRAY (Arrows) FOR RVIZ ---
        self.publish_pose_array(new_hypotheses)

        # --- PUBLISH MARKER ARRAY (Squares) FOR RVIZ ---
        self.publish_square_markers(new_hypotheses)

        num_beams = 16000
        angle_increment = 0.0003927233046852052
        scan_ranges = []
        start_angle = -3.14
        max_range = 10

        reachable = np.zeros_like(self.map_data, dtype=bool)

        for i, (x_r, y_r, theta_r) in enumerate(new_hypotheses, start=1):
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
                reachable[map_y, map_x] = True
            else:
                self.get_logger().debug(f"Computed cell ({map_x}, {map_y}) is out of bounds.")


        return reachable
        
        # # 4) Traverse cells
        # for (cx, cy) in line_cells:
        #     # Check if (cx, cy) is within map bounds
        #     if (cx < 0 or cx >= map_info.width or cy < 0 or cy >= map_info.height):
        #         # we've left the map; means no obstacle found within the map
        #         # => distance = max_range
        #         return max_range
            
        #     # Check occupancy
        #     if occupancy_grid[cy][cx] == 1:  # or > 50 for probability
        #         # obstacle found
        #         # compute distance from (robot_x, robot_y) to center of (cx, cy)
        #         # or optionally to the boundary
        #         obstacle_world_x = origin_x + (cx + 0.5) * resolution
        #         obstacle_world_y = origin_y + (cy + 0.5) * resolution

        #         dist = math.sqrt((obstacle_world_x - robot_x)**2 + 
        #                         (obstacle_world_y - robot_y)**2)
        #         return min(dist, max_range)
        
        # # If we never found an obstacle in the entire line, then distance = max_range
        # return max_range





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
