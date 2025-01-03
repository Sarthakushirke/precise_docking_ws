import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import tf_transformations
import tf2_ros
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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
            qos_profile)
        self.get_logger().info('Subscribed to /map topic with TRANSIENT_LOCAL durability')

        # Odometry subscription (uses default QoS)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.get_logger().info('Subscribed to /odom topic')

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info('Subscribed to /scan topic')

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/local_map', 10)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_data = None
        self.map_info = None
        

    def map_callback(self, msg):
        self.map_info = msg.info
        # Convert the map data to a 2D NumPy array
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_info.height, self.map_info.width))
        # self.get_logger().info('Map callback triggered')
        # np.set_printoptions(threshold=np.inf)
        # print("Map data", self.map_data)

    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.get_logger().debug(f'Odom callback triggered, x: {self.x}, y: {self.y}')


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
    

    def lidar_callback(self, msg):
        if self.map_data is None:
            self.get_logger().warn('Map data not available yet')
            return

        try:
            # Get transform from 'base_link' (robot frame) to 'map' (global frame)
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time(seconds=0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return

        # Transform LIDAR points to map frame
        ranges = np.array(msg.ranges)
        # print("Ranges ", ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        max_range = msg.range_max

        # Replace 'inf' values with max_range
        ranges = np.where(np.isinf(ranges), max_range, ranges)

        # safety_margin = 5
        # max_valid_range = max_range - safety_margin

        # Calculate reachable cells
        reachable = np.zeros_like(self.map_data, dtype=bool)
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        yaw = tf_transformations.euler_from_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w])[2]

        for i, r in enumerate(ranges):
            # and r < max_valid_range
            if np.isfinite(r) :
                angle = angle_min + i * angle_increment + yaw
                x_end = robot_x + r * np.cos(angle)
                y_end = robot_y + r * np.sin(angle)

                # Convert start and end positions to map grid indices
                map_x0 = int(np.floor((robot_x - self.map_info.origin.position.x) / self.map_info.resolution))
                map_y0 = int(np.floor((robot_y - self.map_info.origin.position.y) / self.map_info.resolution))
                map_x1 = int(np.floor((x_end - self.map_info.origin.position.x) / self.map_info.resolution))
                map_y1 = int(np.floor((y_end - self.map_info.origin.position.y) / self.map_info.resolution))

                # Get all grid cells along the LIDAR beam path
                line_cells = self.bresenham_line(map_x0, map_y0, map_x1, map_y1)

                # print("Mapx and mapy",map_x,map_y)

                # print(f"Line cells from ({map_x0}, {map_y0}) to ({map_x1}, {map_y1}): {line_cells}")

                for map_x, map_y in line_cells:
                    # Ensure map_x and map_y are within the grid boundaries
                    if 0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height:
                        reachable[map_y, map_x] = True
                    else:
                        self.get_logger().debug(f"Computed cell ({map_x}, {map_y}) is out of bounds.")


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
        # self.get_logger().info('Published updated map')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
