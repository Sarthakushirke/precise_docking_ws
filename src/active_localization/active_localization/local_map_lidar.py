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
        
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     '/rosbot_base_controller/odom',
        #     self.odom_callback,
        #     10)
        
        
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
        self.local_map_data = None

    def map_callback(self, msg):
        self.map_info = msg.info
        # Convert the map data to a 2D NumPy array
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_info.height, self.map_info.width))
        # self.resolution = self.map_info.resolution

        self.local_map_width = self.map_info.width
        self.local_map_height = self.map_info.height
        self.local_map_resolution = self.map_info.resolution


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
        # 1. If we haven't initialized a local map array yet, do so

        if self.local_map_data is None:
            # Suppose we want a 100x100 grid, each cell = 5cm, all unknown:

            # # Center the robot in the middle of this grid
            self.local_origin_x = - ( self.local_map_width * self.local_map_resolution) / 2.0
            self.local_origin_y = - ( self.local_map_height * self.local_map_resolution) / 2.0

            # self.local_origin_x = self.x 
            # self.local_origin_y = self.y 

            # Initialize the local_map_data with -1 (unknown)
            # self.local_map_data = np.full((self.local_map_height, self.local_map_width), -1, dtype=np.int8)
            self.local_map_data = np.full((self.local_map_height, self.local_map_width), -1, dtype=np.int8)

        # 2. Clear or partially clear local_map each time. For a rolling local map, you might keep old data:
        # self.local_map_data[:] = -1

        # 3. Lookup the transform from 'map' (or 'odom', or 'base_link' – depends on your usage)
        try:
            transform = self.tf_buffer.lookup_transform('map', 
                                                        msg.header.frame_id, 
                                                        rclpy.time.Time(seconds=0))
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return
        
        # Robot pose in the local map
        robot_x_map = transform.transform.translation.x
        robot_y_map = transform.transform.translation.y
        yaw = tf_transformations.euler_from_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])[2]

        # Helper: convert world (map) coords -> local map indices
        def world_to_map_ixyz(wx, wy):
            mx = int((wx - self.local_origin_x) / self.local_map_resolution)
            my = int((wy - self.local_origin_y) / self.local_map_resolution)
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


            # print("Line cells", line_cells)
            print("Map width and height", self.local_map_width,self.local_map_height)

            # If r < msg.range_max * some_factor, we consider the last cell an obstacle
            # beam_hit_obstacle = (r < msg.range_max * 0.99)
            beam_hit_obstacle = (r < msg.range_max * 0.99)

            # Mark free along the line, except possibly the last cell
            for (cx, cy) in line_cells[:-1]:
                if 0 <= cx < self.local_map_width and 0 <= cy < self.local_map_height:
                    self.local_map_data[cy, cx] = 0  # free
            
            # Mark the last cell as occupied if we think we hit an obstacle
            last_cx, last_cy = line_cells[-1]
            if (0 <= last_cx < self.local_map_width) and (0 <= last_cy < self.local_map_height):
                if beam_hit_obstacle:
                    self.local_map_data[last_cy, last_cx] = 100  # obstacle
                else:
                    # didn't actually hit an obstacle, so we consider that unknown or free
                    # Typically you might mark it as free, or just do nothing 
                    self.local_map_data[last_cy, last_cx] = 0  # free



            # Mark free along the line, except possibly the last cell
            # for (cx, cy) in line_cells[:-1]:
            #     if 0 <= cx < self.local_map_width and 0 <= cy < self.local_map_height:
            #         old_val = self.local_map_data[cy, cx]
            #         # Only set to free if it wasn't already occupied
            #         if old_val != 100:
            #             self.local_map_data[cy, cx] = 0

            # # Mark the last cell
            # last_cx, last_cy = line_cells[-1]
            # if (0 <= last_cx < self.local_map_width) and (0 <= last_cy < self.local_map_height):
            #     if beam_hit_obstacle:
            #         # discovered an obstacle
            #         self.local_map_data[last_cy, last_cx] = 100
            #     else:
            #         old_val = self.local_map_data[last_cy, last_cx]
            #         # Only set free if it wasn't already occupied
            #         if old_val != 100:
            #             self.local_map_data[last_cy, last_cx] = 0


            angle += msg.angle_increment

        # 5. Publish this local map
        local_occupancy_grid = OccupancyGrid()
        local_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        local_occupancy_grid.header.frame_id = "map"  # or "odom", or something
        local_occupancy_grid.info.resolution = self.local_map_resolution
        local_occupancy_grid.info.width = self.local_map_width
        local_occupancy_grid.info.height = self.local_map_height
        local_occupancy_grid.info.origin.position.x = self.local_origin_x
        local_occupancy_grid.info.origin.position.y = self.local_origin_y

        local_occupancy_grid.data = self.local_map_data.flatten().tolist()

        self.map_pub.publish(local_occupancy_grid)




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

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
