import rclpy
from rclpy.node import Node
import tf_transformations
import tf2_ros
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class OccupancyGridUpdater(Node):
    def __init__(self):
        super().__init__('occupancy_grid_updater')

        self.get_logger().info('Node initialized and ready')

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 100)

        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)

        self.get_logger().info('Subscribed to map')
        # self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Publishers
        # self.map_pub = self.create_publisher(OccupancyGrid, '/local_map', 10)
        
        # TF buffer and listener
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.map_data = None

    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

        self.get_logger().info(f'ok with map callback, map data length: {len(self.data)}')


    def odom_callback(self,msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.get_logger().info('ok!')




    # def lidar_callback(self, msg):
    #     if self.map_data is None:
    #         return

    #     try:
    #         # Get transform from 'base_link' (robot frame) to 'map' (global frame)
    #         transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         self.get_logger().error(f"Transform lookup failed: {e}")
    #         return

    #     # Transform LIDAR points to map frame
    #     ranges = np.array(msg.ranges)
    #     angle_min = msg.angle_min
    #     angle_increment = msg.angle_increment
    #     max_range = msg.range_max

    #     # Calculate reachable cells
    #     reachable = np.zeros_like(self.map_data, dtype=bool)
    #     robot_x = transform.transform.translation.x
    #     robot_y = transform.transform.translation.y
    #     yaw = tf_transformations.euler_from_quaternion([
    #         transform.transform.rotation.x,
    #         transform.transform.rotation.y,
    #         transform.transform.rotation.z,
    #         transform.transform.rotation.w])[2]

    #     for i, r in enumerate(ranges):
    #         if np.isfinite(r) and r < max_range:
    #             angle = angle_min + i * angle_increment + yaw
    #             x = robot_x + r * np.cos(angle)
    #             y = robot_y + r * np.sin(angle)

    #             # Convert lidar (x, y) to map grid coordinates
    #             map_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
    #             map_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

    #             # Ensure map_x and map_y are within the grid boundaries
    #             if 0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height:
    #                 reachable[map_y, map_x] = True

    #     # Update the map as in your original code
    #     updated_map = self.map_data.copy()
    #     free_space_mask = (updated_map == 0) & ~reachable
    #     updated_map[free_space_mask] = -1  # Mark free, unreachable cells as -1

    #     # Publish the updated map
    #     updated_occupancy_grid = OccupancyGrid()
    #     updated_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
    #     updated_occupancy_grid.header.frame_id = "map"
    #     updated_occupancy_grid.info = self.map_info
    #     updated_occupancy_grid.data = updated_map.flatten().tolist()
    #     self.map_pub.publish(updated_occupancy_grid)


def main(args=None):
    rclpy.init(args=args)  # Start ROS2 communication
    local_node = OccupancyGridUpdater()  # Create a node
    rclpy.spin(local_node)
    local_node.destroy_node()
    rclpy.shutdown()  # Shutting down ROS2 communication

if __name__ == '__main__':
    main() 