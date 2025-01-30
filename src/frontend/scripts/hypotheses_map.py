#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray

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

        # Subscribe to hypothesis maps dynamically (will be updated in pose callback)
        self.map_subs = {}
        self.hypothesis_id = 6
        self.map_publishers = {}  # Dictionary to store publishers for hypothesis maps
        self.start = True


    def pose_callback(self, msg):
        """ Receives hypotheses poses from /pose_array and updates self.hypotheses_dict """

        

        self.hypotheses_dict.clear()

        # Latched QoS (transient local) to get last published map even if published before subscribing
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        print("In the pose callback")

        for pose in msg.poses:
            # Extract hypothesis ID from pose data (assuming ID is stored in position.z)
        
            x = pose.position.x
            y = pose.position.y
            theta = self.yaw_from_quaternion(pose.orientation)
            self.hypotheses_dict[0] = (x, y, theta)

            print("Hypotheses",self.hypotheses_dict)

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

        # Store map metadata (assuming all maps share the same info)
        self.map_info = msg.info


    def map_callback(self, msg):
        self.map_info = msg.info

        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_info.height, self.map_info.width))
        


    def lidar_callback(self, msg):
        """ Processes LIDAR scan data and updates hypothesis maps """
        if not self.hypotheses_dict or not self.local_map_data:
            self.get_logger().warn("No hypotheses or local maps available yet.")
            return

        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        max_range = msg.range_max

        # Replace 'inf' values with max_range
        ranges = np.where(np.isinf(ranges), max_range, ranges)

        h = 6

        for d, (x_r, y_r, theta_r) in self.hypotheses_dict.items():
            # if h not in self.local_map_data:
            #     self.get_logger().warn(f"Map for hypothesis {h} not found.")
            #     continue

            print("Pose",x_r, y_r, theta_r)

            self.get_logger().info(f"Processing LIDAR update for hypothesis {h}")

            reachable = np.zeros_like(self.map_data, dtype=bool)

            for i, r in enumerate(ranges):
                if np.isfinite(r):
                    # Compute global beam angle
                    angle = angle_min + i * angle_increment + theta_r
                    x_end = x_r + r * np.cos(angle)
                    y_end = y_r + r * np.sin(angle)

                    # Convert to grid coordinates
                    map_x0 = int((x_r - self.map_info.origin.position.x) / self.map_info.resolution)
                    map_y0 = int((y_r - self.map_info.origin.position.y) / self.map_info.resolution)
                    map_x1 = int((x_end - self.map_info.origin.position.x) / self.map_info.resolution)
                    map_y1 = int((y_end - self.map_info.origin.position.y) / self.map_info.resolution)

                    line_cells = self.bresenham_line(map_x0, map_y0, map_x1, map_y1)
                    for map_x, map_y in line_cells:
                        if 0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height:
                            reachable[map_y, map_x] = True
                        else:
                            self.get_logger().debug(f"Computed cell ({map_x}, {map_y}) is out of bounds.")

            # Update local map
            
            updated_map = self.map_data.copy()
            free_cells = (updated_map == 0)
            unreachable = ~reachable
            free_and_unreachable = free_cells & unreachable
            updated_map[free_and_unreachable] = -1
            reachable_free = free_cells & reachable
            updated_map[reachable_free] = 0


            # Merge updates into the hypothesis's local map
            if h not in self.local_map_data:
                print("I couldn't find any self.local_map_data")
                self.local_map_data[h] = updated_map
            else:
                # Merge the updated map with the existing local map for this hypothesis
                height, width = self.local_map_data[h].shape
                for r in range(height):
                    for c in range(width):
                        old_val = self.local_map_data[h][r, c]
                        new_val = updated_map[r, c]
                        merged = self.merge_cell(old_val, new_val)
                        self.local_map_data[h][r, c] = merged



            # Publish the updated map for this hypothesis
            self.publish_updated_map(h, self.local_map_data[h])



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
