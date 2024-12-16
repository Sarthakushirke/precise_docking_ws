import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Quaternion,  Point
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import math
import random
import tf_transformations
from copy import deepcopy
from active_localization.pf_localisation.util import rotateQuaternion, getHeading
from active_localization.pf_localisation.sensor_model import SensorModel

class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization_node')

        # Declare parameters
        self.declare_parameter('publish_delta', 0.1)
        self._PUBLISH_DELTA = self.get_parameter('publish_delta').get_parameter_value().double_value

        self.map_data = None
        self.resolution = None
        self.originX = None
        self.originY = None
        self.width = None
        self.height = None
        self.data = None
        self.num_particles = 1000  # number of particles

        self._initial_pose_received = False
        self._particles_initialized = False

        # Create publishers
        self._cloud_publisher = self.create_publisher(PoseArray, '/particlecloud', 10)

        # Create subscribers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("GlobalLocalizationNode started, waiting for map...")

    def map_callback(self, msg):
        """
        Once a map is received, store it and initialize particles globally.
        """
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

        self.get_logger().info(f"Map received: {self.width}x{self.height}, resolution {self.resolution}")
        # Initialize particles over the entire free space
        self.particlecloud = self.initialise_particle_cloud_from_map()
        self._cloud_publisher.publish(self.particlecloud)
        self._particles_initialized = True
        self.get_logger().info("Particle cloud initialized globally.")

    def initialise_particle_cloud_from_map(self):
        """
        Initialize the particle cloud distributed across all free areas of the map.
        
        We will:
        - Extract all free cells (data == 0)
        - Randomly select cells from them to assign to particles
        """
        free_cells = []
        # Identify free cells
        for y in range(self.height):
            for x in range(self.width):
                idx = x + y * self.width
                if self.data[idx] == 0:  # free space
                    free_cells.append((x, y))

        if len(free_cells) == 0:
            self.get_logger().error("No free cells in the map! Cannot initialize particles.")
            return PoseArray()

        poseArray = PoseArray()
        poseArray.header.frame_id = "map"
        for i in range(self.num_particles):
            # choose a random free cell
            x_cell, y_cell = random.choice(free_cells)
            # convert cell to world coordinates
            wx = x_cell * self.resolution + self.originX + (self.resolution / 2.0)
            wy = y_cell * self.resolution + self.originY + (self.resolution / 2.0)

            pose = Pose()
            pose.position.x = wx
            pose.position.y = wy

            theta = random.uniform(-math.pi, math.pi)
            q = tf_transformations.quaternion_from_euler(0,0,theta)
            pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            poseArray.poses.append(pose)

        return poseArray

    def odom_callback(self, msg):
        # If particles not initialized or map not received, return
        if not self._particles_initialized:
            return
        # Implement odometry-based predict step here in a full particle filter approach
        pass

    def scan_callback(self, msg):
        # If no particles initialized, can't do measurement update
        if not self._particles_initialized:
            return
        # Store laser scan; implement measurement update in a full solution
        pass

    def update_particle_cloud(self, scan):
            """
            This should use the supplied laser scan to update the current
            particle cloud. i.e. self.particlecloud should be updated.

            :Args:
                | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

            """
            # --- Initially do not update the particles if no movement has been detected

            # Keep the latest odom as the previous one
            # prev_odom_list = [
            #     self.prev_odom_x == self.latest_odom_x,
            #     self.prev_odom_y == self.latest_odom_y,
            #     self.prev_odom_heading == self.latest_odom_heading
            # ]

            # # If: all are True (not 0), do nothing (the robot can move)
            # if all(prev_odom_list):
            #     return

            # # Else: the previous odom is still the latest one (so, do not move)
            # self.latest_odom_x = self.prev_odom_x
            # self.latest_odom_y = self.prev_odom_y
            # self.latest_odom_heading = self.prev_odom_heading

            # ---------------------------------------------------------------

            # Initializing array of particles/poses
            pose = self.particlecloud

            # Create a list of weights
            weights = []

            # Compute the likelihood weight for each particle
            for i, p in enumerate(pose.poses):
                # get the likelihood weighting
                likhweight = SensorModel.get_weight(scan, p)  # laser scan and position
                weights.append(likhweight)  # add that weight into the list

            total = sum(weights)  # calculate the sum of the weights

            # Normalize weights of the "weight" list
            # (total contribution sums up to 1 - decrease variance - higher accuracy in prediction)
            norm = [w / total for w in weights]

            # Total number of particles
            M = self.num_particles

            # Generate the Cumulative distribution function, using the above normalized weights
            cdf = []
            for i, n in enumerate(norm):
                if i == 0:
                    cdf.append(n)  # add the first weight by default
                else:
                    cdf.append(cdf[-1] + n)  # c[i] = c[i-1] + w, last c[i] will be equal to 1

            # --- Resampling - Make sure each particle contributes proportional to its weight

            # Set parameters
            thold = 1 / M  # threshold = M^(-1)
            u = random.uniform(0, thold)  # distribution between 0 and threshold
            i = 0  # for the while loop
            poseArray = PoseArray()  # Setting a new array of poses/particles

            # For each particle
            for _ in range(M):

                # Every particle with normalized weight over 1/N is guaranteed to be selected at least once
                # If u < cdf, means that there are no more contributions for this particle
                # (exhausted all its chances, no more contributions for the next sample set)
                # So, skip until the next threshold reached
                while u > cdf[i]:
                    i = i + 1

                # Reached a particle that has weight higher than u, thus, it can make contributions
                # So, we are getting that particle from the cloud
                particles = self.particlecloud.poses[i]

                # Take its position and orientation
                x = particles.position.x
                y = particles.position.y
                q = particles.orientation
                t = getHeading(q)  # Get heading (in radians) described by a given orientation

                # Set a new random location proportional to the normalized weight of the current particle
                # (random floating point number with gaussian distribution)
                rx = random.gauss(x, norm[i])
                ry = random.gauss(y, norm[i])
                rt = random.gauss(t, norm[i])

                # Create a new particle/pose with a new location and orientation
                # by using the above (existing particle's) info --> resampling applies
                rPoint = Point(rx, ry, 0.0)  # z usually is 0
                rotateQ = rotateQuaternion(q, rt - t)
                newPose = Pose(rPoint, rotateQ)

                # If the particle appear once, increment threshold
                # This declares if the contributions of the current particle are exhausted or not
                u = u + thold

                # Add the new pose
                poseArray.poses.append(newPose)

            # Update/Replace the existing particles/poses with the new ones on the cloud
            self.particlecloud = poseArray



    

def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
