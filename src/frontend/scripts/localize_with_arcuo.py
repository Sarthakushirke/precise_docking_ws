#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
import heapq , math , random , yaml
import time
from rclpy.clock import Clock

from threading import Thread
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point, Twist
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from active_localization.next_best_view import check_visible_objects_from_centroid_simple
from sensor_msgs.msg import LaserScan


speed = 0.2  # Linear speed (m/s)
lookahead_distance = 0.5  # Lookahead distance for pure pursuit (meters)
robot_r = 0.5  # Robot radius for obstacle detection (meters)
target_error = 0.1  # Acceptable error to consider goal reached (meters)

class MultiLocationMarkerNode(Node):
    def __init__(self):
        super().__init__('multi_location_marker_node')


        #For Gazebo model diff_drive.sdf
        # marker_map[marker_id] => list of (x_map, y_map, theta_map)
        # self.marker_map = {
        #     0: [
        #         # (3.0, -1.0, 0.0),
        #         (3.0, 4.0, 0.0),
        #         (-6.0, 5.0, 0.0),
        #         (-3.0,-4.0,0.0),
        #     ],
        # }

        #For Gazebo model extra_features.sdf
        self.marker_map = {
            0: [
    
                (5.0, -4.0, 0.0),
                (3.0, 2.0, 0.0),
                (5.0,7.0,0.0),
                (-1.0,7.0,0.0),
                (-3.0,-5.0,0.0),
                (-11.0,5.0,0.0),
            ],
        }

        # self.hypothesis_weights = {}
        # for marker_id, poses_list in self.marker_map.items():
        #     n_hypotheses = len(poses_list)
        #     prior = 1.0 / n_hypotheses
        #     self.hypothesis_weights[marker_id] = [prior] * n_hypotheses

            
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
        
        # Odometry subscription (uses default QoS)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.get_logger().info('Subscribed to /odom topic')
        
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info('Subscribed to /scan topic')

        self.centroid_count_true = self.create_subscription(PoseArray, '/frontier_centroids_original', self.frontier_callback, 10)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/hypothesis_map', 10)

        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Publishing velocity commands on /cmd_vel')

        self.centroid_pub = self.create_publisher(MarkerArray, '/centroid_markers', 10)
        self.get_logger().info('Publishing centroid markers on /centroid_markers')

        # Publisher for the path marker
        self.path_pub = self.create_publisher(Marker, '/path_marker', 10)
        self.get_logger().info('Publishing path markers on /path_marker')

        # Publisher for centroids
        self.centroids_pub = self.create_publisher(PoseArray, '/frontier_centroids', 10)
        self.get_logger().info('Publishing centroids on /frontier_centroids')

        # Store current hypotheses (marker_id => list of (x_r, y_r, theta_r))
        self.hypotheses_dict = {}

        self.predicted_distances = {} 

        # Hardcode a single marker ID for demonstration
        self.current_marker_id = 0
        # self.stored_marker_z_values = []

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
        # self.get_logger().info('Publishing frontier markers on /frontier_markers')

        self.map_data = None
        self.map_info = None
        self.column = None
        self.row = None
        self.goal_column = None
        self.goal_row = None
        self.global_centroid = []
        self.all_frontiers_info = {}  # dict of i -> list of frontier dicts
        # Initialize the hypothesis local map data as a dictionary
        self.local_map_data = {}

        self.goal_x = 13.0  # Goal x-coordinate in meters
        self.goal_y = 0.0  # Goal y-coordinate in meters

        self.path = None  # Path to follow
        self.control_thread = None  # Thread for control loop
        self.control_active = False  # Flag to control the thread
        self.i = 0  # Index for pure pursuit
        self.scan_data = None  # Latest laser scan data
        self.centroid_goal_path = None
        self.end_loop = False
        self.global_start = True

        # Initialize robot position attributes
        self.x = None
        self.y = None

        #Original scan centroids
        self.orginal_scan_centroids = None

        self.riviz_publish = True
        self.detection = True

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


    def frontier_callback(self, msg: PoseArray):
        """
        Called each time a PoseArray is received on /frontier_centroids.
        We can log how many frontier poses are in the array
        and optionally do further processing.
        """
        self.orginal_scan_centroids = len(msg.poses)
        self.get_logger().info(f"Received {self.orginal_scan_centroids} frontier centroids.")
   

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

        print("I am in marker callback")

        # if self.global_start is True:
        #     print("start", self.global_start)
             
        self.compute_and_plan(msg)




    def compute_and_plan(self,msg):

        self.marker_pose = msg

        self.stored_marker_z_values = []
         
        if self.marker_pose.point.z not in self.stored_marker_z_values:
            self.stored_marker_z_values.append(round(self.marker_pose.point.z* 10.0, 2))

        print("self.stored_marker_z_values", len(self.stored_marker_z_values))
        print("self.stored_marker_z_values:", [z for z in self.stored_marker_z_values])

        if self.map_data is None:
            self.get_logger().warn('Map data not available yet')
            return
        
        if self.x is None or self.y is None:
            self.get_logger().warn("Robot position not yet received.")
            return
        
        if self.orginal_scan_centroids is None:
            self.get_logger().warn("orginal scan centroids not yet received.")
            return
        
        # If the robot is currently moving towards a goal, skip computation
        # if self.end_loop is True:
        if self.control_active:
            self.get_logger().info("Robot is moving towards a goal, skipping computation.")
            return

        
        
        # Hardcode the marker ID in this example
        marker_id = self.current_marker_id

        # If the marker does not exist in the map dictionary, ignore
        if marker_id not in self.marker_map:
            self.get_logger().warn(
                f"Marker ID {marker_id} not found in marker_map. Ignoring.")
            return

        if self.global_start:

            # For demonstration, let's just assume a fixed distance=2
            distance = 2.0

            self.expansion_size = 3
            self.min_group_size = 40

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
        # self.publish_pose_array(new_hypotheses)

        # # --- PUBLISH MARKER ARRAY (Squares) FOR RVIZ ---
        # self.publish_square_markers(new_hypotheses)
        self.global_start = False

        if len(self.hypotheses_dict[0]) != 1 :
            self.compute_again(new_hypotheses)


        if len(self.hypotheses_dict[0]) == 1:

            print("In the desitnation path")

            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            resolution = self.map_info.resolution
            # Convert goal position to grid indices
            self.goal_column = int((self.goal_x - origin_x) / resolution)
            self.goal_row = int((self.goal_y - origin_y) / resolution)
            self.goal_indices = (self.goal_row, self.goal_column)

            robot_destination_column = int((self.hypotheses_dict[0][0][0] - origin_x) / resolution)
            robot_destination_row = int((self.hypotheses_dict[0][0][1] - origin_y) / resolution)

            print("Robot indices", robot_destination_column, robot_destination_row )

            bot_indices = (robot_destination_row, robot_destination_column)

            # # Compute the path from centroid to goal
            destination_path = self.astar(self.map_data, bot_indices, self.goal_indices)

            
            if destination_path:

                print("Got the final path")

                desination_go = [(p[1] * resolution + origin_x, p[0] * resolution + origin_y) for p in destination_path ]  

                self.publish_path_marker(desination_go)
                     
                # self.final_robot_loop(self.hypotheses_dict[0][0][0],self.hypotheses_dict[0][0][1],desination_go)


    def compute_again(self,new_hypotheses):


        num_beams = 16000
        angle_increment = 0.0003927233046852052
        scan_ranges = []
        start_angle = -3.14
        max_range = 10

        # reachable = np.zeros_like(self.map_data, dtype=bool)
        self.all_frontiers_info = {}

        for h, (x_r, y_r, theta_r) in enumerate(new_hypotheses, start=1):

            # Re-init arrays for each hypothesis
            reachable = np.zeros_like(self.map_data, dtype=bool)
           
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

             # Store or update the local_map_data for this hypothesis
            if h not in self.local_map_data:
                self.local_map_data[h] = updated_map
            else:
                # Merge with existing local map
                height, width = self.local_map_data[h].shape
                for r in range(height):
                    for c in range(width):
                        old_val = self.local_map_data[h][r, c]
                        new_val = updated_map[r, c]
                        self.local_map_data[h][r, c] = self.merge_cell(old_val, new_val)

            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            resolution = self.map_info.resolution

            self.column = int((x_r - origin_x) / resolution)
            self.row = int((y_r - origin_y) / resolution)

            # Convert goal position to grid indices
            self.goal_column = int((self.goal_x - origin_x) / resolution)
            self.goal_row = int((self.goal_y - origin_y) / resolution)
            self.goal_indices = (self.goal_row, self.goal_column)

            # frontiers_middle = self.exploration(updated_map, self.map_info.width, self.map_info.height, resolution, self.column, self.row, origin_x, origin_y)

            updated_exploration_map, frontiers_middle = self.exploration(self.local_map_data[h], self.map_info.width, self.map_info.height, resolution, self.column, self.row, origin_x, origin_y)

            print("Frontiers ", frontiers_middle)

            # Reassign it back
            self.local_map_data[h] = updated_exploration_map

            ######################

            centroids_info = []
            # Filter hypotheses based on the frontier condition
            filtered_hypotheses = []

            for frontier_id, (centroid_x, centroid_y) in frontiers_middle.items():

                # print("Length of frontiers",len(frontiers_middle))

                if self.orginal_scan_centroids == len(frontiers_middle):

                    filtered_hypotheses.append((x_r, y_r, theta_r))

                    # Update the hypotheses with the filtered list
                    new_hypotheses = filtered_hypotheses

                    centroid_indices = (centroid_x, centroid_y)

                    # Compute the path from centroid to goal
                    path = self.astar(self.map_data, centroid_indices, self.goal_indices)

                    # Calculate visible objects from this centroid

                    centroid_visible_x = centroid_x * resolution + origin_x
                    centroid_visible_y = centroid_y * resolution + origin_y
                    print("Coordinates of the centroild in ",centroid_visible_y,centroid_visible_x)
                    visible_objects = check_visible_objects_from_centroid_simple( centroid_visible_y, centroid_visible_x)
                    num_visible_objects = len(visible_objects)

                    if path:
                        # path_length = len(path) * resolution

                        self.centroid_goal_path = [(p[1] * resolution + origin_x, p[0] * resolution + origin_y) for p in path] 

                        self.publish_path_marker( self.centroid_goal_path)

                        # print("The centroild to goal path", self.centroid_goal_path)
                        path_length = self.pathLength(self.centroid_goal_path)

                        # Weights for utility calculation
                        weight_info_gain = 1.0
                        weight_path_length = 1.0

                        # Compute utility
                        utility_value = (weight_info_gain * num_visible_objects) - (weight_path_length * path_length)

                        self.get_logger().info(
                            f"Path length from centroid at ({centroid_x:.2f}, {centroid_y:.2f}) to goal: {path_length:.2f}, "
                            f"utility is {utility_value:.2f},"
                            f"Num of visible objects is {num_visible_objects:.2f},")

                        # Store information for utility calculation
                        centroids_info.append({
                            'centroid_x': centroid_x,
                            'centroid_y': centroid_y,
                            'path': path,
                            'path_length': path_length,
                            'utility': utility_value,
                            'num_visible_objects': num_visible_objects,
                            'centroid_indices': centroid_indices
                        })
                    else:
                        self.get_logger().warn(f"No path found from centroid at ({centroid_x:.2f}, {centroid_y:.2f}) to goal.")

                    

            # store list of frontiers for hypothesis i
            self.all_frontiers_info[h] = centroids_info  


        #Pick one hypothesis and one path (TO DO) #OUTPUT: 1 hypothses which might not be the real one and the path to that 
        # hypotheses best centroid.
        # AFTER processing all hypotheses, pick best overall
        best_global_utility = -float('inf')
        best_global_hypothesis = None
        best_global_frontier_info = None

        for hyp_idx, flist in self.all_frontiers_info.items():
            for f_info in flist:
                if f_info['utility'] > best_global_utility:
                    best_global_utility = f_info['utility']
                    best_global_hypothesis = hyp_idx
                    best_global_frontier_info = f_info

        if best_global_frontier_info:
            # We have the best frontier and which hypothesis it belongs to
            self.get_logger().info(
                f"BEST => Hyp #{best_global_hypothesis}, frontier=({best_global_frontier_info['centroid_x']:.2f}, "
                f"{best_global_frontier_info['centroid_y']:.2f}) utility={best_global_utility:.2f}"
            )

        self.best_global_hypothesis = best_global_hypothesis

        best_global_frontier_column =  best_global_frontier_info['centroid_x'] 
       
        best_global_frontier_row = best_global_frontier_info['centroid_y'] 

        # best_global_frontier_column = int((best_global_frontier_info['centroid_x'] - origin_x) / resolution)
        # best_global_frontier_row = int((best_global_frontier_info['centroid_y'] - origin_y) / resolution)
        best_global_frontier_indices = (best_global_frontier_column, best_global_frontier_row)


        # robot_column = int((self.x - origin_x) / resolution)
        # robot_row = int((self.y - origin_y) / resolution)
        #Example for other hypothesis 
        # robot_column = int((3 - origin_x) / resolution)
        # robot_row = int((7 - origin_y) / resolution)

        print("Global frontier row", best_global_frontier_row )

        print("global frontier column ",best_global_frontier_column)

        robot_column = int((self.hypotheses_dict[0][best_global_hypothesis-1][0] - origin_x) / resolution)
        robot_row = int((self.hypotheses_dict[0][best_global_hypothesis-1][1] - origin_y) / resolution)

        robot_indices = (robot_row, robot_column) #This should be one of the hypothesis
        #Also the best_centroid_indices should that particular hypothesis centroid.

        path_robot_to_centroid = self.astar(self.map_data, robot_indices, best_global_frontier_indices)


        if path_robot_to_centroid:
            # Convert path indices to coordinates
            self.path = [(p[1] * resolution + origin_x, p[0] * resolution + origin_y) for p in path_robot_to_centroid]

            # print("Path robot to the centroid", self.path)
            # Publish the path
            self.publish_path_marker(self.path)

            # Start the control thread
            if not self.control_active:
                self.control_active = True
                self.control_thread = Thread(target=self.control_loop)
                self.control_thread.start()


        # Publish the updated map
        updated_occupancy_grid = OccupancyGrid()
        updated_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        updated_occupancy_grid.header.frame_id = "map"
        updated_occupancy_grid.info = self.map_info
        updated_occupancy_grid.data = updated_map.flatten().tolist()
        self.map_pub.publish(updated_occupancy_grid)



    def odom_callback(self,msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

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
        # self.get_logger().info("Path marker published!")


    def control_loop(self):
        # Keep track of old pose
        old_x, old_y, old_theta = self.x, self.y ,self.yaw

        # self.get_logger().info("Control loop started.")
        #Example for other hypothesis 
        hyp_pose_x = self.hypotheses_dict[0][self.best_global_hypothesis-1][0]
        hyp_pose_y = self.hypotheses_dict[0][self.best_global_hypothesis-1][1]
        hyp_pose_theta = 0
        twist = Twist()
        # dt = 0.09  # your loop rate is time.sleep(0.1)
        while self.control_active:

            
            # 1) get updated real robot pose in map frame
            new_x, new_y, new_theta = self.x, self.y ,self.yaw

            # 2) compute difference
            dx = new_x - old_x
            dy = new_y - old_y
            dtheta = new_theta - old_theta

            # update old pose
            old_x, old_y, old_theta = new_x, new_y, new_theta

            v, w = self.local_control()
            #Example for other hypothesis 
            hyp_pose_x = hyp_pose_x + dx
            hyp_pose_y = hyp_pose_y + dy
            hyp_pose_theta = hyp_pose_theta + dtheta
            # wrap heading
            hyp_pose_theta = (hyp_pose_theta + math.pi) % (2 * math.pi) - math.pi
            if v is None:
                # v, w, self.i = self.pure_pursuit(self.x, self.y, self.yaw, self.path, self.i)
                #Example for other hypothesis 
                v, w, self.i = self.pure_pursuit(hyp_pose_x, hyp_pose_y, hyp_pose_theta, self.path, self.i)
            # Check if goal is reached
            # if self.i >= len(self.path) - 1 and self.distance_to_point(self.x, self.y, self.path[-1][0], self.path[-1][1]) < target_error:
            #Example for other hypothesis 
            if self.i >= len(self.path) - 1 and self.distance_to_point(hyp_pose_x, hyp_pose_y, self.path[-1][0], self.path[-1][1]) < target_error:
                v = 0.0 
                w = 0.0
                self.control_active = False
                self.get_logger().info("Goal reached.")
                
            twist.linear.x = v
            twist.angular.z = w
            self.velocity_pub.publish(twist)

            self.apply_pose_diff_to_hypotheses(dx, dy, dtheta)

            if self.riviz_publish is True:
                # --- PUBLISH POSE ARRAY (Arrows) FOR RVIZ ---
                self.publish_pose_array(self.hypotheses_dict)

                # --- PUBLISH MARKER ARRAY (Squares) FOR RVIZ ---
                self.publish_square_markers(self.hypotheses_dict)
            
            # Store the previous z values
            previous_marker_z_values = getattr(self, 'previous_marker_z_values', [])

            print("self.stored_marker_z_values", len(self.stored_marker_z_values))
            print("self.stored_marker_z_values:", [z for z in self.stored_marker_z_values])

            # Check for drastic change (e.g., significant difference in average or max difference)
            if previous_marker_z_values and len(self.hypotheses_dict[0]) != 1:
                # Calculate element-wise difference
                diff_list = [abs(curr - prev) for curr, prev in zip(self.stored_marker_z_values, previous_marker_z_values)]
                
                print("Element-wise Differences:", diff_list)

                # Define a threshold for "drastic" change
                threshold = 0.5  # Adjust this threshold as per your application
                
                if any(diff > threshold for diff in diff_list):
                    print("Drastic change detected, starting measurement model...")
                    self.measurement_model()
                else:
                    print("No significant change detected.")
            else:
                # Run the measurement model the first time
                if len(self.hypotheses_dict[0]) != 1:
                    print("First measurement, starting measurement model...")
                    self.measurement_model()

            # Update previous_marker_z_values for the next comparison
            self.previous_marker_z_values = self.stored_marker_z_values.copy()

            # Multiply old weight by likelihood
            # updated_weights = []
            # for i in range(len(self.marker_map[0])):
            #     old_w = self.hypothesis_weights[0][i]
            #     new_w = old_w * likelihoods[i]
            #     updated_weights.append(new_w)

            # # Normalize
            # w_sum = sum(updated_weights)
            # if w_sum > 1e-9:
            #     updated_weights = [w / w_sum for w in updated_weights]

            # # Store them back
            # self.hypothesis_weights[0] = updated_weights

            # -- APPLY THE SAME MOTION TO YOUR HYPOTHESES --
            # self.hypotheses_dict = self.apply_motion_to_hypotheses(v, w, dt)

            # Print the best hypotheses for each measured distance
            # print("\nBest Hypotheses for each Measured Distance:")
            # for measured_dist, data in best_hypotheses.items():
            #     print(f"Measured Distance: {measured_dist}")
            #     print(f"  Best Hypothesis: {data['hypothesis']}")
            #     print(f"  Likelihood: {data['likelihood']}")

            # --- PUBLISH POSE ARRAY (Arrows) FOR RVIZ ---
            self.riviz_publish = False

            print("Hypotheses dictnary",self.hypotheses_dict)

            self.publish_pose_array(self.hypotheses_dict)

            # --- PUBLISH MARKER ARRAY (Squares) FOR RVIZ ---
            self.publish_square_markers(self.hypotheses_dict)

            time.sleep(0.1)  # Control loop rate (10 Hz)
        self.get_logger().info("Control loop ended.")


        if len(self.hypotheses_dict[0]) != 1:

            self.remove_out_of_bounds_hypotheses()

        print("control active ", self.control_active)

        self.publish_pose_array(self.hypotheses_dict)

        # --- PUBLISH MARKER ARRAY (Squares) FOR RVIZ ---
        self.publish_square_markers(self.hypotheses_dict)


        if self.control_active is False and len(self.hypotheses_dict[0]) != 1:

            list_from_dict = self.hypotheses_dict[0]
            self.compute_again(list_from_dict)


    
        
    def apply_pose_diff_to_hypotheses(self, dx, dy, dtheta):
        new_hypotheses_dict = {}
        for marker_id, hypotheses in self.hypotheses_dict.items():
            updated_list = []
            for (hx, hy, htheta) in hypotheses:
                hx_new = hx + dx
                hy_new = hy + dy
                htheta_new = htheta + dtheta
                # wrap heading
                htheta_new = (htheta_new + math.pi) % (2 * math.pi) - math.pi
                updated_list.append((hx_new, hy_new, htheta_new))
            new_hypotheses_dict[marker_id] = updated_list

        self.hypotheses_dict = new_hypotheses_dict

    def remove_out_of_bounds_hypotheses(self):
        """
        Removes hypotheses that are out of the map bounds from hypotheses_dict.
        """
        valid_hypotheses_dict = {}
        map_width = self.map_info.width
        map_height = self.map_info.height
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        # Define map boundaries in world coordinates
        map_x_min = origin_x
        map_x_max = origin_x + map_width * resolution
        map_y_min = origin_y
        map_y_max = origin_y + map_height * resolution

        for marker_id, hypotheses in self.hypotheses_dict.items():
            valid_hypotheses = []
            for (x_r, y_r, theta_r) in hypotheses:
                # Check if hypothesis is within map bounds
                if map_x_min <= x_r < map_x_max and map_y_min <= y_r < map_y_max:
                    valid_hypotheses.append((x_r, y_r, theta_r))
                else:
                    self.get_logger().warn(
                        f"Hypothesis ({x_r:.2f}, {y_r:.2f}) is out of map bounds and will be removed."
                    )
            if valid_hypotheses:
                valid_hypotheses_dict[marker_id] = valid_hypotheses

        self.hypotheses_dict = valid_hypotheses_dict
        self.get_logger().info(f"Updated hypotheses_dict: {self.hypotheses_dict}")



    def measurement_model(self):
        # Store hypotheses per measurement
        measurement_best_hypotheses = []

        for measured_dist in self.stored_marker_z_values:

            print("measured_distance", measured_dist)

            # best_likelihood = float('-inf')  # Initialize with negative infinity
            # best_hypothesis = None

            current_best_hypotheses = []  # Store best hypotheses for this measurement
            best_likelihood = float('-inf')  # Start with negative infinity

            for marker_id, hypotheses in self.hypotheses_dict.items():
                # retrieve the list of marker poses for that marker_id
                if marker_id not in self.marker_map:
                    self.get_logger().warn(f"Marker ID {marker_id} not found in marker_map. Skipping.")
                    continue
                
                marker_poses_list = self.marker_map[marker_id] 

                for marker_pose in marker_poses_list:

                    print("Marker pose:", marker_pose)

                    for hypothesis in hypotheses:
                        # hypothesis = (x_r, y_r, theta_r)
                        x_r, y_r, theta_r = hypothesis
                        print("Hypothesis:", hypothesis)
                        
                        if hypothesis:
                            output_obj = check_visible_objects_from_centroid_simple(x_r, y_r)
                            print("Object returned from check_visible_objects_from_centroid_simple:", output_obj)

                            # Check if the marker pose is in output_obj
                            if (marker_pose[0], marker_pose[1]) in output_obj:
                                print("Marker pose found in output_obj:", marker_pose)
                                # Perform further actions if needed
                                
                                dist_pred = self.predict_marker_measurement(hypothesis, marker_pose)
                                
                                sigma_distance = 0.4 
                                likelihood_hyp = self.compute_likelihood(measured_dist,dist_pred,sigma_distance)

                                print("Predicted Distance:", dist_pred)
                                print("Liklihood hyp:", likelihood_hyp)

                                # Store all hypotheses with the same maximum likelihood
                                if likelihood_hyp > best_likelihood:
                                    best_likelihood = likelihood_hyp
                                    current_best_hypotheses = [(hypothesis, likelihood_hyp)]
                                elif likelihood_hyp == best_likelihood:
                                    current_best_hypotheses.append((hypothesis, likelihood_hyp))
                                
                            else:
                                print("Marker pose not found, checking next hypothesis...")

                        # # Update the best hypothesis if this one is better
                        # if likelihood_hyp > best_likelihood:
                        #     best_likelihood = likelihood_hyp
                        #     best_hypothesis = hypothesis

            
            # print(f"Best Hypotheses for Measurement {round(measured_dist * 10.0, 2)}:")
            # for hyp, likelihood in current_best_hypotheses:
            #     print(f"  Hypothesis: {hyp}, Likelihood: {likelihood}")
                # Store current best hypotheses for this measurement

            # Ensure current_best_hypotheses is added to measurement_best_hypotheses
            if current_best_hypotheses:
                measurement_best_hypotheses.append(current_best_hypotheses)
                print("Measurement Best Hypotheses:", measurement_best_hypotheses)
            else:
                self.get_logger().warn("No valid hypotheses found for this measurement.")



        # Compare hypotheses across measurements to find common ones
        print("self.stored_marker_z_values", len(self.stored_marker_z_values))
        if len(self.stored_marker_z_values) != 1:
            if len(measurement_best_hypotheses) == len(self.stored_marker_z_values):
                common_hypotheses = set(hyp for hyp, _ in measurement_best_hypotheses[0])
                
                for measurement in measurement_best_hypotheses[1:]:
                    current_hypotheses_set = set(hyp for hyp, _ in measurement)
                    common_hypotheses &= current_hypotheses_set  # Keep only common hypotheses

                # If there are common hypotheses, find the one with the highest cumulative likelihood
                final_best_hypothesis = None
                highest_combined_likelihood = float('-inf')

                for common_hypothesis in common_hypotheses:
                    cumulative_likelihood = sum(
                        likelihood for measurement in measurement_best_hypotheses
                        for hyp, likelihood in measurement if hyp == common_hypothesis
                    )
                    if cumulative_likelihood > highest_combined_likelihood:
                        highest_combined_likelihood = cumulative_likelihood
                        final_best_hypothesis = common_hypothesis

                print("\nFinal Best Hypothesis Across Measurements:")
                print(f"  Hypothesis: {final_best_hypothesis}, Combined Likelihood: {highest_combined_likelihood}")

                # Keep only the final best hypothesis in self.hypotheses_dict
                for marker_id, hypotheses in self.hypotheses_dict.items():
                    if final_best_hypothesis in hypotheses:
                        print(f"Keeping only Hypothesis: {final_best_hypothesis} for Marker ID: {marker_id}")
                        self.hypotheses_dict[marker_id] = [final_best_hypothesis]
                        break  # Once updated, stop iterating
            else:
                print("Not enough measurements to compare hypotheses across them.")
        else:
            print("More than one hypothesis detected:", len(measurement_best_hypotheses[0]))
            if len(measurement_best_hypotheses[0]) == 1: 
                # # Keep only the final best hypothesis in self.hypotheses_dict
                actual_best_hypothesis = measurement_best_hypotheses[0][0][0]
                print("Actual hypothesis ", actual_best_hypothesis)
                for marker_id, hypotheses in self.hypotheses_dict.items():
                    print("Hypothes",hypotheses)
                    print("Matches:", hypotheses == actual_best_hypothesis)
                    if actual_best_hypothesis in hypotheses:
                        print(f"Keeping only Hypothesis: {actual_best_hypothesis} for Marker ID: {marker_id}")
                        self.hypotheses_dict[marker_id] = [actual_best_hypothesis]
                        break  # Once updated, stop iterating


    def final_robot_loop(self,robot_x,robot_y,robot_theta,journey_path):
        # Keep track of old pose
        old_x, old_y, old_theta = self.x, self.y ,self.yaw

        # self.get_logger().info("Control loop started.")
        #Example for other hypothesis  
        hyp_pose_x = robot_x
        hyp_pose_y = robot_y
        hyp_pose_theta = robot_theta
        twist = Twist()
       
        while self.control_active:

            # 1) get updated real robot pose in map frame
            new_x, new_y, new_theta = self.x, self.y ,self.yaw

            # 2) compute difference
            dx = new_x - old_x
            dy = new_y - old_y
            dtheta = new_theta - old_theta

            # update old pose
            old_x, old_y, old_theta = new_x, new_y, new_theta

            v, w = self.local_control()
            #Example for other hypothesis 
            hyp_pose_x = hyp_pose_x + dx
            hyp_pose_y = hyp_pose_y + dy
            hyp_pose_theta = hyp_pose_theta + dtheta
            # wrap heading
            hyp_pose_theta = (hyp_pose_theta + math.pi) % (2 * math.pi) - math.pi
            if v is None:
                # v, w, self.i = self.pure_pursuit(self.x, self.y, self.yaw, self.path, self.i)
                #Example for other hypothesis 
                v, w, self.i = self.pure_pursuit(hyp_pose_x, hyp_pose_y, hyp_pose_theta,journey_path , self.i)
            # Check if goal is reached
            # if self.i >= len(self.path) - 1 and self.distance_to_point(self.x, self.y, self.path[-1][0], self.path[-1][1]) < target_error:
            #Example for other hypothesis 
            if self.i >= len(self.path) - 1 and self.distance_to_point(hyp_pose_x, hyp_pose_y, journey_path[-1][0], journey_path[-1][1]) < target_error:
                v = 0.0 
                w = 0.0
                self.control_active = False
                self.get_logger().info("Goal reached.")
                
            twist.linear.x = v
            twist.angular.z = w
            self.velocity_pub.publish(twist)

    
    def local_control(self):
        v = None
        w = None
        if self.scan_data is None:
            return v, w  # No scan data available
        # Convert LaserScan ranges to numpy array
        scan_ranges = np.array(self.scan_data.ranges)
        # Check front area for obstacles
        front_angles = np.concatenate((scan_ranges[:30], scan_ranges[-30:]))  # Front 60 degrees
        if np.any(front_angles < robot_r):
            v = 0.0
            w = -math.pi / 4
        else:
            # Check right side
            right_angles = scan_ranges[300:360]  # Right 60 degrees
            if np.any(right_angles < robot_r):
                v = 0.0
                w = math.pi / 4
        return v, w
    

    def pure_pursuit(self, current_x, current_y, current_heading, path, index):
        closest_point = None
        v = speed
        for i in range(index, len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = self.distance_to_point(current_x, current_y, x, y)
            if distance >= lookahead_distance:
                closest_point = (x, y)
                index = i
                break
        if closest_point is not None:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        else:
            target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
            index = len(path) - 1
        desired_steering_angle = target_heading - current_heading
        desired_steering_angle = (desired_steering_angle + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
        # Limit the steering angle
        if abs(desired_steering_angle) > math.pi / 6:
            v = 0.0
            desired_steering_angle = math.copysign(math.pi / 6, desired_steering_angle)
        return v, desired_steering_angle, index
    
    def distance_to_point(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)
    
    def pathLength(self,path):
        for i in range(len(path)):
            path[i] = (path[i][0],path[i][1])
            points = np.array(path)
        differences = np.diff(points, axis=0)
        distances = np.hypot(differences[:,0], differences[:,1])
        total_distance = np.sum(distances)
        return total_distance
    
    def lidar_callback(self, msg):
        self.scan_data = msg
        self.scan = msg.ranges

        # Define the safe distance and the front scan range
        min_distance = 0.5  # Minimum safe distance in meters
        front_angle_range = 30  # Degrees (e.g., ±30° around the front)

        # LiDAR scan parameters
        num_readings = len(self.scan)  # Total number of LiDAR readings
        angle_increment = msg.angle_increment  # Angular resolution of each scan in radians
        front_indices = int(front_angle_range / (angle_increment * 180 / math.pi))  # Convert angle to index range

        # Extract front-facing LiDAR readings (center ± front_angle_range)
        center_index = num_readings // 2
        front_scan = self.scan[center_index - front_indices:center_index + front_indices]

        # Check for collisions in the front scan
        too_close = any(distance < min_distance for distance in front_scan if distance > 0)

        if too_close:
            self.get_logger().warn("Warning: Obstacle detected in front!")
            # Example: Stop the robot if too close
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_pub.publish(twist)
            
        else:
            self.get_logger().info("Front is clear.")


    def compute_likelihood(self,
        distance_measured: float,
        distance_expected: float,
        sigma_distance: float,
    ) -> float:
        """
        Computes the likelihood of a measurement given the predicted measurement
        using a simple Gaussian sensor model in distance and orientation.
        
        :param distance_measured:   The distance observed by the sensor (e.g. from robot to marker)
        :param distance_expected:   The distance we expect if this hypothesis is correct
        :param sigma_distance:      Std dev for distance measurement noise
        :return: A likelihood value in [0, 1].
        """

        # 1) Compute errors
        error_d = distance_measured - distance_expected

        # 2) Compute Gaussian likelihood for distance
        # L_d = exp(-(error_d^2) / (2*sigma_distance^2))
        if sigma_distance <= 0:
            # Avoid division by zero or negative
            likelihood_distance = 1.0 if abs(error_d) < 1e-9 else 0.0
        else:
            likelihood_distance = math.exp(-(error_d**2) / (2.0 * sigma_distance**2))

        # 4) Combine (assuming independence => multiply)
        L = likelihood_distance

        return L

    def predict_marker_measurement(self,hyp_pose, marker_pose):
        """
        Predict the (distance, bearing) you'd see for 'marker_pose'
        from the robot's hypothesized pose 'hyp_pose' in a 2D map.

        :param hyp_pose: (x_r, y_r, theta_r)
        :param marker_pose: (x_m, y_m)
        :return: (dist_pred, bearing_pred)
        """
        x_r, y_r, theta_r = hyp_pose
        x_m, y_m, theta_map = marker_pose

        dx = x_m - x_r
        dy = y_m - y_r

        dist_pred = math.sqrt(dx*dx + dy*dy)
        # angle_global = math.atan2(dy, dx)
        # bearing_pred = angle_global - theta_r

        # # Wrap bearing to [-pi, +pi], optional
        # bearing_pred = (bearing_pred + math.pi) % (2*math.pi) - math.pi

        return dist_pred


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



    def publish_pose_array(self, hypotheses_dict):
        """
        Publishes a PoseArray where each pose corresponds to one hypothesis.
        These will appear as arrows in RViz if you add a 'PoseArray' display.
        """
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "map"  # Hypotheses are in the 'map' frame

        for marker_id, hypotheses in hypotheses_dict.items():
            
            for hypothesis in hypotheses:

                x_r, y_r, theta_r = hypothesis
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


    def publish_square_markers(self, hypotheses_dict):
        """
        Publishes a MarkerArray with squares (cubes) to represent each hypothesis.
        Each marker is a small, flat cube.
        """
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()

        # delete_all_marker = Marker()
        # delete_all_marker.action = Marker.DELETEALL
        # marker_array.markers.append(delete_all_marker)
        marker_array.markers.clear()


        for marker_id, hypotheses in hypotheses_dict.items():

            for i, hypothesis in enumerate(hypotheses):

                x_r, y_r, theta_r = hypothesis
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

    def destroy_node(self):
    # Stop the control thread if active
        self.get_logger().info("Destroying node and stopping control thread.")
        if self.control_active:
            self.control_active = False
            if self.control_thread is not None:
                self.control_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiLocationMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





    # def apply_motion_to_hypotheses(self, v, w, dt):
    #     """
    #     Apply the same (v, w) motion to each hypothesis for time dt.
    #     """

    #     new_hypotheses_dict = {}

    #     for marker_id, hypotheses in self.hypotheses_dict.items():
    #         hypotheses_motion_new = []
    #         for h in hypotheses:
    #             x, y, theta = h

    #             # Compute new pose with simple 2D kinematics
    #             x_new = x + v * math.cos(theta) * dt
    #             y_new = y + v * math.sin(theta) * dt
    #             theta_new = theta + w * dt

    #             # x_new = x + v * math.cos(theta) 
    #             # y_new = y + v * math.sin(theta) 
    #             # theta_new = theta + w 
    #             # Wrap theta to [-pi, pi]
    #             theta_new = (theta_new + math.pi) % (2 * math.pi) - math.pi

    #             hypotheses_motion_new.append((x_new, y_new, theta_new))

    #         # Update each marker's hypotheses in the new dictionary
    #         new_hypotheses_dict[marker_id] = hypotheses_motion_new

    #     # Update the original dictionary in place
    #     self.hypotheses_dict = new_hypotheses_dict

    #     return self.hypotheses_dict