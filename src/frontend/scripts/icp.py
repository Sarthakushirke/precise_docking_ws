#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Point32, Twist, PoseWithCovarianceStamped, TransformStamped
from d_msgs.msg import KeyframeGraph, Keyframe, WeightedEdge
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster


from rcl_interfaces.srv import GetParameters

from d_autonomous_exploration.RobotHandler import RobotHandler
from d_autonomous_exploration.KeyframeHandler import KeyframeHandler
from d_autonomous_exploration.GraphHandler import GraphHandler
from d_autonomous_exploration import tools
from d_autonomous_exploration.tools import ANSI_COLORS as colorCode

from tf_transformations import quaternion_from_euler

from scipy.spatial import cKDTree
import pygeos as pg
from shapely.geometry import Point
import struct
import open3d as o3d
import numpy as np
import pygeos as pg
import networkx as nx
import pickle,  copy, time, math
from collections import deque
from threading import Lock




class Robot_Localization(Node, RobotHandler, GraphHandler):
    
    def __init__(self):

        super().__init__(
            node_name='robot_localization_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
            )
        
        RobotHandler.__init__(self)
        GraphHandler.__init__(self, self.laser_params, self.robot_params)

        self._initialize()
        self.__get_node_parameters__()

        self.map_pcd = None
        self.initial_pose = Point32(x=0.0, y=0.0, z=0.0)
        self.current_trans_estimate = None
        self.T_prev = None
    
        self.estimated_pose = None
    
        self.callback_lock = Lock()

        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.estimated_pose_pub = self.create_publisher(PoseStamped, 'estimated_pose', self.qos)
        
        self.map_pcd_pub = self.create_publisher(PointCloud2, 'map_pcd', self.qos)
        self.current_pcd_pub = self.create_publisher(PointCloud2, 'current_pcd', self.qos)
        self.transformed_pcd_pub = self.create_publisher(PointCloud2, 'transformed_pcd', self.qos)


        self.__setup_subscribers__()


        self.create_timer(1.0/self.rate, self.__synchronization_thread, callback_group=MutuallyExclusiveCallbackGroup())

        # self.create_timer(1.0/self.rate, self.publish_map_to_odom, callback_group=MutuallyExclusiveCallbackGroup())


    def _initialize(self):

        self.laser_queue = deque(maxlen=5)
        self.laser_msg = None
        self.pose_queue = deque(maxlen=5) 
        self.pose_msg = None
        self.current_time = None

        self.timeout = 0.04

        self.node_count = None
        
        self.cmd_vel = Twist()

        self.temp_keyframe_graph = nx.Graph()
        

    def _get_laser_param_from_file(self):
        try:
            with open('/home/dharshan/thesis_ws/src/dharshan_ros_humble_pkgs/file_server/laser_params.dat', 'rb') as file:
                self.laser_params = pickle.load(file)
                # print("Laser params loaded")
        except Exception as e:
            self.get_logger().info(f'Failed to read file: {str(e)}')

    def __get_node_parameters__(self):
        
        # Get namespace
        self.namespace = self.get_namespace()

        # Get World Parameters        
        try:    
            self.declare_parameter('world_model', 'large_office')
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.world_model = self.get_parameter('world_model').value
        
        # Get robot control parameters
        self.client = self.create_client(GetParameters, '/robocyl/planar_twist_teleop_key/get_parameters')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for node1 to start the parameter service...')
        self.req = GetParameters.Request()
        self.req.names = ['max_linear_speed', 'max_angular_speed']
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.__handle_response_node__)


        #  Get rate parameter
        try:    
            self.declare_parameter('rate', 10.0)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.rate = self.get_parameter('rate').value

        # Get frame IDs
        try:    
            self.declare_parameter('map_frame_id', 'map')
            self.declare_parameter('odom_frame_id', 'odom')
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.map_frame_id = self.namespace + '/' + self.map_frame_id
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.odom_frame_id = self.namespace + '/' + self.odom_frame_id

        self.get_logger().info(f'{colorCode["cyan"]}Map Frame ID: {self.map_frame_id}, Odom frame ID: {self.odom_frame_id}')  


    def __handle_response_node__(self, future):
        try:
            response = future.result()
            robot_params = {
                "robot_radius": 0.25,
                "max_linear_speed": 0.5,
                "max_angular_speed": 2.0,
                "robot_frame_id": "base_link"
                }
            
            robot_params['max_linear_speed'] = response.values[0].double_value
            robot_params['max_angular_speed'] = response.values[1].double_value

            self.robot_params.set_params(robot_params)

        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    def __setup_subscribers__(self):
        # Get topics parameters
        self.topics_parameters = self.get_parameters_by_prefix('topics')
        self.topics = {}

        for key, value in self.topics_parameters.items():
            key_elements = key.split('.')
            topic_temp = self.topics.setdefault(key_elements[0], {})
            topic_temp[key_elements[1]] = value

        for key, value in self.topics.items():
            # Topic time initialization
            if key == "laserScan":
                # Create and register joy topic subscriber with the node
                self.create_subscription(
                    LaserScan,
                    value['topic'].value,
                    lambda msg, 
                    topic=key: self.__laser_callback(msg, topic),
                    self.qos,
                    callback_group=MutuallyExclusiveCallbackGroup()
                    )
                
            if key == "position":
                self.create_subscription(
                    PoseStamped,
                    value['topic'].value,
                    lambda msg, 
                    topic=key: self.__pose_callback(msg, topic),
                    self.qos,
                    callback_group=MutuallyExclusiveCallbackGroup()
                    )
                
            if key == "keyframeGraph":
                self.create_subscription(
                    KeyframeGraph,
                    value['topic'].value,
                    lambda msg, 
                    topic=key: self.__keyframe_graph_callback(msg, topic),
                    self.qos,
                    callback_group=MutuallyExclusiveCallbackGroup()              
                    )

    def __pose_callback(self, msg, topic):
        if msg is not None:
                self.pose_msg = msg
                if self.laser_msg is not None:
                    pose_time = tools.get_time_from_stamp(msg.header.stamp)
                    # self.get_logger().info(f"Pose Time: {pose_time}")
                    self.pose_queue.append((msg, pose_time))
                
    def __laser_callback(self, msg, topic):
        if msg is not None:
                self.laser_msg = msg
                if self.pose_msg is not None:
                    laser_time = tools.get_time_from_stamp(msg.header.stamp)
                    # self.get_logger().info(f"Laser Time: {laser_time}")
                    self.laser_queue.append((msg, laser_time))

    def __keyframe_graph_callback(self, msg, topic):
        self.node_count = len(self.keyframe_node_list)
        if msg is not None:
            self.setup_GraphHandler(msg)


    def __synchronization_thread(self):
        while True:
            if len(self.pose_queue) > 0 and len(self.laser_queue) > 0:
                pose_msg, pose_time = self.pose_queue[0]
                laser_msg, laser_time = self.laser_queue[0]
                if abs(pose_time - laser_time) < self.timeout:
                    self.current_time = laser_time
                    self.perform_localization(pose_msg, laser_msg)
                    self.publish_map_to_odom()
                    # self.get_logger().info("~~~~~~~~~~~SYNCED~~~~~~~~~~~")
                    # self.augmented_graph_pub.publish(KeyframeGraph())
                    self.pose_queue.popleft()
                    self.laser_queue.popleft()
                elif pose_time < laser_time:
                    self.pose_queue.popleft()
                else:
                    self.laser_queue.popleft()


    def publish_map_to_odom(self):

        # print(f"TF: {self.initial_pose.x}, {self.initial_pose.y}")
        if self.initial_pose is not None:
            # print("Pose publisher", self.robot_pose)
            map2odom = TransformStamped()
            map2odom.header.stamp = tools.get_stamp_from_time(self.current_time)
            map2odom.header.frame_id = self.map_frame_id
            map2odom.child_frame_id = self.odom_frame_id
            map2odom.transform.translation.x =  -self.initial_pose.x
            map2odom.transform.translation.y =  -self.initial_pose.y
            map2odom.transform.translation.z = 0.0
            
            # z is used for rotation
            q = quaternion_from_euler(0.0, 0.0, -self.initial_pose.z)
            map2odom.transform.rotation.x = q[0]
            map2odom.transform.rotation.y = q[1]
            map2odom.transform.rotation.z = q[2]
            map2odom.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(map2odom)

    def publish_estimated_pose(self):
        if self.estimated_pose is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = tools.get_stamp_from_time(self.current_time)
            pose_msg.header.frame_id = self.map_frame_id
            pose_msg.pose.position.x = self.estimated_pose.x
            pose_msg.pose.position.y = self.estimated_pose.y
            pose_msg.pose.position.z = 0.0
            
            q = quaternion_from_euler(0.0, 0.0, self.estimated_pose.z)
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            
            self.estimated_pose_pub.publish(pose_msg)


    def keyframe_msg_to_keyframe_handle(self, keyframe_msg: Keyframe):
        
        temp_keyframe_handle = KeyframeHandler(self.laser_params, self.robot_params)    
        temp_keyframe_handle.set_center(keyframe_msg.center)
        temp_keyframe_handle.set_keyframe_id(keyframe_msg.keyframe_id)
        temp_keyframe_handle.set_control_point_cloud(keyframe_msg.control_point_cloud)
    
        temp_keyframe_handle.set_point_cloud(keyframe_msg.point_cloud)
        
        polygon_vertices = []
        control_polygon_vertices = []
        hit_points = []
        
        for id in range(len(keyframe_msg.hit_flag)):
            
            control_polygon_vertices.append((keyframe_msg.control_point_cloud[id].x, keyframe_msg.control_point_cloud[id].y))
            polygon_vertices.append((keyframe_msg.point_cloud[id].x, keyframe_msg.point_cloud[id].y))

            if keyframe_msg.hit_flag[id]:
                hit_points.append([keyframe_msg.point_cloud[id].x, keyframe_msg.point_cloud[id].y])
        
        if control_polygon_vertices:
            temp_keyframe_handle.set_control_polygon(pg.polygons([polygon_vertices]))
        
        if polygon_vertices:
            temp_keyframe_handle.set_cloud_polygon(pg.polygons([polygon_vertices]))
        
        if hit_points:
            temp_keyframe_handle.set_hit_points(hit_points)
        
        return temp_keyframe_handle

    def generate_point_cloud(self, hit_points):
        pcd_msg = PointCloud2()
        pcd_msg.header.frame_id = self.map_frame_id
        pcd_msg.header.stamp = self.get_clock().now().to_msg()
        pcd_msg.width = len(hit_points)
        pcd_msg.height = 1
        pcd_msg.is_dense = False
        pcd_msg.is_bigendian = False
        pcd_msg.point_step = 12  # 3 * 4 bytes for float32 fields x, y, z
        pcd_msg.row_step = pcd_msg.point_step * pcd_msg.width

        pcd_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        points = []
        for point in hit_points:
            points.append(struct.pack('fff', point[0], point[1], 0.0))

        pcd_msg.data = b''.join(points)


        return pcd_msg
    

    def keyframe_handler_to_pcd(self, keyframe_handle: KeyframeHandler):

        # pcd_arr = np.array(keyframe_handle.get_hit_points(), dtype=np.float64)
        # pcd_arr_3d = np.c_[pcd_arr, np.zeros(pcd_arr.shape[0])]
        pcd_arr_3d = []
        for point in keyframe_handle.get_hit_points():
            pcd_arr_3d.append([point[0], point[1], 0.0])
        
        pcd_arr_3d = np.array(pcd_arr_3d, dtype=np.float64)

        pcd = o3d.geometry.PointCloud()
        if pcd_arr_3d.size > 0:
            pcd.points = o3d.utility.Vector3dVector(pcd_arr_3d)
            
        else:
            self.get_logger().info(f'{colorCode["cyan"]}No points in the point cloud')

        return pcd



    def setup_GraphHandler(self, msg):
        keyframe_list = msg.keyframe_nodes

        if not self.keyframe_node_list:
            self._get_laser_param_from_file()
        
        if keyframe_list:
            # self.get_logger().info(f'diff: {len(keyframe_list) - len(self.keyframe_node_list)}')
            if keyframe_list[-1].keyframe_id >= self.node_count:
                # for i in range(len(keyframe_list)):
                self.callback_lock.acquire()
                self.get_logger().info(f'{colorCode["cyan"]}******* Creating new keyframe element: {keyframe_list[-1].keyframe_id} *******')
                new_node = self.keyframe_msg_to_keyframe_handle(keyframe_list[-1])
                new_node.map_pcd = self.keyframe_handler_to_pcd(new_node)

                self.keyframe_node_list.append(new_node)
                self.keyframe_graph.add_node(keyframe_list[-1].keyframe_id)                        

                graph_edges = msg.keyframe_edges
                if len(graph_edges) > 0:
                    for edge in graph_edges:
                        self.keyframe_graph.add_edge(edge.from_node, edge.to_node, weight=edge.weight)
                
                self.callback_lock.release()



    def perform_icp(self, source, target, threshold=5.0, max_iteration=100):
        print("++++++++++++++++++++Performing ICP++++++++++++++++++++")
        icp_result = o3d.pipelines.registration.registration_icp(
            source, target, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration)
        )
        return icp_result
    

    def hit_points_from_laserScan(self, laser_msg, robot_pose):
        
        min_angle = self.laser_params.get_laser_min_angle()
        max_angle = self.laser_params.get_laser_max_angle()
        max_range = self.laser_params.get_laser_max_range()
        laser_samples = self.laser_params.get_laser_samples()
        
        angles = np.linspace(min_angle, max_angle, laser_samples)

        hit_points = []

        for i in range(len(angles)):
            r = laser_msg.ranges[i]
            angle = laser_msg.angle_min + i * laser_msg.angle_increment

            if r < max_range:                
                x = robot_pose.x + r * math.cos(angle + robot_pose.z)
                y = robot_pose.y + r * math.sin(angle + robot_pose.z)
                hit_points.append([x, y])
    
                if math.isinf(x) or math.isinf(y) or math.isnan(x) or math.isnan(y):
                    hit_points = []
                    break
        
        return np.array(hit_points)
    

    def filter_outliers(self, laser_msg, polygon, center_pose, max_range):

        prev_pose = Point32()
        prev_pose.x = self.T_prev[0,3]
        prev_pose.y = self.T_prev[1,3]
        prev_pose.z = math.atan2(self.T_prev[1,0], self.T_prev[0,0])

        # current_hit_points = self.hit_points_from_laserScan(laser_msg, Point32(y=0.0))
        current_hit_points = self.hit_points_from_laserScan(laser_msg, prev_pose)
        print(f'Hit points: {len(current_hit_points)}')

        kd_tree = cKDTree(current_hit_points)
        center_point = [center_pose.x, center_pose.y]

        prefiltered_indices = kd_tree.query_ball_point(center_point, max_range)

        prefiltered_hit_points = current_hit_points[prefiltered_indices]
        
        prefiltered_hit_pts = pg.points((prefiltered_hit_points))

        buffer_polygon = pg.buffer(polygon, 0.1)

        mask = pg.contains(buffer_polygon, prefiltered_hit_pts)

        print('buffer_polygon:', buffer_polygon)    

        

        count = 0   
        for i in range(len(mask)):
            if mask[i]:
                count += 1
               
        print(f'Filtered points: {count}')

        filtered_hit_points = prefiltered_hit_points[mask]

        return filtered_hit_points

    

    def scan_matching(self, laser_msg):

        max_range = laser_msg.range_max

        observable_keyframes = self.localize_robot_in_keyframe_graph(self.estimated_pose)
        # print("BEFORE:")
        # print(f'prev_pose: {self.prev_pose.x}, {self.prev_pose.y}; \n estimated_pose: {self.estimated_pose.x}, {self.estimated_pose.y}')

        self.map_pcd = o3d.geometry.PointCloud()
        for keyframe_id in observable_keyframes:    
            self.map_pcd += self.keyframe_node_list[keyframe_id].map_pcd

        
        keyframe_center = self.keyframe_node_list[-1].get_center()
        keyframe_polygon = self.keyframe_node_list[-1].get_cloud_polygon()

        filtered_current_hit_points = self.filter_outliers(laser_msg, keyframe_polygon, keyframe_center, max_range)

        # Filter points that are max range away from the keyframe center
        map_pcd_msg = self.generate_point_cloud(self.keyframe_node_list[-1].get_hit_points())
        self.map_pcd_pub.publish(map_pcd_msg)


        current_pcd_msg = self.generate_point_cloud(filtered_current_hit_points)
        self.current_pcd_pub.publish(current_pcd_msg)        

        # time.sleep(1.0)

        if self.T_prev is not None and len(filtered_current_hit_points) > 0:
            
            hit_points_arr_3d = np.c_[filtered_current_hit_points, np.zeros(filtered_current_hit_points.shape[0])]

            current_pcd = o3d.geometry.PointCloud()
            current_pcd.points = o3d.utility.Vector3dVector(hit_points_arr_3d)
            
            icp_result = self.perform_icp(current_pcd, self.map_pcd, threshold=0.5, max_iteration=1000)
            
            T_delta_icp = icp_result.transformation

            current_transform = np.dot(self.T_prev, T_delta_icp)

            # self.get_logger().info(f'{colorCode["bright_red"]} T_prev:\n {self.T_prev}')

            # self.get_logger().info(f'{colorCode["bright_yellow"]} T_delta_icp:\n {T_delta_icp}')

            self.get_logger().info(f'{colorCode["bright_green"]} T_current:\n {current_transform}')
            
            new_hit_points = self.hit_points_from_laserScan(laser_msg, Point32(x=current_transform[0,3], y=current_transform[1,3], z=0.0))

            transformed_pcd_msg = self.generate_point_cloud(new_hit_points)
            self.transformed_pcd_pub.publish(transformed_pcd_msg)
            
            if icp_result.fitness > 0.9:
                self.T_prev = self.T_current.copy()
                self.T_current = current_transform.copy()

            else:
                self.get_logger().info(f'{colorCode["bright_red"]}ICP failed')
                
        # print("AFTER:")
        # print(f'prev_pose: {self.prev_pose.x}, {self.prev_pose.y}; \n estimated_pose: {self.estimated_pose.x}, {self.estimated_pose.y}')

    def perform_localization(self, pose_msg, laser_msg):
        self.robot_pose = self.get_pose_from_msg(pose_msg)

        # if self.node_count is not None:
        with self.callback_lock:
            if self.keyframe_node_list:
                    self.scan_matching(laser_msg)
                    # self.get_logger().info(f'{colorCode["cyan"]} Perform scan matching')
            else:
                self.get_logger().info(f'{colorCode["cyan"]}No keyframes')
                self.initial_pose = copy.deepcopy(self.robot_pose)
                self.T_prev = np.eye(4)
                self.T_current = np.eye(4)
                self.estimated_pose = Point32()

        self.estimated_pose.x = self.T_current[0,3]
        self.estimated_pose.y = self.T_current[1,3]
        self.estimated_pose.z = math.atan2(self.T_current[1,0], self.T_current[0,0])
        self.publish_estimated_pose()


def main(args=None):
    rclpy.init(args=args)
    node = Robot_Localization()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
if __name__ == '__main__':
    main()