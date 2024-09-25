import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Bool # Handles boolean messages
from std_msgs.msg import Int32 # Handles int 32 type message
import sys
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped


# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }


class arcuo_marker_detection(Node): # Node class 
    def __init__(self):
        super().__init__('arcuo_marker_detection_node') #Node name 
        # self.publisher_aruco = self.create_publisher(Point, 'predicted_pallet_position', 10)
        # self.publisher_aruco_marker_detected = self.create_publisher(Bool, 'aruco_marker_detected', 10)
        self.subscription = self.create_subscription(
            Image,  #message type 
            '/diff_drive/camera_rgb',  #topic name
            self.arcuo_detection_callback, 
            10)
        
        self.publisher_marker_in_robot_frame = self.create_publisher(PointStamped, 'marker_in_robot_frame', 10)

        

        # Declare parameters
        self.declare_parameter("aruco_dictionary_name", "DICT_6X6_250")
        self.declare_parameter("aruco_marker_side_length", 0.05)
        self.declare_parameter("aruco_marker_name", "aruco_marker")


        self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
        aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
        self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value


        #Load camera matrix
        self.mtx = np.array([
            [476.7014503479004, 0.0, 400.0],
            [0.0, 476.7014503479004, 400.0],
            [0.0, 0.0, 1.0]
        ])
        self.dst = np.array([1e-08, 1e-08, 1e-08, 1e-08, 1e-08], dtype=np.float64)
        # Load the ArUco dictionary
        self.get_logger().info("[INFO] detecting '{}' markers...".format(aruco_dictionary_name))
        self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Initialize the transform broadcaster and TF buffer
        self.tfbroadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        
    def arcuo_detection_callback(self, msg: Image):
        
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')


        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg)

        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
        current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters,
        cameraMatrix=self.mtx, distCoeff=self.dst)


         # Check that at least one ArUco marker was detected
        if marker_ids is not None:

            self.get_logger().info('Marker id detected')
        
            # Draw a square around detected markers in the video frame
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_side_length,
                self.mtx,
                self.dst)
                
            # The pose of the marker is with respect to the camera lens frame.
            # Imagine you are looking through the camera viewfinder, 
            # the camera lens frame's:
            # x-axis points to the right
            # y-axis points straight down towards your toes
            # z-axis points straight ahead away from your eye, out of the camera
            for i, marker_id in enumerate(marker_ids): 

                self.get_logger().info('Publishing')   
      

                # Create the coordinate transform in the camera frame
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_link'
                t.child_frame_id = self.aruco_marker_name
            
                # Store the translation (i.e. position) information
                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]

                
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                
                # Quaternion format     
                t.transform.rotation.x = quat[0] 
                t.transform.rotation.y = quat[1] 
                t.transform.rotation.z = quat[2] 
                t.transform.rotation.w = quat[3] 

                # Log the rotation and translation vectors
                self.get_logger().info(f'Translation Vector {i}: x={t.transform.translation.x}, y={t.transform.translation.y}, z={t.transform.translation.z}')
                self.get_logger().info(f'Rotation Vector {i}: x={quat[0]}, y={quat[1]}, z={quat[2]}, w={quat[3]}')


                # Send the transform
                # self.tfbroadcaster.sendTransform(t)    
                

            # Transform to robot base frame using TF lookup
            try:
                # Lookup the transformation between 'diff_drive/lidar_link' and 'camera_link'
                transform = self.tf_buffer.lookup_transform('diff_drive/lidar_link', 'camera_link', rclpy.time.Time())
                
                # Create a PointStamped message to hold the marker position in the camera frame
                point_in_camera_frame = PointStamped()
                point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
                point_in_camera_frame.header.frame_id = 'camera_link'
                
                # Set the marker's position in the camera frame
                point_in_camera_frame.point.x = t.transform.translation.x
                point_in_camera_frame.point.y = t.transform.translation.y
                point_in_camera_frame.point.z = t.transform.translation.z

                # Perform the transformation to the robot's frame
                marker_in_robot_frame = tf2_geometry_msgs.do_transform_point(point_in_camera_frame, transform)

                # Log the transformed position in the robot's frame
                self.get_logger().info(f'Marker position in robot frame: x={marker_in_robot_frame.point.x}, y={marker_in_robot_frame.point.y}, z={marker_in_robot_frame.point.z}')
            
                # Publish the transformed marker position
                self.publisher_marker_in_robot_frame.publish(marker_in_robot_frame)
            
            except Exception as e:
                self.get_logger().error(f'Could not transform marker pose to robot frame: {e}')


                # Draw the axes on the marker
                cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05) 

        # Resize the frame to the desired size
        desired_width = 800
        desired_height = 800
        resized_frame = cv2.resize(current_frame, (desired_width, desired_height))
            
        # Display image for testing
        # cv2.imshow("camera", current_frame)
        cv2.imshow("camera", resized_frame)
        cv2.waitKey(1)
        

        # position_pallet_msg = Point()
        # position_pallet_msg.x = x_posteriori[0,0]
        # position_pallet_msg.y = x_posteriori[1,0]
        # point_msg.z = x_posteriori[2,0]  # Set to zero or another value as appropriate

        # # Publish the center position of Arcuo marker
        # self.publisher_aruco.publish(point_msg)



def main(args=None):
    rclpy.init(args=args)   # We start ROS2 communication 
    arcuo_marker_node = arcuo_marker_detection() #Create a node
    rclpy.spin(arcuo_marker_node)
    arcuo_marker_node.destroy_node()
    rclpy.shutdown() # Shutting down ROS2 communication

if __name__ == '__main__':
    main()