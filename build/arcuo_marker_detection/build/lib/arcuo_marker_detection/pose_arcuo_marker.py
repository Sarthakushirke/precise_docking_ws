'''
Welcome to the ArUco Marker Pose Estimator!
  
This program:
  - Estimates the pose of an ArUco Marker
'''
from __future__ import print_function
import sys # Python 2/3 compatibility
import cv2  # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library

 
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
import math

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
        self.declare_parameter("aruco_marker_side_length", 0.0785)
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
        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.this_aruco_dictionary, self.this_aruco_parameters)

        # Initialize the transform broadcaster and TF buffer
        self.tfbroadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()


    def insert_ids_corner(self,corners,ids):
        Detected_ArUco_markers = {}
        for i in range(len(ids)):
            Detected_ArUco_markers.update({ids[i][0]:corners[i]})
        return Detected_ArUco_markers
    
    def put_the_angle_on_frame(self,img,Detected_ArUco_markers,ArUco_marker_angles):
        for key in Detected_ArUco_markers:
            corners = Detected_ArUco_markers[key]
            tl = corners[0][0]	# top left
            tr = corners[0][1]	# top right
            br = corners[0][2]	# bottom right
            bl = corners[0][3]
            top = int((tl[0]+tr[0])//2), int((tl[1]+tr[1])//2)
            centre = int((tl[0]+tr[0]+bl[0]+br[0])//4), int((tl[1]+tr[1]+bl[1]+br[1])//4)
            img = cv2.putText(img, str(ArUco_marker_angles[key]), top, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
            return img
    

    def Calculate_orientation_in_degree(self,Detected_ArUco_markers):
        ArUco_marker_angles = {}
        for key in Detected_ArUco_markers:
            corners = Detected_ArUco_markers[key]
            tl = corners[0][0]	# top left
            tr = corners[0][1]	# top right
            br = corners[0][2]	# bottom right
            bl = corners[0][3]	# bottom left
            top = (tl[0]+tr[0])/2, -((tl[1]+tr[1])/2)
            centre = (tl[0]+tr[0]+bl[0]+br[0])/4, -((tl[1]+tr[1]+bl[1]+br[1])/4)
            try:
                    angle = round(math.degrees(np.arctan((top[1]-centre[1])/(top[0]-centre[0]))))
            except:
                # add some conditions for 90 and 270
                if(top[1]>centre[1]):
                    angle = 90
                elif(top[1]<centre[1]):
                    angle = 270
            if(top[0] >= centre[0] and top[1] < centre[1]):
                angle = 360 + angle
            elif(top[0]<centre[0]):
                angle = 180 + angle
            ArUco_marker_angles.update({key: angle})
        return ArUco_marker_angles
    
    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
            
        return roll_x, pitch_y, yaw_z # in radians

    def arcuo_detection_callback(self, msg: Image):
        
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')


        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg)

        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
        current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)


        if marker_ids is not None:

            self.get_logger().info('Marker id detected')

            Detected_ArUco_markers_val = self.insert_ids_corner(corners,marker_ids)
            anglevalue = self.Calculate_orientation_in_degree(Detected_ArUco_markers_val)
            print("Angle value",anglevalue)
            # Draw a square around detected markers in the video frame
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
            
            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_side_length,
                self.mtx,
                self.dst)
                
            # Print the pose for the ArUco marker
            # The pose of the marker is with respect to the camera lens frame.
            # Imagine you are looking through the camera viewfinder, 
            # the camera lens frame's:
            # x-axis points to the right
            # y-axis points straight down towards your toes
            # z-axis points straight ahead away from your eye, out of the camera
            for i, marker_id in enumerate(marker_ids):
            
                # Store the translation (i.e. position) information
                transform_translation_x = tvecs[i][0][0]
                transform_translation_y = tvecs[i][0][1]
                transform_translation_z = tvecs[i][0][2]
        
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                
                # Quaternion format     
                transform_rotation_x = quat[0] 
                transform_rotation_y = quat[1] 
                transform_rotation_z = quat[2] 
                transform_rotation_w = quat[3] 
                
                # Euler angle format in radians
                roll_x, pitch_y, yaw_z = self.euler_from_quaternion(transform_rotation_x, 
                                                            transform_rotation_y, 
                                                            transform_rotation_z, 
                                                            transform_rotation_w)
                
                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)
                print("transform_translation_x: {}".format(transform_translation_x))
                print("transform_translation_y: {}".format(transform_translation_y))
                print("transform_translation_z: {}".format(transform_translation_z))
                print("roll_x: {}".format(roll_x))
                print("pitch_y: {}".format(pitch_y))
                print("yaw_z: {}".format(yaw_z))
                print()
                
                # Draw the axes on the marker
                cv2.drawFrameAxes(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)
                frame = self.put_the_angle_on_frame(current_frame,Detected_ArUco_markers_val,anglevalue)
                tolerance = 80
                direction = "None"
                frame_center = int(640/2), int(480/2)
                start_point = int(640/2 - tolerance), int(480/2 - tolerance)
                end_point =  int(640/2 + tolerance), int(480/2 + tolerance)
                cv2.rectangle(frame, start_point, end_point, (255,0,0), 3)
                cv2.circle(frame, (int(640/2), int(480/2)), 4, (0, 0, 255), -1)

            # Transform to robot base frame using TF lookup
            try:
                # Lookup the transformation between 'diff_drive/lidar_link' and 'camera_link'
                transform = self.tf_buffer.lookup_transform('map', 'diff_drive/cam_link/camera1', rclpy.time.Time())
                
                # Create a PointStamped message to hold the marker position in the camera frame
                point_in_camera_frame = PointStamped()
                point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
                point_in_camera_frame.header.frame_id = 'diff_drive/cam_link/camera1'
                
                # Set the marker's position in the camera frame
                point_in_camera_frame.point.x = transform_translation_x
                point_in_camera_frame.point.y = transform_translation_y
                point_in_camera_frame.point.z = transform_translation_z

                # Perform the transformation to the robot's frame
                marker_in_robot_frame = tf2_geometry_msgs.do_transform_point(point_in_camera_frame, transform)

                # Extract x,y,z in robot/base frame
                x_robot = marker_in_robot_frame.point.x
                y_robot = marker_in_robot_frame.point.y
                z_robot = marker_in_robot_frame.point.z

                # # ========== COMPUTE DISTANCE AND ANGLE ==========
                # # 2D distance in the robot's XY-plane
                # distance = math.sqrt(x_robot**2 + y_robot**2)

                # # Angle (yaw) around Z, in degrees (optional: in radians)
                # # If you only care about angle in the horizontal plane:
                # angle = math.degrees(math.atan2(y_robot, x_robot))

                # # Log or publish the distance and angle
                # self.get_logger().info(
                #     f"Marker {marker_id} in robot frame => "
                #     f"x={x_robot:.3f}, y={y_robot:.3f}, z={z_robot:.3f} | "
                #     f"distance={distance:.3f} m, angle={angle:.1f} deg"
                # )

                # Log the transformed position in the robot's frame
                self.get_logger().info(f'Marker position in robot frame: x={x_robot}, y={y_robot}, z={z_robot}')
            
            

                # Publish the transformed marker position
                self.publisher_marker_in_robot_frame.publish(marker_in_robot_frame)
            
            except Exception as e:
                self.get_logger().error(f'Could not transform marker pose to robot frame: {e}')

 
def main(args=None):
    rclpy.init(args=args)   # We start ROS2 communication 
    arcuo_marker_node = arcuo_marker_detection() #Create a node
    rclpy.spin(arcuo_marker_node)
    arcuo_marker_node.destroy_node()
    rclpy.shutdown() # Shutting down ROS2 communication

if __name__ == '__main__':
    main()