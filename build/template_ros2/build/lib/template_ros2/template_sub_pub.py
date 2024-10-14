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




class template_node_detection(Node): # Node class 
    def __init__(self):
        super().__init__('arcuo_marker_detection_node') #Node name 
        # self.publisher_aruco = self.create_publisher(Point, 'predicted_pallet_position', 10)
        # self.publisher_aruco_marker_detected = self.create_publisher(Bool, 'aruco_marker_detected', 10)
        self.subscription = self.create_subscription(
            Image,  #message type 
            '/diff_drive/camera_rgb',  #topic name
            self.template_node_callback,  # Callback function
            10)

        
        
    def template_node_callback(self, msg):
        pass
        

def main(args=None):
    rclpy.init(args=args)   # We start ROS2 communication 
    template_node = template_node_detection() #Create a node
    rclpy.spin(template_node)
    template_node.destroy_node()
    rclpy.shutdown() # Shutting down ROS2 communication

if __name__ == '__main__':
    main()