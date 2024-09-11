import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_py_node')
        self.get_logger().info("Test Python")
        