from launch import LaunchDescription 
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the launch arguments for using NAV2

    # declare_x = DeclareLaunchArgument('x', default_value='1.0', description='X coordinate')
    # declare_y = DeclareLaunchArgument('y', default_value='2.0', description='Y coordinate')
    # declare_z = DeclareLaunchArgument('z', default_value='0.0', description='Z coordinate')
    # declare_qx = DeclareLaunchArgument('qx', default_value='0.0', description='Quaternion X')
    # declare_qy = DeclareLaunchArgument('qy', default_value='0.0', description='Quaternion Y')
    # declare_qz = DeclareLaunchArgument('qz', default_value='0.0', description='Quaternion Z')
    # declare_qw = DeclareLaunchArgument('qw', default_value='0.707', description='Quaternion W')

    # Arcuo Detection 
    aruco_marker_detection_node = Node(
        package = "arcuo_marker_detection",
        executable = "aruco_marker_detection_node",

    )

    # reach_the_goal_node = Node(

    #     package = "reach_the_goal",
    #     executable = "reach_the_goal_node",
    # )

    # Create the node and pass the parameters from the launch configuration for NAV2

    # reach_the_goal_node = Node(
    #     package="reach_the_goal",
    #     executable="reach_the_goal_node",
    #     parameters=[{
    #         'x': LaunchConfiguration('x'),
    #         'y': LaunchConfiguration('y'),
    #         'z': LaunchConfiguration('z'),
    #         'qx': LaunchConfiguration('qx'),
    #         'qy': LaunchConfiguration('qy'),
    #         'qz': LaunchConfiguration('qz'),
    #         'qw': LaunchConfiguration('qw'),
    #     }]
    # )

    #Start the simulation

    # simulation_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ros_gz_example_bringup'),
    #                     'launch/diff_drive.launch.py')
    # )
    # )


    ld.add_action(aruco_marker_detection_node)
    #NAV2 part Start
    # ld.add_action(declare_x)
    # ld.add_action(declare_y)
    # ld.add_action(declare_z)
    # ld.add_action(declare_qx)
    # ld.add_action(declare_qy)
    # ld.add_action(declare_qz)
    # ld.add_action(declare_qw)
    # ld.add_action(reach_the_goal_node)
    # ld.add_action(simulation_launch_file)
    #nav2 PART END

    return ld


