cmake_minimum_required(VERSION 3.8)
project(frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)  # Add sensor_msgs
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(libpointmatcher CONFIG REQUIRED) #For ICP
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(tf2_sensor_msgs REQUIRED)


# Include Cpp "include" directory
include_directories(include)
include_directories(include ${libpointmatcher_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src/frontend/include)
include_directories(${Python3_INCLUDE_DIRS})


# Create Cpp executable
add_executable(cpp_executable src/cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp )


# Create ICP executable
add_executable(icp_node src/icp_node.cpp)
ament_target_dependencies(icp_node rclcpp sensor_msgs libpointmatcher tf2 tf2_geometry_msgs tf2_ros)

target_link_libraries(icp_node ${Python3_LIBRARIES})

# Install Cpp executables
# install(TARGETS
#   cpp_executable
#   icp_node  # Ensure the ICP node is installed
#   DESTINATION lib/${PROJECT_NAME}
# )
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/py_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/main.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/laser_scan.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/odom_to_map.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/scan_to_map.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/map_create.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/pose_graph.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/pose_graph_odom.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/pose_graph_odom_landmark.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/plot_odom.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   scripts/two_landmark.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(PROGRAMS
  scripts/localize_with_arcuo.py
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/hypotheses_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
