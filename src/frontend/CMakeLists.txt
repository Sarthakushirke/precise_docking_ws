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

# Include Cpp "include" directory
include_directories(include)
include_directories(include ${libpointmatcher_INCLUDE_DIRS})

# Create Cpp executable
add_executable(cpp_executable src/cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp )


# Create ICP executable
# add_executable(icp_node src/icp_node.cpp)
# ament_target_dependencies(icp_node rclcpp sensor_msgs libpointmatcher)


# Install Cpp executables
install(TARGETS
  cpp_executable
  # icp_node  # Ensure the ICP node is installed
  DESTINATION lib/${PROJECT_NAME}
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/py_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_to_point.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/laser_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
