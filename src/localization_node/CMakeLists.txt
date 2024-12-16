cmake_minimum_required(VERSION 3.8)
project(localization_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)  # Add sensor_msgs
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(include)

# Create Cpp executable
add_executable(active_docking src/main.cpp src/active_amcl.cpp)
ament_target_dependencies(active_docking rclcpp )

# Install Cpp executables
install(TARGETS
  active_docking
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
