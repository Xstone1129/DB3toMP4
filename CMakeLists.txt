cmake_minimum_required(VERSION 3.10)
project(db3tomp4)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# Include directories
include_directories(include)

# Executable
add_executable(db3tomp4_node src/db3tomp4.cpp)

# Link libraries
ament_target_dependencies(db3tomp4_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  rosbag2_cpp
  rosbag2_storage
)

# Install
install(TARGETS
  db3tomp4_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
