cmake_minimum_required(VERSION 3.8)
project(rosbag_vision)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(
    INSTALL_TO_SHARE
    launch
)
