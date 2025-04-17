cmake_minimum_required(VERSION 3.10)
project(db3tomp4)

# 开启 ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件目录
include_directories(include)

# 添加可执行程序
ament_auto_add_executable(db3tomp4_node src/db3tomp4.cpp)

# 安装头文件（可选）
install(DIRECTORY include/
  DESTINATION include
)

ament_auto_package()
