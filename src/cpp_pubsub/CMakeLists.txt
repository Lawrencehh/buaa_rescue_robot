cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到ROS 2相关的CMake包和库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加talker节点
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# 添加listener节点
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# 安装所有可执行文件
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# 如果你有其他依赖或测试，添加它们
ament_package()
