cmake_minimum_required(VERSION 3.5)
project(serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section if you need asio
# find_package(asio REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(rosidl_default_generators REQUIRED)
find_package(buaa_rescue_robot_msgs REQUIRED)  # 已经添加了这个依赖


# 创建 serial_sender 可执行文件
add_executable(serial_sender src/serial_sender.cpp)
# 为 serial_sender 添加依赖
ament_target_dependencies(serial_sender rclcpp std_msgs buaa_rescue_robot_msgs) 

# 创建 serial_receiver 可执行文件
add_executable(serial_receiver src/serial_receiver.cpp)
# 为 serial_receiver 添加依赖
ament_target_dependencies(serial_receiver rclcpp std_msgs buaa_rescue_robot_msgs) 


# 安装目标
install(TARGETS
  serial_sender
  serial_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# 打包
ament_package()
