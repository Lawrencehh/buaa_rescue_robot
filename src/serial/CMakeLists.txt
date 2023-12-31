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


# 创建 serial_master_devices 可执行文件
add_executable(serial_master_devices src/serial_master_devices.cpp)
# 为 serial_master_devices 添加依赖
ament_target_dependencies(serial_master_devices rclcpp std_msgs buaa_rescue_robot_msgs) 

# 创建 serial_receiver 可执行文件
add_executable(serial_receiver src/serial_receiver.cpp)
# 为 serial_receiver 添加依赖
ament_target_dependencies(serial_receiver rclcpp std_msgs buaa_rescue_robot_msgs) 

# 添加serial_robomaster_1节点的编译指令
add_executable(serial_robomaster_1 src/serial_robomaster_1.cpp)
ament_target_dependencies(serial_robomaster_1 rclcpp std_msgs buaa_rescue_robot_msgs)

# 添加serial_robomaster_2节点的编译指令
add_executable(serial_robomaster_2 src/serial_robomaster_2.cpp)
ament_target_dependencies(serial_robomaster_2 rclcpp std_msgs buaa_rescue_robot_msgs)

# 添加pull_push_sensors_1节点的编译指令
add_executable(serial_PullPushSensors_1 src/serial_PullPushSensors_1.cpp)
ament_target_dependencies(serial_PullPushSensors_1 rclcpp std_msgs buaa_rescue_robot_msgs)

# 添加pull_push_sensors_2节点的编译指令
add_executable(serial_PullPushSensors_2 src/serial_PullPushSensors_2.cpp)
ament_target_dependencies(serial_PullPushSensors_2 rclcpp std_msgs buaa_rescue_robot_msgs)


# 安装目标
install(TARGETS
  serial_master_devices
  serial_receiver
  serial_robomaster_1
  serial_robomaster_2
  serial_PullPushSensors_1
  serial_PullPushSensors_2
  DESTINATION lib/${PROJECT_NAME}
)

# 打包
ament_package()
