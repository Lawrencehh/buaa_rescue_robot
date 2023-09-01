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

find_package(rclcpp REQUIRED)

add_executable(serial_sender src/serial_sender.cpp)
ament_target_dependencies(serial_sender rclcpp)  # removed asio

add_executable(serial_receiver src/serial_receiver.cpp)
ament_target_dependencies(serial_receiver rclcpp)


install(TARGETS
  serial_sender
  serial_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
