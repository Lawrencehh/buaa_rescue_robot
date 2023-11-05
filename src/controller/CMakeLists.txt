cmake_minimum_required(VERSION 3.5)
project(controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(buaa_rescue_robot_msgs REQUIRED)

# Create executable
add_executable(keyboard_publisher src/keyboard_publisher.cpp)
add_executable(auto_controller src/auto_controller.cpp)

# Add dependencies to the executable target
ament_target_dependencies(keyboard_publisher rclcpp std_msgs buaa_rescue_robot_msgs)
ament_target_dependencies(auto_controller rclcpp std_msgs buaa_rescue_robot_msgs)


install(TARGETS
  keyboard_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
