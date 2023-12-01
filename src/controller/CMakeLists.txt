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
add_executable(auto_controller_1 src/auto_controller_1.cpp)
add_executable(auto_controller_2 src/auto_controller_2.cpp)

# Add dependencies to the executable target
ament_target_dependencies(auto_controller_1 rclcpp std_msgs buaa_rescue_robot_msgs)
ament_target_dependencies(auto_controller_2 rclcpp std_msgs buaa_rescue_robot_msgs)


install(TARGETS
  auto_controller_1
  auto_controller_2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
