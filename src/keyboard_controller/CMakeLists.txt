cmake_minimum_required(VERSION 3.5)
project(keyboard_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMessage.msg"
)

# Create executable
add_executable(keyboard_publisher src/keyboard_publisher.cpp)

# Link generated message files to the executable target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(keyboard_publisher ${cpp_typesupport_target})

# Add dependencies to the executable target
ament_target_dependencies(keyboard_publisher rclcpp std_msgs)

install(TARGETS
  keyboard_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
