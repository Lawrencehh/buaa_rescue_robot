cmake_minimum_required(VERSION 3.5)
project(buaa_rescue_robot_gui)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(std_msgs REQUIRED)
find_package(buaa_rescue_robot_msgs REQUIRED)

# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

# Wrap UI files
qt5_wrap_ui(UIC_FILES src/mainwindow.ui)

# Add executable
add_executable(buaa_rescue_robot_gui_node src/main.cpp src/mainwindow.cpp ${UIC_FILES})


# Add dependencies
ament_target_dependencies(buaa_rescue_robot_gui_node rclcpp std_msgs buaa_rescue_robot_msgs)

# Link libraries
target_link_libraries(buaa_rescue_robot_gui_node Qt5::Widgets)

# Include directories
target_include_directories(buaa_rescue_robot_gui_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

# 安装所有可执行文件
install(TARGETS
  buaa_rescue_robot_gui_node
  DESTINATION lib/${PROJECT_NAME})

# Debugging: Print UIC_FILES variable
message("UIC_FILES: ${UIC_FILES}")

# Package setup
ament_package()
