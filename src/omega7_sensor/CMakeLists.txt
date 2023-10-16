cmake_minimum_required(VERSION 3.8)
project(omega7_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# 找到Eigen库
find_package(Eigen3 REQUIRED)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(omega7_sensor src/omega7_sensor.cpp)
ament_target_dependencies(omega7_sensor rclcpp std_msgs)

# 设置rpath
set_target_properties(omega7_sensor PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)


# 添加Eigen库
target_link_libraries(omega7_sensor Eigen3::Eigen)

install(TARGETS
  omega7_sensor
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(omega7_sub src/omega7_sub.cpp)
ament_target_dependencies(omega7_sub rclcpp std_msgs)

install(TARGETS
  omega7_sub
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/omega7_sensor  
)

link_directories(
  ${catkin_LIB_DIRS} lib			# 加入链接库的目录
)

find_library(DHDLIB libdhd.so.3 lib)	# 设置动态链接库环境变量
find_library(DRDLIB libdrd.so.3 lib)
find_library(USB1LIB  libusb-1.0.so)

target_link_libraries(omega7_sensor		# 链接节点依赖的库
  ${DHDLIB}
  ${DRDLIB}
  ${USB1LIB}
  ${catkin_LIBRARIES}
)
ament_package()