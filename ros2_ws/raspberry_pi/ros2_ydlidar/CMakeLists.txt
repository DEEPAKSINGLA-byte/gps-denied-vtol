cmake_minimum_required(VERSION 3.8)
project(ydlidar_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find YDLIDAR-SDK
find_package(ydlidar_sdk REQUIRED)

add_executable(ydlidar_node src/ydlidar_node.cpp)
target_include_directories(ydlidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)
ament_target_dependencies(ydlidar_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(ydlidar_node
  ${YDLIDAR_SDK_LIBRARIES}
)

install(TARGETS
  ydlidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

# Install python script
install(PROGRAMS
  src/ydlidar_py_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
